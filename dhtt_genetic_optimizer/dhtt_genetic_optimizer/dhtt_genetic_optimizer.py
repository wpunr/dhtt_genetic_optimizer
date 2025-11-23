#!/usr/bin/python3

"""Chatbot"""

# !/usr/bin/env python3
"""
Genetic Algorithm (GA) boilerplate using DEAP + ROS 2 (Jazzy) service evaluation.

- Selection: Tournament
- Crossover: Uniform
- Mutation: Swap (swap two genes' positions)
- Representation: List of genes, each gene is a pair (str, float) — you will implement gene creation later.
- Evaluation: Calls ROS 2 service `dhtt_genetic_optimizer_msgs/srv/RunEval` to obtain an objective
             value to be MINIMIZED. This code uses `ticks_elapsed` as the objective.

Parallelization:
- Evaluations are dispatched in parallel (thread pool) using round-robin across multiple namespaces:
  `/ns1/...`, `/ns2/...`, etc. The number of namespaces and the service name are ROS parameters.

Important ROS 2 aspects:
- A MultiThreadedExecutor spins in a background thread to process service responses.
- Each evaluation is an async service call; we wait until the future is done.

You already have your ROS 2 package set up; place this file in your node/package and run it.
"""

import time
import math
import random
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Tuple, Callable, Iterable, Optional, Any
from dataclasses import dataclass
import yaml
import os
import uuid

import rclpy
import rcl_interfaces.srv
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import dhtt_msgs.msg
from dhtt_genetic_optimizer_msgs.srv import RunEval

from deap import base, creator, tools, algorithms

# Watchdog killer imports
import signal
import subprocess

try:
    import psutil

    _HAVE_PSUTIL = True
except Exception:
    _HAVE_PSUTIL = False

# ---------------------------
# Helper: gene representation
# ---------------------------

PLUGINS = ['dhtt_plugins::EfficiencyPotential', 'dhtt_plugins::ResourcePotential',
           'dhtt_genetic_optimizer::FixedPotential']

PARAM_NODE_NAMES = "FixedPotentialNodeNames"
PARAM_NODE_VALUES = "FixedPotentialValues"


@dataclass
class Gene:
    plugin: str  # in PLUGINS
    parameter: Optional[float] = None


class Individual:
    def __init__(self, genes: list[Gene], original_tree_path: str, target_type: int):
        self.genes = genes
        self.original_tree_path = original_tree_path
        self.target_type = target_type
        self.yaml_path = ""
        self.param_names = rclpy.parameter.Parameter(PARAM_NODE_NAMES, rclpy.Parameter.Type.STRING_ARRAY,
                                                     []).to_parameter_msg()
        self.param_vals = rclpy.parameter.Parameter(PARAM_NODE_VALUES, rclpy.Parameter.Type.DOUBLE_ARRAY,
                                                    []).to_parameter_msg()
        self.generate_tree()

    # def __del__(self):
    #     self._delete_yaml()

    # define len, and [] so the deap uniformcx works correctly
    def __len__(self):
        return self.genes.__len__()

    def __getitem__(self, item):
        return self.genes.__getitem__(item)

    def __setitem__(self, key, value):
        return self.genes.__setitem__(key, value)

    def _delete_yaml(self):
        # Delete old file if exists
        if self.yaml_path and os.path.exists(self.yaml_path):
            os.remove(self.yaml_path)

    def generate_tree(self) -> None:
        self._delete_yaml() # TODO just make the uuid file once

        # Load original YAML
        with open(self.original_tree_path, 'r') as f:
            tree = yaml.safe_load(f)

        fixed_plugin_names = list[str]()
        fixed_plugin_vals = list[float]()

        # Substitute plugins for nodes matching target_type
        gene_index = 0
        for node_name in tree['NodeList']:
            node = tree['Nodes'][node_name]
            if node.get('type') == self.target_type and gene_index < len(self.genes):
                node['potential_type'] = self.genes[gene_index].plugin
                if self.genes[gene_index].plugin == 'dhtt_genetic_optimizer::FixedPotential':
                    fixed_plugin_names.append(node_name)
                    fixed_plugin_vals.append(self.genes[gene_index].parameter)
                gene_index += 1
        self.param_names = rclpy.parameter.Parameter(PARAM_NODE_NAMES, rclpy.Parameter.Type.STRING_ARRAY,
                                                     fixed_plugin_names).to_parameter_msg()
        self.param_vals = rclpy.parameter.Parameter(PARAM_NODE_VALUES, rclpy.Parameter.Type.DOUBLE_ARRAY,
                                                    fixed_plugin_vals).to_parameter_msg()

        # Write new YAML to /tmp with unique name
        new_path = f"/tmp/{uuid.uuid4()}.yaml"
        with open(new_path, 'w') as f:
            yaml.dump(tree, f)

        self.yaml_path = new_path
        # print(f"Generated new tree at: {self.yaml_path}")


def make_gene() -> Gene:
    plugin = random.choice(PLUGINS)
    param = random.uniform(0.0, 1.0) if plugin == 'dhtt_genetic_optimizer::FixedPotential' else None
    return Gene(plugin, param)


def make_individual(original_tree_path: str, target_type: int) -> Individual:
    # Parse YAML to count nodes of target_type
    with open(original_tree_path, 'r') as f:
        tree = yaml.safe_load(f)

    count = sum(1 for node_name in tree['NodeList'] if tree['Nodes'][node_name].get('type') == target_type)

    # Generate required genes
    genes = [make_gene() for _ in range(count)]

    return Individual(genes, original_tree_path, target_type)


# ---------------------------
# ROS 2 service pool (round-robin, parallel)
# ---------------------------

class RosEvalPool:
    """
    Manages multiple RunEval service clients across namespaces and provides:
      - round-robin assignment of calls
      - parallel execution via thread pool
      - background spinning for futures
    """

    def __init__(self, node: Node):
        self._node = node

        # Parameters (declare defaults; override via ROS params)
        self._num_instances = self._declare_get_int('num_instances', 1)  # e.g., 2 -> /ns1, /ns2
        self._eval_service_name = self._declare_get_str('eval_service_name', 'eval/run')  # e.g., 'Eval_Server/run_eval'
        self._param_client_service_name = self._declare_get_str('param_client_service_name', 'ComAgg/set_parameters')
        self._reset_tree = self._declare_get_bool('reset_tree', True)
        self._reset_level = self._declare_get_bool('reset_level', True)
        self._default_timeout_sec = self._declare_get_float('default_timeout_sec', 20)
        self._to_add = self._declare_get_str('to_add',
                                             '/IdeaProjects/CS776-dHTT/ros2_ws/src/dhtt_base/cooking_test/dhtt_cooking/test/experiment_descriptions/recipe_pasta_with_tomato_sauce.yaml')  # absolute path to YAML tree
        self._file_args = self._declare_get_str_arr('file_args', [])
        self._service_wait_timeout = self._declare_get_float('service_wait_timeout_sec', 5.0)
        self._eval_max_retries = self._declare_get_int('eval_max_retries', 2)

        # Create clients for each namespace, e.g., /ns1/<service>, /ns2/<service>, ...
        self._clients: List[rclpy.client.Client] = []
        self._param_clients: List[rclpy.client.Client] = []
        for i in range(1, self._num_instances + 1):
            ns_prefix = f"/ns{i}"
            service_full = self._join_service(ns_prefix, self._eval_service_name)
            client = self._node.create_client(RunEval, service_full)
            self._clients.append(client)
            self._node.get_logger().info(f"[GA] Created client for service: {service_full}")

            param_client_service_full = self._join_service(ns_prefix, self._param_client_service_name)
            param_client = self._node.create_client(rcl_interfaces.srv.SetParameters, param_client_service_full)
            self._param_clients.append(param_client)
            self._node.get_logger().info(f"[GA] Created client for service: {param_client_service_full}")

        # Background spinner
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._stop_event = threading.Event()
        self._spinner_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spinner_thread.start()

        # Round-robin index
        self._rr_lock = threading.Lock()
        self._rr_index = 0

        # Thread pool size equal to number of namespaces
        self._pool = ThreadPoolExecutor(max_workers=self._num_instances)

        # per-namespace restart locks to avoid race conditions during restarts
        self._ns_restart_locks: List[threading.Lock] = [threading.Lock() for _ in range(self._num_instances)]

    def _join_service(self, namespace: str, service_name: str) -> str:
        """Make sure we have a valid fully-qualified service path."""
        ns = namespace.rstrip('/')
        svc = service_name.lstrip('/')
        return f"{ns}/{svc}"

    def _declare_get_int(self, name: str, default: int) -> int:
        self._node.declare_parameter(name, default)
        return int(self._node.get_parameter(name).value)

    def _declare_get_bool(self, name: str, default: bool) -> bool:
        self._node.declare_parameter(name, default)
        return bool(self._node.get_parameter(name).value)

    def _declare_get_float(self, name: str, default: float) -> float:
        self._node.declare_parameter(name, default)
        return float(self._node.get_parameter(name).value)

    def _declare_get_str(self, name: str, default: str) -> str:
        self._node.declare_parameter(name, default)
        return str(self._node.get_parameter(name).value)

    def _declare_get_str_arr(self, name: str, default: list[str]) -> list[str]:
        self._node.declare_parameter(name, default)
        return list[str](self._node.get_parameter(name).value)

    def _spin_loop(self):
        """Continuously spin the executor to process service responses."""
        self._node.get_logger().info("[GA] Spinner thread started.")
        while not self._stop_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                self._node.get_logger().error(f"[GA] Executor spin_once error: {e}")

    def _next_clients(self) -> tuple[rclpy.client.Client, rclpy.client.Client, int]:
        """Round-robin client selector, also returns index (0-based)."""
        with self._rr_lock:
            index = self._rr_index % len(self._clients)
            client = self._clients[index]
            param_client = self._param_clients[index]
            self._rr_index += 1
            return client, param_client, index

    def _tick_node(self, fut, end_time: int, tick_char='.', ticker_modulo: int = 100):
        ticker = 0
        while not fut.done():
            if self._node.get_clock().now().nanoseconds >= end_time:
                break
            time.sleep(0.01)
            if (ticker == 0):
                print(tick_char, end='', flush=True)
            ticker = (ticker + 1) % ticker_modulo
        print('')

    def _call_fixed_potential_service(self, param_client: rclpy.client.Client, individual: Individual):
        # Ensure param client is available
        if not param_client.service_is_ready():
            ok = param_client.wait_for_service(timeout_sec=self._service_wait_timeout)
            if not ok:
                self._node.get_logger().warn("[GA] Param Service not available within timeout; penalizing fitness.")

        req = rcl_interfaces.srv.SetParameters.Request()
        req.parameters = [individual.param_names, individual.param_vals]
        param_fut = param_client.call_async(req)

        # Wait until completed; executor spinner thread will progress the future
        self._tick_node(param_fut, end_time=self._node.get_clock().now().nanoseconds + int(1.0 * 1e9), tick_char='p',
                        ticker_modulo=2)

        try:
            resp = param_fut.result()
        except Exception as e:
            self._node.get_logger().error(f"[GA] Param Service call exception: {e}")
            return

        if resp is None:
            self._node.get_logger().warn("[GA] Param Service returned None")

    def _restart_namespace(self, ns_index_0_based: int, grace_sec: float = 0.8) -> None:
        ns_num = ns_index_0_based + 1
        ns_token = f"__ns:=/ns{ns_num}"
        killed_pids = []

        if _HAVE_PSUTIL:
            for p in psutil.process_iter(["pid", "cmdline"]):
                try:
                    cmd = " ".join(p.info.get("cmdline") or [])
                    if ns_token in cmd:
                        p.send_signal(signal.SIGTERM)
                        killed_pids.append(p.pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            time.sleep(grace_sec)
            for pid in killed_pids:
                try:
                    psutil.Process(pid).kill()
                except Exception:
                    pass
        else:
            subprocess.run(["pkill", "-TERM", "-f", ns_token], check=False)
            time.sleep(grace_sec)
            subprocess.run(["pkill", "-KILL", "-f", ns_token], check=False)

        self._node.get_logger().warn(f"[GA] Restarted namespace /ns{ns_num}; waiting for services…")

        eval_client = self._clients[ns_index_0_based]
        param_client = self._param_clients[ns_index_0_based]
        deadline = time.time() + (self._service_wait_timeout * 4.0)

        while time.time() < deadline:
            ok_eval = eval_client.wait_for_service(timeout_sec=0.5)
            ok_param = param_client.wait_for_service(timeout_sec=0.5)
            if ok_eval and ok_param:
                break
            time.sleep(0.1)

    def _eval_once(self, client: rclpy.client.Client, req: RunEval.Request) -> Tuple[float, bool, bool]:
        """:return (objective, did_timeout, failtimeout_flag)"""
        future = client.call_async(req)
        end_time = self._node.get_clock().now().nanoseconds + int(req.timeout_sec * 1e9 * 1.02)
        self._tick_node(future, end_time=end_time, tick_char='e', ticker_modulo=10)

        if not future.done():
            return (float('inf'), True, False)

        try:
            resp = future.result()
        except Exception as e:
            self._node.get_logger().error(f"[GA] Service call exception: {e}")
            return (float('inf'), False, False)

        if resp is None:
            self._node.get_logger().warn("[GA] Service returned None; penalizing fitness.")
            return (float('inf'), False, False)

        failtimeout = (
                not getattr(resp, "success", True)
                and getattr(resp, "message", "") == RunEval.Response.FAILTIMEOUT
        )
        if failtimeout:
            self._node.get_logger().warn("[GA] Server reported FAILTIMEOUT.")
            return (float('inf'), False, True)

        objective = float(resp.ticks_elapsed)
        return (objective, False, False)

    def evaluate(self, individual: Individual) -> Tuple[float]:
        """
        Evaluate one Individual by calling RunEval asynchronously on the next namespace client.
        If the async call times out/hangs or returns FAILTIMEOUT, perform:
          eval -> (kill namespace, wait, reapply params) -> retry
        for a configurable number of retries (ROS param: eval_max_retries).
        """
        client, param_client, ns_index = self._next_clients()
        ns_str = f"/ns{ns_index + 1}"

        # Build request once
        req = RunEval.Request()
        req.reset_tree = self._reset_tree
        req.reset_level = self._reset_level
        req.timeout_sec = self._default_timeout_sec
        req.to_add = individual.yaml_path
        req.file_args = self._file_args

        total_attempts = self._eval_max_retries + 1  # initial attempt + retry count

        for attempt in range(total_attempts):
            # Apply FixedPotential parameters before all attempts
            self._call_fixed_potential_service(param_client, individual)

            # For attempt 0, no restart; for subsequent attempts, restart first
            if attempt != 0:
                # Serialize restarts per namespace to avoid races
                with self._ns_restart_locks[ns_index]:
                    self._node.get_logger().warn(f"[GA] Restarting {ns_str} before retry {attempt}/{total_attempts}…")
                    self._restart_namespace(ns_index)
                    self._call_fixed_potential_service(param_client, individual)

            # Ensure service is available
            if not client.service_is_ready():
                ok = client.wait_for_service(timeout_sec=self._service_wait_timeout)
                if not ok:
                    self._node.get_logger().warn(
                        f"[GA] {ns_str} service not available; attempt {attempt + 1}/{total_attempts} penalized.")
                    continue  # proceed to next attempt (or exit if out of retries)

            # Perform the evaluation attempt
            objective, did_timeout, failtimeout_flag = self._eval_once(client, req)
            if not did_timeout and not failtimeout_flag:
                # Success
                return (objective,)

            # Log and continue
            reason = "async timeout/hang" if did_timeout else "server reported FAILTIMEOUT"
            self._node.get_logger().warn(
                f"[GA] Eval in {ns_str} failed ({reason}); attempt {attempt + 1}/{total_attempts}."
            )

        # All attempts failed
        self._node.get_logger().warn(f"[GA] All {total_attempts} attempts in {ns_str} failed; penalizing fitness.")
        return (float('inf'),)

    def parallel_map(self, func: Callable, iterable: Iterable):
        """
        A parallel map for DEAP. Dispatch tasks to the thread pool.
        DEAP will call this to evaluate a population.
        """
        futures = [self._pool.submit(func, item) for item in iterable]
        results = []
        for f in futures:
            results.append(f.result())
        return results

    def shutdown(self):
        """Clean shutdown of pool and spinner."""
        self._stop_event.set()
        try:
            self._spinner_thread.join(timeout=2.0)
        except Exception:
            pass
        self._pool.shutdown(wait=True)
        # Executor/node shutdown handled by main

    @property
    def to_add(self):
        return self._to_add


# ---------------------------
# Mutation operator: swap two positions
# ---------------------------

# type creator.Individual, not my Individual
def mut_swap(individual) -> Tuple[Any,]:
    """
    Swap mutation: randomly choose two distinct indices and swap their genes.
    Returns a tuple (individual,) as per DEAP's mutation convention.
    """
    size = len(individual.genes)
    if size < 2:
        return (individual,)

    i, j = random.sample(range(size), 2)
    individual.genes[i], individual.genes[j] = individual.genes[j], individual.genes[i]
    individual.generate_tree()

    return (individual,)


# ---------------------------
# GA setup and run
# ---------------------------

def run_ga(node: Node):
    # ROS eval pool & evaluation function
    eval_pool = RosEvalPool(node)

    random.seed(node.get_parameter('random_seed').value if node.has_parameter('random_seed') else 42)

    # GA parameters (ROS params with sane defaults)
    node.declare_parameter('population_size', 4)
    node.declare_parameter('num_generations', 4)
    node.declare_parameter('individual_size', 10)
    node.declare_parameter('cx_prob', 0.8)
    node.declare_parameter('mut_prob', 0.2)
    node.declare_parameter('tournament_size', 3)
    node.declare_parameter('uniform_indpb', 0.5)

    POP_SIZE = int(node.get_parameter('population_size').value)
    NGEN = int(node.get_parameter('num_generations').value)
    CX_PB = float(node.get_parameter('cx_prob').value)
    MUT_PB = float(node.get_parameter('mut_prob').value)
    TOUR_K = int(node.get_parameter('tournament_size').value)
    UNIFORM_INDPB = float(node.get_parameter('uniform_indpb').value)

    # DEAP: define Fitness/Individual
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", Individual, fitness=creator.FitnessMin)

    def build_individual_factory(original_tree_path: str, target_type: int):
        def _factory():
            individual = make_individual(original_tree_path, target_type)
            return creator.Individual(individual.genes, individual.original_tree_path, individual.target_type)

        return _factory

    toolbox = base.Toolbox()

    # Attribute (gene) and individual/population initializers
    toolbox.register("individual", build_individual_factory(eval_pool.to_add, dhtt_msgs.msg.Node.BEHAVIOR))
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # Genetic operators
    toolbox.register("select", tools.selTournament, tournsize=TOUR_K)
    toolbox.register("mate", tools.cxUniform, indpb=UNIFORM_INDPB)
    toolbox.register("mutate", mut_swap)

    def _evaluate(ind):
        return eval_pool.evaluate(ind)

    toolbox.register("evaluate", _evaluate)
    toolbox.register("map", eval_pool.parallel_map)  # let DEAP parallelize evaluations

    # Stats & Hall of Fame
    stats = tools.Statistics(lambda ind: ind.fitness.values[0])
    stats.register("avg", lambda xs: float(sum(xs) / len(xs)) if xs else float('nan'))
    stats.register("min", min)
    stats.register("max", max)

    hof = tools.HallOfFame(maxsize=1)

    # Initialize population
    population = toolbox.population(n=POP_SIZE)

    node.get_logger().info(
        f"[GA] Starting optimization: pop={POP_SIZE}, ngen={NGEN}, "
        f"cx_pb={CX_PB}, mut_pb={MUT_PB}, tourn={TOUR_K}, indpb={UNIFORM_INDPB}"
    )

    # Run GA
    try:
        population, logbook = algorithms.eaSimple(
            population=population,
            toolbox=toolbox,
            cxpb=CX_PB,
            mutpb=MUT_PB,
            ngen=NGEN,
            stats=stats,
            halloffame=hof,
            verbose=True
        )
    finally:
        # Ensure eval pool is properly shut down
        eval_pool.shutdown()

    # Report best
    best = hof[0] if len(hof) > 0 else None
    if best is not None:
        node.get_logger().info(f"[GA] Best fitness: {best.fitness.values[0]:.6g}")
    else:
        node.get_logger().warn("[GA] No best individual found.")

    return best


def main():
    rclpy.init()
    node = Node('ga_optimizer')

    # Optional parameter for the YAML tree path and default timeout, etc.
    # You can set these with ROS parameters, e.g.:
    #   ros2 run <your_pkg> ga_optimizer --ros-args -p to_add:=/abs/path/tree.yaml -p num_instances:=4 -p eval_service_name:=Eval_Server/run_eval
    # node.declare_parameter('to_add',
    #                        '/IdeaProjects/CS776-dHTT/ros2_ws/src/dhtt_base/cooking_test/dhtt_cooking/test/experiment_descriptions/recipe_pasta_with_tomato_sauce.yaml')
    # node.declare_parameter('default_timeout_sec', 0.0)
    # node.declare_parameter('reset_tree', True)
    # node.declare_parameter('reset_level', True)
    # node.declare_parameter('num_instances', 1)
    # node.declare_parameter('eval_service_name', 'eval/run')
    # node.declare_parameter('service_wait_timeout_sec', 5.0)
    node.declare_parameter('random_seed', 42)

    try:
        _ = run_ga(node)
    except KeyboardInterrupt:
        node.get_logger().info("[GA] Interrupted by user.")
    finally:
        # Clean-up
        node.get_logger().info("[GA] Shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
