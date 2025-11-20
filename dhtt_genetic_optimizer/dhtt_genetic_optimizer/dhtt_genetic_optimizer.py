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
from typing import List, Tuple, Callable, Iterable

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from dhtt_genetic_optimizer_msgs.srv import RunEval  # Provided by you

from deap import base, creator, tools, algorithms


# ---------------------------
# Helper: gene representation
# ---------------------------

def make_gene() -> Tuple[str, float]:
    """
    TODO: Implement your domain-specific gene creation here.
    Each gene must be a pair (str, float).

    Examples (replace with your logic):
      - return (random.choice(["a", "b", "c"]), random.uniform(0.0, 1.0))
      - return (param_name, param_value)

    For now, we provide a placeholder.
    """
    return (f"param_{random.randint(0, 9)}", random.uniform(0.0, 1.0))


def individual_to_file_args(individual: List[Tuple[str, float]]) -> List[str]:
    """
    Convert an individual (list of (key, value) pairs) into string arguments
    for the ROS service request field `file_args`.

    Customize this to match your evaluator's expectations.
    Example format: ["key=value", ...]
    """
    args = []
    for k, v in individual:
        # Format float consistently; adjust precision if needed.
        if isinstance(v, float) and (math.isinf(v) or math.isnan(v)):
            fv = "nan"
        else:
            fv = f"{v:.6g}"
        args.append(f"{k}={fv}")
    return args


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
        self._num_namespaces = self._declare_get_int('num_namespaces', 1)  # e.g., 2 -> /ns1, /ns2
        self._eval_service_name = self._declare_get_str('eval_service_name', 'eval/run')  # e.g., 'Eval_Server/run_eval'
        self._reset_tree = self._declare_get_bool('reset_tree', True)
        self._reset_level = self._declare_get_bool('reset_level', True)
        self._default_timeout_sec = self._declare_get_float('default_timeout_sec', 0.0)
        self._to_add = self._declare_get_str('to_add',
                                             '/IdeaProjects/CS776-dHTT/ros2_ws/src/dhtt_base/cooking_test/dhtt_cooking/test/experiment_descriptions/recipe_pasta_with_tomato_sauce.yaml')  # absolute path to YAML tree
        self._service_wait_timeout = self._declare_get_float('service_wait_timeout_sec', 5.0)

        # Create clients for each namespace, e.g., /ns1/<service>, /ns2/<service>, ...
        self._clients: List[rclpy.client.Client] = []
        for i in range(1, self._num_namespaces + 1):
            ns_prefix = f"/ns{i}"
            service_full = self._join_service(ns_prefix, self._eval_service_name)
            client = self._node.create_client(RunEval, service_full)
            self._clients.append(client)
            self._node.get_logger().info(f"[GA] Created client for service: {service_full}")

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
        self._pool = ThreadPoolExecutor(max_workers=self._num_namespaces)

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

    def _spin_loop(self):
        """Continuously spin the executor to process service responses."""
        self._node.get_logger().info("[GA] Spinner thread started.")
        while not self._stop_event.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                self._node.get_logger().error(f"[GA] Executor spin_once error: {e}")

    def _next_client(self) -> rclpy.client.Client:
        """Round-robin client selector."""
        with self._rr_lock:
            client = self._clients[self._rr_index % len(self._clients)]
            self._rr_index += 1
        return client

    def evaluate(self, individual: List[Tuple[str, float]]) -> Tuple[float]:
        """
        Evaluate one individual by calling RunEval service on the next client (round-robin).
        Returns (objective,) — here we use ticks_elapsed as the objective to minimize.
        """
        client = self._next_client()

        # Ensure service is available
        if not client.service_is_ready():
            ok = client.wait_for_service(timeout_sec=self._service_wait_timeout)
            if not ok:
                self._node.get_logger().warn("[GA] Service not available within timeout; penalizing fitness.")
                return (float('inf'),)

        # Build request
        req = RunEval.Request()
        req.reset_tree = self._reset_tree
        req.reset_level = self._reset_level
        req.timeout_sec = self._default_timeout_sec
        req.to_add = self._to_add
        # req.file_args = individual_to_file_args(individual)

        # Call async and wait for result
        future = client.call_async(req)
        # Wait until completed; executor spinner thread will progress the future
        while not future.done():
            time.sleep(0.01)

        try:
            resp = future.result()
        except Exception as e:
            self._node.get_logger().error(f"[GA] Service call exception: {e}")
            return (float('inf'),)

        if resp is None:
            self._node.get_logger().warn("[GA] Service returned None; penalizing fitness.")
            return (float('inf'),)

        # Use ticks_elapsed as the objective to minimize
        objective = float(resp.ticks_elapsed)

        # Optional: if success is False, you might want to penalize further
        if not getattr(resp, "success", True):
            self._node.get_logger().warn(f"[GA] Eval unsuccessful: {getattr(resp, 'message', '')}")
            # You can choose to add a penalty; for now, use objective as-is
            # Or: objective = float('inf')

        return (objective,)

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


# ---------------------------
# Mutation operator: swap two positions
# ---------------------------

def mut_swap(individual: List[Tuple[str, float]]) -> Tuple[List[Tuple[str, float]],]:
    """
    Swap mutation: randomly choose two distinct indices and swap their genes.
    Returns a tuple (individual,) as per DEAP's mutation convention.
    """
    size = len(individual)
    if size < 2:
        return (individual,)

    i, j = random.sample(range(size), 2)
    individual[i], individual[j] = individual[j], individual[i]
    return (individual,)


# ---------------------------
# GA setup and run
# ---------------------------

def run_ga(node: Node):
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
    IND_SIZE = int(node.get_parameter('individual_size').value)
    CX_PB = float(node.get_parameter('cx_prob').value)
    MUT_PB = float(node.get_parameter('mut_prob').value)
    TOUR_K = int(node.get_parameter('tournament_size').value)
    UNIFORM_INDPB = float(node.get_parameter('uniform_indpb').value)

    # DEAP: define Fitness/Individual
    if not hasattr(creator, "FitnessMin"):
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))  # MINIMIZE
    if not hasattr(creator, "Individual"):
        creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()

    # Attribute (gene) and individual/population initializers
    toolbox.register("gene", make_gene)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.gene, n=IND_SIZE)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # Genetic operators
    toolbox.register("select", tools.selTournament, tournsize=TOUR_K)
    toolbox.register("mate", tools.cxUniform, indpb=UNIFORM_INDPB)
    toolbox.register("mutate", mut_swap)

    # ROS eval pool & evaluation function
    eval_pool = RosEvalPool(node)

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
        f"[GA] Starting optimization: pop={POP_SIZE}, ngen={NGEN}, ind_size={IND_SIZE}, "
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
        node.get_logger().info(f"[GA] Best individual (key=value args): {individual_to_file_args(best)}")
    else:
        node.get_logger().warn("[GA] No best individual found.")

    return best


def main():
    rclpy.init()
    node = Node('ga_optimizer')

    # Optional parameter for the YAML tree path and default timeout, etc.
    # You can set these with ROS parameters, e.g.:
    #   ros2 run <your_pkg> ga_optimizer --ros-args -p to_add:=/abs/path/tree.yaml -p num_namespaces:=4 -p eval_service_name:=Eval_Server/run_eval
    # node.declare_parameter('to_add',
    #                        '/IdeaProjects/CS776-dHTT/ros2_ws/src/dhtt_base/cooking_test/dhtt_cooking/test/experiment_descriptions/recipe_pasta_with_tomato_sauce.yaml')
    # node.declare_parameter('default_timeout_sec', 0.0)
    # node.declare_parameter('reset_tree', True)
    # node.declare_parameter('reset_level', True)
    # node.declare_parameter('num_namespaces', 1)
    # node.declare_parameter('eval_service_name', 'eval/run')
    # node.declare_parameter('service_wait_timeout_sec', 5.0)
    node.declare_parameter('random_seed', 42)

    try:
        _ = run_ga(node)
    except KeyboardInterrupt:
        node.get_logger().info("[GA] Interrupted by user.")
    except Exception as e:
        node.get_logger().error(f"[GA] Exception: {e}")
    finally:
        # Clean-up
        node.get_logger().info("[GA] Shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
