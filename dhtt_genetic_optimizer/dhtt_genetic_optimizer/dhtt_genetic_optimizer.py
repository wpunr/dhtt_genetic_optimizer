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
import datetime
import copy
import pathlib

import rclpy
import rcl_interfaces.srv
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import rclpy.callback_groups

import numpy as np

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
        self.tree_data = None  # a dict

    # def __del__(self):
    #     self._delete_yaml()

    # define len, and [] so the deap uniformcx works correctly
    def __len__(self):
        return self.genes.__len__()

    def __getitem__(self, item):
        return self.genes.__getitem__(item)

    def __setitem__(self, key, value):
        return self.genes.__setitem__(key, value)

    # Helpers to unroll nested yaml files

    @staticmethod
    def _parse_params(params_list):
        """
        Convert the 'params' list (e.g., ['mark: A#', 'object: Plate']) into a dict.
        Supports either strings 'key: value' or direct YAML mappings.
        """
        result = {}
        if not params_list:
            return result
        for item in params_list:
            if isinstance(item, str):
                # Split only on the first colon
                key, sep, val = item.partition(':')
                if sep:
                    result[key.strip()] = val.strip()
                else:
                    # If no colon, treat entire string as a key with empty value.
                    result[item.strip()] = ''
            elif isinstance(item, dict):
                for k, v in item.items():
                    result[str(k)] = str(v)
            else:
                # Unsupported item type—ignore gracefully
                pass
        return result

    @staticmethod
    def _subst_str(s, subs):
        """
        Replace all occurrences of $key with subs[key] in a string.
        """
        if not isinstance(s, str) or not subs:
            return s
        for k, v in subs.items():
            s = s.replace(f'${k}', v)
        return s

    @staticmethod
    def _deep_substitute(obj, subs):
        """
        Recursively apply variable substitution to strings across a Python structure.
        """
        if isinstance(obj, str):
            return Individual._subst_str(obj, subs)
        elif isinstance(obj, list):
            return [Individual._deep_substitute(x, subs) for x in obj]
        elif isinstance(obj, dict):
            return {k: Individual._deep_substitute(v, subs) for k, v in obj.items()}
        else:
            return obj

    @staticmethod
    def _namespace_nodes(sub_nodes, prefix):
        """
        Namespace all node names under 'prefix' to avoid collisions.
        Returns:
          new_nodes: dict with namespaced keys
          name_map:  dict { old_name: namespaced_name }
        Also updates 'parent' references that point to nodes inside the subtree.
        """
        name_map = {old: f"{prefix}::{old}" for old in sub_nodes.keys()}
        new_nodes = {}
        for old_name, node in sub_nodes.items():
            node_copy = copy.deepcopy(node)
            parent = node_copy.get('parent')
            if isinstance(parent, str) and parent in name_map:
                node_copy['parent'] = name_map[parent]
            new_nodes[name_map[old_name]] = node_copy
        return new_nodes, name_map

    @staticmethod
    def _load_yaml_file(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    @staticmethod
    def _expand_once(data, current_dir):
        """
        Expand exactly one SUBTREE (type=5) occurrence in 'data' (if present).
        Returns a tuple (expanded_data, changed, next_dir), where:
          - expanded_data: updated YAML dict
          - changed: True if an expansion happened, else False
          - next_dir: the directory to use for subsequent nested expansions
        We expand based on the first SUBTREE occurrence found in NodeList order.
        """
        node_list = data.get('NodeList', [])
        nodes = data.get('Nodes', {})

        for idx, node_name in enumerate(node_list):
            node_def = nodes.get(node_name)
            if not node_def:
                continue

            if int(node_def.get('type', -1)) != dhtt_msgs.msg.Node.SUBTREE:
                continue  # Not a subtree node, skip

            include_rel = node_def.get('behavior_type')
            if not include_rel or not isinstance(include_rel, str):
                raise ValueError(f"SUBTREE node '{node_name}' has invalid behavior_type: {include_rel}")

            include_path = os.path.join(current_dir, include_rel)
            if not os.path.exists(include_path):
                raise FileNotFoundError(f"Included YAML not found: {include_path}")

            # Load included subtree YAML
            sub_data = Individual._load_yaml_file(include_path)
            if not isinstance(sub_data, dict):
                raise ValueError(f"Included YAML at {include_path} is not a mapping/dict.")

            sub_node_list = sub_data.get('NodeList', [])
            sub_nodes = copy.deepcopy(sub_data.get('Nodes', {}))

            # Parse variables from 'params' and substitute them through the subtree
            subs = Individual._parse_params(node_def.get('params', []))
            sub_nodes = Individual._deep_substitute(sub_nodes, subs)
            sub_node_list = Individual._deep_substitute(sub_node_list, subs)

            # Attach subtree root(s) to the including node's parent
            include_parent = node_def.get('parent')
            for n, nd in sub_nodes.items():
                # The highest node in the lower YAML has parent 'NONE' -> becomes including node's parent
                if str(nd.get('parent', '')).strip() == 'NONE':
                    nd['parent'] = include_parent

            # Namespace the subtree nodes under the including node's name
            namespaced_nodes, name_map = Individual._namespace_nodes(sub_nodes, node_name)
            namespaced_node_list = [name_map.get(n, n) for n in sub_node_list]

            # Splice into main: replace the subtree placeholder slot with the expanded nodes
            new_node_list = node_list[:idx] + namespaced_node_list + node_list[idx + 1:]
            new_nodes = copy.deepcopy(nodes)
            # Remove the placeholder node
            if node_name in new_nodes:
                del new_nodes[node_name]
            # Add the expanded namespaced nodes
            for k, v in namespaced_nodes.items():
                new_nodes[k] = v

            new_data = {
                'NodeList': new_node_list,
                'Nodes': new_nodes
            }

            # Next dir is the dir of the included YAML (for nested includes)
            next_dir = os.path.dirname(include_path)

            return new_data, True, next_dir

        # No subtree nodes found
        return data, False, current_dir

    @staticmethod
    def expand_all_subtrees(data, base_dir):
        """
        Recursively expand all SUBTREE nodes in the YAML 'data'.
        base_dir: directory of the main YAML, used to resolve includes.
        Returns a new dict with all subtrees unrolled.
        """
        current_dir = base_dir
        while True:
            data, changed, current_dir = Individual._expand_once(data, current_dir)
            if not changed:
                break
            # After each expansion we loop again, allowing nested expansions to resolve.
        return data

    @staticmethod
    def unroll_subtrees_file(main_yaml_path):
        """
        Load the main YAML file, recursively unroll all subtrees, and
        return the final combined YAML as a Python dict.
        """
        base_dir = os.path.dirname(os.path.abspath(main_yaml_path))
        data = Individual._load_yaml_file(main_yaml_path)
        if not isinstance(data, dict) or 'NodeList' not in data or 'Nodes' not in data:
            raise ValueError("Main YAML must contain 'NodeList' and 'Nodes' keys.")
        return Individual.expand_all_subtrees(data, base_dir)

    @staticmethod
    def dump_yaml(data, path):
        """
        Dump the Python dict back to YAML, preserving insertion order.
        """
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def delete_yaml(self):
        # Delete old file if exists
        if self.yaml_path and os.path.exists(self.yaml_path):
            os.remove(self.yaml_path)

    def generate_tree(self) -> None:
        """Expected to be called at eval time, that is, after recombination"""

        # Unroll original YAML
        tree = Individual.unroll_subtrees_file(self.original_tree_path)

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
        self.yaml_path = f"/tmp/{uuid.uuid4()}.yaml"
        with open(self.yaml_path, 'w') as f:
            yaml.dump(tree, f)
        self.tree_data = tree

        # print(f"Generated new tree at: {self.yaml_path}")


def make_gene() -> Gene:
    plugin = random.choice(PLUGINS)
    # activation potential should be (0, 1.0], this is close enough
    param = random.uniform(0.1, 1.0) if plugin == 'dhtt_genetic_optimizer::FixedPotential' else None
    return Gene(plugin, param)


def make_individual(original_tree_path: str, target_type: int) -> Individual:
    tree = Individual.unroll_subtrees_file(original_tree_path)

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
        self._get_param_client_service_name = self._declare_get_str('get_param_client_service_name',
                                                                    'ComAgg/get_parameters')
        self._reset_tree = self._declare_get_bool('reset_tree', True)
        self._reset_level = self._declare_get_bool('reset_level', True)
        self._default_timeout_sec = self._declare_get_float('default_timeout_sec', 20)
        self._heartbeat_sec = self._declare_get_float('heartbeat_sec', 5)
        self._to_add = self._declare_get_str('to_add',
                                             '/ros2_ws/src/dhtt_base/cooking_test/dhtt_cooking/test/experiment_descriptions/recipe_pasta_with_tomato_sauce.yaml')  # absolute path to YAML tree
        self._file_args = self._declare_get_str_arr('file_args', [])
        self._service_wait_timeout = self._declare_get_float('service_wait_timeout_sec', 5.0)
        self._eval_max_retries = self._declare_get_int('eval_max_retries', 2)

        # Create clients for each namespace, e.g., /ns1/<service>, /ns2/<service>, ...
        self.cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._clients: List[rclpy.client.Client] = []
        self._param_clients: List[rclpy.client.Client] = []
        self._get_param_clients: List[rclpy.client.Client] = []
        self._last_heartbeats: List[datetime.datetime] = []
        self._root_status_subs: List[rclpy.subscriber.Subscriber] = []
        for i in range(1, self._num_instances + 1):
            ns_prefix = f"/ns{i}"
            service_full = self._join_service(ns_prefix, self._eval_service_name)
            client = self._node.create_client(RunEval, service_full, callback_group=self.cb_group)
            self._clients.append(client)
            self._node.get_logger().info(f"[GA] Created client for service: {service_full}")

            param_client_service_full = self._join_service(ns_prefix, self._param_client_service_name)
            param_client = self._node.create_client(rcl_interfaces.srv.SetParameters, param_client_service_full,
                                                    callback_group=self.cb_group)
            self._param_clients.append(param_client)
            self._node.get_logger().info(f"[GA] Created client for service: {param_client_service_full}")

            get_param_client_service_full = self._join_service(ns_prefix, self._get_param_client_service_name)
            get_param_client = self._node.create_client(rcl_interfaces.srv.GetParameters, get_param_client_service_full,
                                                        callback_group=self.cb_group)
            self._get_param_clients.append(get_param_client)
            self._node.get_logger().info(f"[GA] Created client for service: {get_param_client_service_full}")

            status_topic_full = self._join_service(ns_prefix, 'root_status')
            root_status_cb = lambda _: self._last_heartbeats.__setitem__(i - 1, datetime.datetime.now())
            root_status_subscriber = self._node.create_subscription(dhtt_msgs.msg.NodeStatus, status_topic_full,
                                                                    root_status_cb, 10, callback_group=self.cb_group)
            self._last_heartbeats.append(datetime.datetime.now())
            self._root_status_subs.append(root_status_subscriber)

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
        self._ns_locks: List[threading.Lock] = [threading.Lock() for _ in range(self._num_instances)]

        self.slowest_successful_wall_time: datetime.timedelta | None = None

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

    def _next_clients(self) -> tuple[rclpy.client.Client, rclpy.client.Client, rclpy.client.Client, int]:
        """Round-robin client selector, also returns index (0-based)."""
        with self._rr_lock:
            index = self._rr_index % len(self._clients)
            client = self._clients[index]
            param_client = self._param_clients[index]
            get_param_client = self._get_param_clients[index]
            self._rr_index += 1
            return client, param_client, get_param_client, index

    def _tick_node(self, fut, end_time: int, tick_char='.', ticker_modulo: int = 100, check_heartbeat=False,
                   heartbeat_index: int | None = None):
        ticker = 0
        while not fut.done():
            if self._node.get_clock().now().nanoseconds >= end_time:
                break
            if (check_heartbeat and heartbeat_index is not None) and self._last_heartbeats[heartbeat_index] and (
                    datetime.datetime.now() - self._last_heartbeats[
                heartbeat_index]).total_seconds() > self._heartbeat_sec:
                print("heartbeat check failed")
                break
            time.sleep(0.01)
            if (ticker == 0):
                pass  # print(tick_char, end='', flush=True)
            ticker = (ticker + 1) % ticker_modulo
        # print('')

    def _call_fixed_potential_service(self, param_client: rclpy.client.Client, get_param_client: rclpy.client.Client,
                                      individual: Individual) -> bool:
        # Ensure param client is available
        if not param_client.service_is_ready():
            ok = param_client.wait_for_service(timeout_sec=self._service_wait_timeout)
            if not ok:
                self._node.get_logger().warn("[GA] Param Service not available within timeout; penalizing fitness.")
        if not get_param_client.service_is_ready():
            ok = get_param_client.wait_for_service(timeout_sec=self._service_wait_timeout)
            if not ok:
                self._node.get_logger().warn("[GA] Get Param Service not available within timeout; penalizing fitness.")

        # Send set parameters request
        req = rcl_interfaces.srv.SetParameters.Request()
        req.parameters = [individual.param_names, individual.param_vals]
        param_fut = param_client.call_async(req)

        # Wait until completed; executor spinner thread will progress the future
        # self._node.get_logger().fatal(
        #     f"[GA] sending parameters {individual.param_names.value.string_array_value} and {individual.param_vals.value.double_array_value}")
        self._tick_node(param_fut, end_time=self._node.get_clock().now().nanoseconds + int(1.0 * 1e9), tick_char='p',
                        ticker_modulo=2)

        try:
            resp = param_fut.result()
        except Exception as e:
            self._node.get_logger().error(f"[GA] Param Service call exception: {e}")
            return False

        if resp is None:
            self._node.get_logger().error("[GA] Param Service returned None")
            return False

        # Check that parameters were received correctly
        get_req = rcl_interfaces.srv.GetParameters.Request()
        get_req.names = [PARAM_NODE_NAMES, PARAM_NODE_VALUES]
        get_param_fut = get_param_client.call_async(get_req)
        # self._node.get_logger().fatal(
        #     f"[GA] getting parameters {get_req.names}")
        self._tick_node(get_param_fut, end_time=self._node.get_clock().now().nanoseconds + int(1.0 * 1e9),
                        tick_char='g',
                        ticker_modulo=2)

        try:
            get_resp = get_param_fut.result()
        except Exception as e:
            self._node.get_logger().error(f"[GA] Get Param Service call exception: {e}")
            return False

        if get_resp is None:
            self._node.get_logger().error("[GA] Get Param Service returned None")
            return False

        get_param_names_result = get_param_fut.result().values[0].string_array_value
        get_param_vals_result = get_param_fut.result().values[1].double_array_value

        if get_param_names_result != req.parameters[0].value.string_array_value or get_param_vals_result != \
                req.parameters[1].value.double_array_value:
            self._node.get_logger().error("[GA] Params were set incorrectly")
            raise RuntimeError("look at me!")
            return False

        return True

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
                return
            time.sleep(0.1)
        raise RuntimeError("Restarted ns and services didn't come up")

    def _eval_once(self, client: rclpy.client.Client, req: RunEval.Request, ns_index: int) -> Tuple[
        float, bool, bool, bool]:
        """:return (objective, did_timeout, failtimeout_flag, general_fail)"""
        start_wall_time = datetime.datetime.now()
        future = client.call_async(req)
        end_time = self._node.get_clock().now().nanoseconds + int(req.timeout_sec * 1e9 * 1.02)
        self._tick_node(future, end_time=end_time, tick_char='e', ticker_modulo=10, check_heartbeat=False,
                        heartbeat_index=ns_index)  # leave heartbeat disabled for now

        if not future.done():
            return (float('inf'), True, False, True)

        try:
            resp = future.result()
        except Exception as e:
            self._node.get_logger().error(f"[GA] Service call exception: {e}")
            return (float('inf'), False, False, True)

        if resp is None:
            self._node.get_logger().warn("[GA] Service returned None; penalizing fitness.")
            return (float('inf'), False, False, True)

        failtimeout = (
                not getattr(resp, "success", True)
                and getattr(resp, "message", "") == RunEval.Response.FAILTIMEOUT
        )
        if failtimeout:
            self._node.get_logger().warn("[GA] Server reported FAILTIMEOUT.")
            return (float('inf'), False, True, True)

        if not resp.success or resp.ticks_elapsed <= 0:  # failed for some other reason
            self._node.get_logger().warn(f"[GA] General fail, Server reported {resp.message}.")
            return (float('inf'), False, False, True)

        wall_duration = datetime.datetime.now() - start_wall_time
        if self.slowest_successful_wall_time is None or wall_duration > self.slowest_successful_wall_time:
            self.slowest_successful_wall_time = wall_duration

        objective = float(resp.ticks_elapsed)
        return (objective, False, False, False)

    def evaluate(self, individual: Individual) -> Tuple[float]:
        """
        Evaluate one Individual by calling RunEval asynchronously on the next namespace client.
        If the async call times out/hangs or returns FAILTIMEOUT, perform:
          eval -> (kill namespace, wait, reapply params) -> retry
        for a configurable number of retries (ROS param: eval_max_retries).
        """
        client, param_client, get_param_client, ns_index = self._next_clients()
        with self._ns_locks[ns_index]:
            ns_str = f"/ns{ns_index + 1}"

            # First regenerate the tree
            individual.generate_tree()
            # Build request once
            req = RunEval.Request()
            req.reset_tree = self._reset_tree
            req.reset_level = self._reset_level
            # timeout 10% higher than slowest seen so far
            req.timeout_sec = self._default_timeout_sec
            req.to_add = individual.yaml_path
            req.file_args = self._file_args

            total_attempts = self._eval_max_retries + 1  # initial attempt + retry count

            for attempt in range(total_attempts):
                # Apply FixedPotential parameters before all attempts
                # self._node.get_logger().fatal(
                #     f"[GA] sending PARAM request to 0-indexed rr {ns_index}, _clients: {len(self._clients)}, _param_clients: {len(self._param_clients)}")
                did_param = self._call_fixed_potential_service(param_client, get_param_client, individual)

                if did_param:
                    # Ensure service is available
                    if not client.service_is_ready():
                        ok = client.wait_for_service(timeout_sec=self._service_wait_timeout)
                        if not ok:
                            self._node.get_logger().warn(
                                f"[GA] {ns_str} service not available; attempt {attempt + 1}/{total_attempts} penalized.")
                            continue  # proceed to next attempt (or exit if out of retries)

                    # Perform the evaluation attempt
                    # self._node.get_logger().fatal(
                    #     f"[GA] sending EVAL request to 0-indexed rr {ns_index}, _clients: {len(self._clients)}, _param_clients: {len(self._param_clients)}")
                    objective, did_timeout, failtimeout_flag, general_fail = self._eval_once(client, req, ns_index)
                    if not any([did_timeout, failtimeout_flag, general_fail]):
                        # Success
                        self._node.get_logger().info(
                            f"[GA] Eval in {ns_str} success; attempt {attempt + 1}/{total_attempts}."
                        )
                        individual.delete_yaml()
                        if objective <= 0:
                            self._node.get_logger().info(f"[GA] Eval in {ns_str} returned 0 ticks.")
                            # do not return, continue the loop
                        else:
                            return (objective,)
                    # else keep going

                # Log and continue
                if not did_param:
                    reason = "set param failed"
                elif did_timeout:
                    reason = "async timeout/hang"
                elif failtimeout_flag:
                    reason = "server reported FAILTIMEOUT"
                elif general_fail:
                    reason = "general fail"
                else:
                    reason = "unknown failure"

                self._node.get_logger().warn(
                    f"[GA] Eval in {ns_str} failed ({reason}); attempt {attempt + 1}/{total_attempts}; timeout {req.timeout_sec}."
                )
                # Serialize restarts per namespace to avoid races
                with self._ns_restart_locks[ns_index]:
                    self._node.get_logger().warn(
                        f"[GA] Restarting {ns_str} before retry {attempt + 1}/{total_attempts}…")
                    self._restart_namespace(ns_index)

            # All attempts failed
            self._node.get_logger().warn(f"[GA] All {total_attempts} attempts in {ns_str} failed; penalizing fitness.")
            individual.delete_yaml()
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
class CallbackStatistics(tools.Statistics):
    """Custom statistics tool that accepts a callback, we use this to log to file"""

    def __init__(self, key=None, callback=None, interval=10):
        super().__init__(key)
        self.callback = callback
        self.interval = interval
        self.gens = 0

    def compile(self, population):
        record = super().compile(population)
        gen = self.gens
        self.gens += 1
        if self.callback and gen % self.interval == 0:
            self.callback(gen, population, record)
        return record


def log_to_file_factory(id, dir, popsize: int, gens: int, px: float, pm: float) -> Callable[[Any, Any, Any], None]:
    def log_to_file(gen, population, record):
        base_filename = f'ga-{id}-{popsize}-{gens}-{px}-{pm}'
        base_path = f'{dir}/{base_filename}'

        log_path = f'{base_path}/{base_filename}.log'
        yaml_path = f'{base_path}/{gen}-{base_filename}.yaml'
        yaml_genes_path = f'{base_path}/{gen}-{base_filename}-genes.yaml'

        pathlib.Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(yaml_path).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(yaml_genes_path).parent.mkdir(parents=True, exist_ok=True)

        with open(log_path, 'a') as f:
            f.write(f'{gen},{record['min']},{record['avg']}\n')
        best_individual: Individual = min(population, key=lambda obj: obj.fitness.values[0])

        with open(yaml_path, 'w') as f:
            yaml.dump(best_individual.tree_data, f)

        with open(yaml_genes_path, 'w') as f:
            yaml.dump(best_individual.genes, f)

    return log_to_file


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

    node.declare_parameter('log_dir', '/ros2_ws/src/ga_runs/')
    _log_dir = node.get_parameter('log_dir').value

    node.declare_parameter('log_id', '')
    _log_id = node.get_parameter('log_id').value

    node.declare_parameter('heuristics', 'all')  # all, fixed, distance, resource
    _heuristics = node.get_parameter('heuristics').value
    global PLUGINS
    if _heuristics == 'all':
        pass  # already set correctly
    elif _heuristics == 'fixed':
        PLUGINS = ['dhtt_genetic_optimizer::FixedPotential']
    elif _heuristics == 'distance':
        PLUGINS = ['dhtt_plugins::EfficiencyPotential']
    elif _heuristics == 'resource':
        PLUGINS = ['dhtt_plugins::ResourcePotential']
    else:
        raise RuntimeError(f'Param \'heuristics\' must be in: all, fixed, distance, resource; got {_heuristics}')

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
    log_to_file_cb = log_to_file_factory(id=f'{_log_id}-{datetime.datetime.now()}', dir=_log_dir, popsize=POP_SIZE,
                                         gens=NGEN, px=CX_PB,
                                         pm=MUT_PB)
    stats = CallbackStatistics(lambda ind: ind.fitness.values[0], callback=log_to_file_cb, interval=1)

    def my_avg(fitnesses: list[float]):
        if not fitnesses:
            return float('nan')
        filtered = [x for x in fitnesses if x < np.inf]
        if len(filtered) == 0:
            return float('nan')
        return float(sum(filtered) / len(filtered))

    def my_max(fitnesses: list[float]):
        filtered = [x for x in fitnesses if x < np.inf]
        if len(filtered) == 0:
            return float('nan')
        return float(max(filtered))

    def my_min(fitnesses: list[float]):
        filtered = [x for x in fitnesses if x < np.inf]
        if len(filtered) == 0:
            return float('nan')
        return float(min(filtered))

    stats.register("avg", my_avg)
    stats.register("min", my_min)
    stats.register("max", my_max)

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
