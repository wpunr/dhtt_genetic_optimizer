#!/usr/bin/python3

"""Chatbot special, if you can't tell"""

import threading
from typing import Optional

import yaml

import rclpy
import rclpy.node
import rclpy.callback_groups

# --- External services/messages your tests were using ---
from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest
from dhtt_cooking_msgs.srv import CookingRequest

from dhtt_msgs.msg import NodeStatus, Node
from dhtt_cooking_msgs.msg import CookingObservation

# --- Our custom service for triggering the run ---
from dhtt_genetic_optimizer_msgs.srv import RunEval

DEFAULT_WAIT_TIMEOUT = 300.0  # seconds, if request.timeout_sec <= 0

SPIN_TIMEOUT_SEC_FAST = 0.1 # for repeated spin_once
SPIN_TIMEOUT_SEC_SLOW = 10 # for critical spins like service requests


class EvalNode(rclpy.node.Node):
    """
    A minimal node ('eval') that mirrors the test's initialize/start behavior:
      - optional tree reset and level reset
      - start the tree
      - wait until DONE
      - return ticks elapsed observed on /Cooking_Observations
    """

    def __init__(self) -> None:
        super().__init__('eval')

        self.cb = rclpy.callback_groups.ReentrantCallbackGroup()

        # --- Service clients (mirror your pytest ServerNode) ---
        self.modifysrv = self.create_client(ModifyRequest, '/modify_service', callback_group=self.cb)
        self.fetchsrv = self.create_client(FetchRequest, '/fetch_service', callback_group=self.cb)
        self.controlsrv = self.create_client(ControlRequest, '/control_service', callback_group=self.cb)
        self.historysrv = self.create_client(HistoryRequest, '/history_service', callback_group=self.cb)
        self.cooking_client = self.create_client(CookingRequest, '/Cooking_Server', callback_group=self.cb)

        # Wait for services (short, repeated waits so node can exit cleanly if unavailable)
        for name, cli, t in [
            ('/modify_service', self.modifysrv, 5.0),
            ('/fetch_service', self.fetchsrv, 5.0),
            ('/control_service', self.controlsrv, 5.0),
            ('/history_service', self.historysrv, 5.0),
            ('/Cooking_Server', self.cooking_client, 5.0),
        ]:
            if not cli.wait_for_service(timeout_sec=t):
                self.get_logger().warn(f"Service {name} not available after {t}s; continuing.")

        # --- Subscriptions ---
        self.root_state = 0
        self.create_subscription(NodeStatus, '/root_status', self._on_root_status, 10, callback_group=self.cb)

        # Track ticks from /Cooking_Observations
        self._ticks_lock = threading.Lock()
        self._last_ticks: Optional[int] = None
        self.create_subscription(CookingObservation, '/Cooking_Observations', self._on_observation, 10,
                                 callback_group=self.cb)

        # --- Public service to trigger a run ---
        self._run_srv = self.create_service(RunEval, '/eval/run', self._on_run_eval, callback_group=self.cb)

        self.get_logger().info("eval node is up; service '/eval/run' ready.")

    # -----------------------------
    # Subscriptions
    # -----------------------------
    def _on_root_status(self, msg: NodeStatus) -> None:
        self.root_state = msg.state

    def _on_observation(self, msg: CookingObservation) -> None:
        with self._ticks_lock:
            self._last_ticks = int(msg.ticks)

    # -----------------------------
    # Helpers mirroring your tests
    # -----------------------------
    def _control(self, control_type: int) -> bool:
        req = ControlRequest.Request()
        req.type = control_type
        fut = self.controlsrv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=SPIN_TIMEOUT_SEC_SLOW)
        res = fut.result()
        if res is None:
            self.get_logger().error("Control service call failed (no response).")
            return False
        return bool(res.success)

    def interrupt_tree(self) -> bool:
        return self._control(ControlRequest.Request.STOP)

    def reset_tree(self) -> bool:
        # Note: your test's reset_tree() first STOPs; we mirror that.
        ok_stop = self.interrupt_tree()
        ok_reset = self._control(ControlRequest.Request.RESET)
        return ok_stop and ok_reset

    def start_tree(self) -> bool:
        return self._control(ControlRequest.Request.START)

    def reset_level(self) -> bool:
        req = CookingRequest.Request()
        # your test: request.super_action = CookingRequest.Request.START
        req.super_action = CookingRequest.Request.START
        fut = self.cooking_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=SPIN_TIMEOUT_SEC_SLOW)
        res = fut.result()
        return res is not None

    def add_root_and(self) -> str:
        req = ModifyRequest.Request()
        req.type = ModifyRequest.Request.ADD
        req.to_modify.append('ROOT_0')
        req.add_node = Node()
        req.add_node.type = Node.AND
        req.add_node.node_name = 'AllOrdersAnd'
        req.add_node.plugin_name = 'dhtt_plugins::AndBehavior'
        fut = self.modifysrv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=SPIN_TIMEOUT_SEC_SLOW)

        true_name = fut.result().added_nodes[0]
        return true_name

    def add_root_then(self) -> str:
        req = ModifyRequest.Request()
        req.type = ModifyRequest.Request.ADD
        req.to_modify.append('ROOT_0')
        req.add_node = Node()
        req.add_node.type = Node.THEN
        req.add_node.node_name = 'AllOrdersThen'
        req.add_node.plugin_name = 'dhtt_plugins::ThenBehavior'
        fut = self.modifysrv.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=SPIN_TIMEOUT_SEC_SLOW)

        true_name = fut.result().added_nodes[0]
        return true_name

    def add_from_yaml(self, absolute_path: str, add_to: str, file_args: list[str] | None = None) -> list[str]:
        with open(absolute_path, 'r') as file:
            yaml_dict = yaml.safe_load(file)

        if yaml_dict is None:
            raise RuntimeError("yaml did not load correctly")

        modify_rq = ModifyRequest.Request()
        modify_rq.type = ModifyRequest.Request.ADD_FROM_FILE
        modify_rq.force = True
        modify_rq.file_args = file_args if file_args is not None else []

        modify_rq.to_modify.append(add_to)
        modify_rq.to_add = absolute_path

        modify_future = self.modifysrv.call_async(modify_rq)
        rclpy.spin_until_future_complete(self, modify_future, timeout_sec=SPIN_TIMEOUT_SEC_SLOW)

        modify_rs = modify_future.result()

        if modify_rs.success == False:
            raise RuntimeError("add from file failed")

        return modify_rs.added_nodes

    def _wait_for_state(self, target_state: int, timeout_sec: float) -> bool:
        """
        Spin until root_status == target_state or timeout.
        """
        if timeout_sec is None or timeout_sec <= 0:
            timeout_sec = DEFAULT_WAIT_TIMEOUT

        end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok():
            if self.root_state == target_state:
                return True
            if self.get_clock().now().nanoseconds >= end_time:
                return False
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_SEC_FAST)
        return False

    def _snapshot_ticks(self, must_wait: bool, wait_timeout_sec: float) -> Optional[int]:
        """
        Get current ticks from /Cooking_Observations.
        If must_wait and no ticks yet, wait up to wait_timeout_sec for the first tick.
        """
        with self._ticks_lock:
            ticks = self._last_ticks

        if ticks is not None or not must_wait:
            return ticks

        # Wait for first tick
        end_time = self.get_clock().now().nanoseconds + int(wait_timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_SEC_FAST)
            with self._ticks_lock:
                if self._last_ticks is not None:
                    return self._last_ticks
        return None

    # -----------------------------
    # Service callback
    # -----------------------------
    def _on_run_eval(self, req: RunEval.Request, res: RunEval.Response) -> RunEval.Response:
        """
        Implements:
          1) optional reset_tree
          2) optional reset_level (CookingRequest.START)
          3) capture start ticks
          4) start tree
          5) wait for DONE (or timeout)
          6) capture finish ticks and return delta
        """
        timeout = req.timeout_sec if req.timeout_sec > 0.0 else DEFAULT_WAIT_TIMEOUT

        # Reset operations (optional)
        if req.reset_tree:
            if not self.reset_tree():
                res.success = False
                res.message = RunEval.Response.FAILRESETDHTT
                res.ticks_elapsed = 0
                return res

        if req.reset_level:
            if not self.reset_level():
                res.success = False
                res.message = RunEval.Response.FAILRESETLEVEL
                res.ticks_elapsed = 0
                return res

        if not req.to_add:
            res.success = False
            res.message = RunEval.Response.FAILLOADYAML
            res.ticks_elapsed = 0
            return res

        try:
            self.add_from_yaml(req.to_add, "ROOT_0", req.file_args)
        except RuntimeError:
            res.success = False
            res.message = RunEval.Response.FAILRESETLEVEL
            res.ticks_elapsed = 0
            return res

        # Snapshot starting ticks (do not *require* a tick before start, but try briefly)
        start_ticks = self._snapshot_ticks(must_wait=False, wait_timeout_sec=2.0)

        # Start execution
        if not self.start_tree():
            res.success = False
            res.message = RunEval.Response.FAILSTART
            res.ticks_elapsed = 0
            return res

        # Wait for DONE
        done_ok = self._wait_for_state(NodeStatus.DONE, timeout)
        if not done_ok:
            res.success = False
            res.message = RunEval.Response.FAILTIMEOUT
            # Even on timeout, report delta if we have a finish tick
            finish_ticks = self._snapshot_ticks(must_wait=False, wait_timeout_sec=0.0)
            if start_ticks is not None and finish_ticks is not None and finish_ticks >= start_ticks:
                res.ticks_elapsed = finish_ticks - start_ticks
            else:
                res.ticks_elapsed = 0
            return res

        # Finished: compute ticks elapsed
        finish_ticks = self._snapshot_ticks(must_wait=True, wait_timeout_sec=2.0)
        ticks_elapsed = 0
        if start_ticks is not None and finish_ticks is not None and finish_ticks >= start_ticks:
            ticks_elapsed = finish_ticks - start_ticks

        res.success = True
        res.message = RunEval.Response.FAILSUCCESS
        res.ticks_elapsed = ticks_elapsed
        return res


def main() -> None:
    rclpy.init()
    node = EvalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
