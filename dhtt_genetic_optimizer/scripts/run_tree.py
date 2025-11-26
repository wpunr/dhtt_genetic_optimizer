#!/usr/bin/python3

import rclpy
import pathlib
import yaml
import sys
from rclpy.node import Node
from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest
from dhtt_cooking_msgs.srv import CookingRequest
from dhtt_msgs.msg import NodeStatus


class ServerNode(Node):
    def __init__(self):
        super().__init__('tree_runner')
        self.modifysrv = self.create_client(ModifyRequest, '/modify_service')
        self.fetchsrv = self.create_client(FetchRequest, '/fetch_service')
        self.controlsrv = self.create_client(ControlRequest, '/control_service')
        self.historysrv = self.create_client(HistoryRequest, '/history_service')
        self.cooking_client = self.create_client(CookingRequest, '/Cooking_Server')

        # Wait for services
        for client in [self.modifysrv, self.fetchsrv, self.controlsrv, self.historysrv, self.cooking_client]:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("Required service not available")

        # Subscribe to root status to track state transitions
        self.root_state = 0
        self.create_subscription(NodeStatus, '/root_status', self.root_status_listener, 10)

    def root_status_listener(self, data: NodeStatus):
        self.root_state = data.state

    def reset_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.RESET
        future = self.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def reset_level(self):
        request = CookingRequest.Request()
        # Mirrors original test's "super_action = START"
        request.super_action = CookingRequest.Request.START
        future = self.cooking_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_waiting(self):
        while self.root_state != NodeStatus.WAITING and rclpy.ok():
            rclpy.spin_once(self)

    def add_from_yaml(self, file_name, force=True, add_to="ROOT_0", file_args=[]):
        # Basic validation and load to ensure file is well-formed
        if not file_name.endswith('.yaml'):
            raise ValueError("File must be a YAML file (.yaml)")
        with open(file_name, 'r') as file:
            yaml_dict = yaml.safe_load(file)
        if yaml_dict is None:
            raise ValueError("YAML file is empty or invalid")

        modify_rq = ModifyRequest.Request()
        modify_rq.type = ModifyRequest.Request.ADD_FROM_FILE
        modify_rq.force = force
        modify_rq.file_args = file_args
        modify_rq.to_modify.append(add_to)
        modify_rq.to_add = str(pathlib.Path(file_name).resolve())

        future = self.modifysrv.call_async(modify_rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def start_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.START
        future = self.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_finished_execution(self):
        while self.root_state != NodeStatus.DONE and rclpy.ok():
            rclpy.spin_once(self)

    def get_history(self):
        history_rq = HistoryRequest.Request()
        future = self.historysrv.call_async(history_rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result().node_history


def main():
    if len(sys.argv) < 2:
        print("Usage: python run_tree.py <path_to_yaml>")
        sys.exit(1)

    yaml_path = sys.argv[1]
    rclpy.init()
    node = ServerNode()

    print("Resetting tree...")
    node.reset_tree()
    print("Resetting level...")
    node.reset_level()
    print("Waiting for tree to be in WAITING state...")
    node.wait_for_waiting()

    print(f"Adding nodes from {yaml_path}...")
    add_rs = node.add_from_yaml(yaml_path)
    if hasattr(add_rs, 'success') and not add_rs.success:
        print("Failed to add nodes from YAML.")
        rclpy.shutdown()
        sys.exit(2)

    print("Starting tree execution...")
    start_rs = node.start_tree()
    if hasattr(start_rs, 'success') and not start_rs.success:
        print("Failed to start tree.")
        rclpy.shutdown()
        sys.exit(3)

    print("Waiting for execution to finish...")
    node.wait_for_finished_execution()
    print("Execution finished.")

    # Fetch and print history
    history = node.get_history()
    print("\n=== Execution History ===")
    if history:
        for idx, entry in enumerate(history, start=1):
            print(f"{idx:02d}. {entry}")
    else:
        print("(no history entries)")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
