#!/usr/bin/python3

import rclpy
import pathlib
import yaml
import sys
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest
from dhtt_cooking_msgs.srv import CookingRequest
from dhtt_msgs.msg import NodeStatus
from dhtt_genetic_optimizer.dhtt_genetic_optimizer import Gene


class ServerNode(Node):
    def __init__(self):
        super().__init__('tree_runner')
        self.modifysrv = self.create_client(ModifyRequest, '/modify_service')
        self.fetchsrv = self.create_client(FetchRequest, '/fetch_service')
        self.controlsrv = self.create_client(ControlRequest, '/control_service')
        self.historysrv = self.create_client(HistoryRequest, '/history_service')
        self.cooking_client = self.create_client(CookingRequest, '/Cooking_Server')
        self.param_client = self.create_client(SetParameters, '/ComAgg/set_parameters')
        self.get_param_client = self.create_client(GetParameters, '/ComAgg/get_parameters')

        # Wait for services
        for client in [self.modifysrv, self.fetchsrv, self.controlsrv, self.historysrv,
                       self.cooking_client, self.param_client, self.get_param_client]:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("Required service not available")

        # Subscribe to root status
        self.root_state = 0
        self.create_subscription(NodeStatus, '/root_status', self.root_status_listener, 10)

    def root_status_listener(self, data: NodeStatus):
        self.root_state = data.state

    def reset_tree(self):
        rq = ControlRequest.Request()
        rq.type = ControlRequest.Request.RESET
        future = self.controlsrv.call_async(rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def reset_level(self):
        rq = CookingRequest.Request()
        rq.super_action = CookingRequest.Request.START
        future = self.cooking_client.call_async(rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_waiting(self):
        while self.root_state != NodeStatus.WAITING and rclpy.ok():
            rclpy.spin_once(self)

    def add_from_yaml(self, file_name, force=True, add_to="ROOT_0", file_args=[]):
        if not file_name.endswith('.yaml'):
            raise ValueError("File must be a YAML file (.yaml)")
        with open(file_name, 'r') as file:
            yaml_dict = yaml.safe_load(file)
        if yaml_dict is None:
            raise ValueError("YAML file is empty or invalid")
        rq = ModifyRequest.Request()
        rq.type = ModifyRequest.Request.ADD_FROM_FILE
        rq.force = force
        rq.file_args = file_args
        rq.to_modify.append(add_to)
        rq.to_add = str(pathlib.Path(file_name).resolve())
        future = self.modifysrv.call_async(rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def start_tree(self):
        rq = ControlRequest.Request()
        rq.type = ControlRequest.Request.START
        future = self.controlsrv.call_async(rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def wait_for_finished_execution(self):
        while self.root_state != NodeStatus.DONE and rclpy.ok():
            rclpy.spin_once(self)

    def get_history(self):
        rq = HistoryRequest.Request()
        future = self.historysrv.call_async(rq)
        rclpy.spin_until_future_complete(self, future)
        return future.result().node_history

    def apply_fixed_potentials(self, tree_file: str, genes_file: str):
        # Parse tree for NodeList
        with open(tree_file, 'r') as f:
            tree_data = yaml.safe_load(f)
        node_list = tree_data.get('NodeList', []) # TODO need to check that the node has correct potential type

        # Parse genes for FixedPotential
        with open(genes_file, 'r') as f:
            raw_genes = yaml.unsafe_load(f)

        fixed_names = []
        fixed_values = []
        for idx, gene in enumerate(raw_genes):
            plugin = gene.plugin
            param = gene.parameter
            if plugin == 'dhtt_genetic_optimizer::FixedPotential' and param is not None:
                if idx < len(node_list):
                    fixed_names.append(node_list[idx])
                    fixed_values.append(param)

        if not fixed_names:
            self.get_logger().info("No FixedPotential genes found.")
            return

        # Send SetParameters request
        req = SetParameters.Request()
        req.parameters = [
            Parameter('FixedPotentialNodeNames', Parameter.Type.STRING_ARRAY, fixed_names).to_parameter_msg(),
            Parameter('FixedPotentialValues', Parameter.Type.DOUBLE_ARRAY, fixed_values).to_parameter_msg()
        ]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None:
            self.get_logger().error("Failed to set FixedPotential parameters.")
        else:
            self.get_logger().info(f"Applied FixedPotential to {len(fixed_names)} nodes.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python run_tree.py <path_to_yaml>")
        sys.exit(1)

    yaml_path = sys.argv[1]
    genes_path = yaml_path.replace('.yaml', '-genes.yaml')

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

    print(f"Applying fixed potentials from {genes_path}...")
    node.apply_fixed_potentials(yaml_path, genes_path)

    print("Starting tree execution...")
    start_rs = node.start_tree()
    if hasattr(start_rs, 'success') and not start_rs.success:
        print("Failed to start tree.")
        rclpy.shutdown()
        sys.exit(3)

    print("Waiting for execution to finish...")
    node.wait_for_finished_execution()
    print("Execution finished.")

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
