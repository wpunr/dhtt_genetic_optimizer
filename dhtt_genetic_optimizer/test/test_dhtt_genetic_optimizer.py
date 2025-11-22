import os.path

import dhtt_msgs.msg
import pytest
import rclpy
import rclpy.node
import rcl_interfaces.srv
import pathlib
import yaml

from dhtt_msgs.srv import ModifyRequest, FetchRequest, ControlRequest, HistoryRequest
from dhtt_cooking_msgs.srv import CookingRequest
from dhtt_msgs.msg import Node, NodeStatus

import dhtt_genetic_optimizer.dhtt_genetic_optimizer


# boilerplate copied from dhtt_cooking tests
class ServerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('test_node')
        self.modifysrv = self.create_client(ModifyRequest, '/modify_service')
        assert self.modifysrv.wait_for_service(timeout_sec=1.0)

        self.fetchsrv = self.create_client(FetchRequest, '/fetch_service')
        assert self.fetchsrv.wait_for_service(timeout_sec=1.0)

        self.controlsrv = self.create_client(ControlRequest, '/control_service')
        assert self.controlsrv.wait_for_service(timeout_sec=1.0)

        self.historysrv = self.create_client(HistoryRequest, '/history_service')
        assert self.historysrv.wait_for_service(timeout_sec=1.0)

        self.set_param_client = self.create_client(rcl_interfaces.srv.SetParameters, '/ComAgg/set_parameters')
        assert self.set_param_client.wait_for_service(timeout_sec=1.0)

        self.get_param_client = self.create_client(rcl_interfaces.srv.GetParameters, '/ComAgg/get_parameters')
        assert self.get_param_client.wait_for_service(timeout_sec=1.0)

        self.cooking_client = self.create_client(CookingRequest, '/Cooking_Server')
        assert self.historysrv.wait_for_service(timeout_sec=1.0)

        self.root_status_sub = self.create_subscription(NodeStatus, '/root_status', self.root_status_listener, 10)
        self.status_sub = self.create_subscription(Node, "/status", self.status_listener, 10)

        self.root_state = 0
        self.node_states = {}

    def root_status_listener(self, data):
        self.root_state = data.state

    def status_listener(self, data):
        self.node_states[data.node_name] = data

    def get_tree(self):
        fetch_rq = FetchRequest.Request()
        fetch_rq.return_full_subtree = True

        fetch_future = self.fetchsrv.call_async(fetch_rq)
        rclpy.spin_until_future_complete(self, fetch_future, timeout_sec=1.0)

        assert fetch_future.done()

        fetch_rs = fetch_future.result()

        assert fetch_rs.success == True

        return fetch_rs

    def wait_for_node_in_state(self, node_name, state):
        while self.node_states[node_name].node_status.state != state and rclpy.ok():
            rclpy.spin_once(self)

    def interrupt_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.STOP

        control_future = self.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self, control_future)

        control_rs = control_future.result()

    def reset_tree(self):
        self.interrupt_tree()

        fetch_rs = self.get_tree()

        if len(fetch_rs.found_subtrees[0].tree_nodes) > 1:
            nodes_to_remove = [i.node_name for i in fetch_rs.found_subtrees[0].tree_nodes[1:]]

            reset_rq = ControlRequest.Request()
            reset_rq.type = ControlRequest.Request.RESET

            reset_future = self.controlsrv.call_async(reset_rq)
            rclpy.spin_until_future_complete(self, reset_future)

            reset_rs = reset_future.result()

            assert reset_rs.success == True

    def add_from_yaml(self, file_name, force, add_to="ROOT_0", file_args=[]):

        wd = pathlib.Path(__file__).parent.resolve()

        yaml_dict = None

        assert file_name.split('.')[-1] == 'yaml'

        with open(f'{wd}{file_name}', 'r') as file:
            yaml_dict = yaml.safe_load(file)

        assert yaml_dict is not None

        modify_rq = ModifyRequest.Request()
        modify_rq.type = ModifyRequest.Request.ADD_FROM_FILE
        modify_rq.force = force
        modify_rq.file_args = file_args

        modify_rq.to_modify.append(add_to)
        modify_rq.to_add = f'{wd}{file_name}'

        modify_future = self.modifysrv.call_async(modify_rq)
        rclpy.spin_until_future_complete(self, modify_future)

        modify_rs = modify_future.result()

        assert modify_rs.success == True

    def start_tree(self):
        control_rq = ControlRequest.Request()
        control_rq.type = ControlRequest.Request.START

        control_future = self.controlsrv.call_async(control_rq)
        rclpy.spin_until_future_complete(self, control_future)

        control_rs = control_future.result()

        assert control_rs.success == True

        return control_rs

    def wait_for_waiting(self):
        while self.root_state != NodeStatus.WAITING and rclpy.ok():
            rclpy.spin_once(self)

        assert rclpy.ok()

    def wait_for_finished_execution(self):
        while self.root_state != NodeStatus.DONE and rclpy.ok():
            rclpy.spin_once(self)

        assert rclpy.ok()

    def get_history(self):
        history_rq = HistoryRequest.Request()

        history_future = self.historysrv.call_async(history_rq)
        rclpy.spin_until_future_complete(self, history_future)

        history_rs = history_future.result()

        return history_rs.node_history


class TestGeneticOptimizer:
    PARAM_NODE_NAMES = dhtt_genetic_optimizer.dhtt_genetic_optimizer.PARAM_NODE_NAMES
    PARAM_NODE_VALUES = dhtt_genetic_optimizer.dhtt_genetic_optimizer.PARAM_NODE_VALUES

    rclpy.init()
    node = ServerNode()

    def test_fixed_potential(self):
        def set_up(names: list[str], vals: list[float]):
            self.node.reset_tree()
            self.node.wait_for_waiting()
            self.node.add_from_yaml('/test_descriptions/simple_and.yaml', True)

            param_names = rclpy.parameter.Parameter(self.PARAM_NODE_NAMES, rclpy.Parameter.Type.STRING_ARRAY,
                                                    names).to_parameter_msg()
            param_vals = rclpy.parameter.Parameter(self.PARAM_NODE_VALUES, rclpy.Parameter.Type.DOUBLE_ARRAY,
                                                   vals).to_parameter_msg()
            req = rcl_interfaces.srv.SetParameters.Request()
            req.parameters = [param_names, param_vals]
            fut = self.node.set_param_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=1.0)
            res = fut.result()
            print(res)
            assert res is not None

            req = rcl_interfaces.srv.GetParameters.Request(names={self.PARAM_NODE_NAMES, self.PARAM_NODE_VALUES})
            fut = self.node.get_param_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=1.0)
            res = fut.result()
            print(res)
            assert res is not None
            assert res.values[0] == param_names.value or res.values[1] == param_names.value
            assert res.values[1] == param_vals.value or res.values[0] == param_vals.value

            self.node.start_tree()
            self.node.wait_for_finished_execution()
            history = self.node.get_history()
            print(history)

            self.node.reset_tree()
            return history

        # TODO do this with more nodes after fixing TestBehavior
        history = set_up(['FirstTask', 'SecondTask'], [0.9, 0.5])  # First then Second
        assert 'FirstTask' in history[0] and 'SecondTask' in history[1]  # number suffixes may change
        history = set_up(['FirstTask', 'SecondTask'], [0.1, 0.9])  # Second then First
        assert 'SecondTask' in history[0] and 'FirstTask' in history[1]


class TestDeap:
    def test_make_gene(self):
        my_set = set[str]()
        fixed_set = set[float]()
        for _ in range(100):
            gene = dhtt_genetic_optimizer.dhtt_genetic_optimizer.make_gene()
            my_set.add(gene.plugin)
            if ("FixedPotential" in gene.plugin):
                fixed_set.add(gene.parameter)
        print(my_set, fixed_set)
        assert my_set == set(dhtt_genetic_optimizer.dhtt_genetic_optimizer.PLUGINS)
        assert len(fixed_set) > 1

    def test_make_individual(self):
        individual = dhtt_genetic_optimizer.dhtt_genetic_optimizer.make_individual(
            '/ros2_ws/src/dhtt_genetic_optimizer_base/dhtt_genetic_optimizer/test/test_descriptions/simple_and.yaml',
            dhtt_msgs.msg.Node.BEHAVIOR)
        assert os.path.exists(individual.yaml_path)
        individual.genes[0].plugin = "fooasdf"
        individual.generate_tree()
        with open(individual.yaml_path, 'r') as f:
            assert "fooasdf" in f.read()

        for _ in range(100):
            individual = dhtt_genetic_optimizer.dhtt_genetic_optimizer.make_individual(
                '/ros2_ws/src/dhtt_genetic_optimizer_base/dhtt_genetic_optimizer/test/test_descriptions/simple_and.yaml',
                dhtt_msgs.msg.Node.BEHAVIOR)

            count = 0
            for i in range(len(individual.genes)):
                if individual.genes[i].parameter is not None:
                    assert individual.param_names.value.string_array_value[count]
                    assert individual.param_vals.value.double_array_value[count]