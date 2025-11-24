# !/usr/bin/env python3
"""
ROS 2 (Jazzy) launch file to spawn N namespaced sets of:
  - dhtt/start_server
  - dhtt_cooking/dhtt_cooking
  - dhtt_cooking/eval

Example:
  ros2 launch your_pkg dhtt_multi.launch.py num_instances:=3 ns_prefix:=ns start_index:=1
  # Spawns under namespaces: ns1, ns2, ns3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

# global remappings since everything we've done mixes absolute and relative names, which is a pain
# this will make them all relative, so the namespace stuff works
# your choice to do /rosout or /parameter_events
REMAPPINGS = {
    # ("/rosout", "rosout"),
    ("/parameter_events", "parameter_events"),
    ("/ComAgg/describe_parameters", "ComAgg/describe_parameters"),
    ("/ComAgg/get_parameter_types", "ComAgg/get_parameter_types"),
    ("/ComAgg/get_parameters", "ComAgg/get_parameters"),
    ("/ComAgg/get_type_description", "ComAgg/get_type_description"),
    ("/ComAgg/list_parameters", "ComAgg/list_parameters"),
    ("/ComAgg/set_parameters", "ComAgg/set_parameters"),
    ("/ComAgg/set_parameters_atomically", "ComAgg/set_parameters_atomically"),
    ("/Cooking_Observations", "Cooking_Observations"),
    ("/Cooking_Server", "Cooking_Server"),
    ("/control_service", "control_service"),
    ("/cooking_zoo/describe_parameters", "cooking_zoo/describe_parameters"),
    ("/cooking_zoo/get_parameter_types", "cooking_zoo/get_parameter_types"),
    ("/cooking_zoo/get_parameters", "cooking_zoo/get_parameters"),
    ("/cooking_zoo/get_type_description", "cooking_zoo/get_type_description"),
    ("/cooking_zoo/list_parameters", "cooking_zoo/list_parameters"),
    ("/cooking_zoo/set_parameters", "cooking_zoo/set_parameters"),
    ("/cooking_zoo/set_parameters_atomically", "cooking_zoo/set_parameters_atomically"),
    ("/dHTT_server/describe_parameters", "dHTT_server/describe_parameters"),
    ("/dHTT_server/get_parameter_types", "dHTT_server/get_parameter_types"),
    ("/dHTT_server/get_parameters", "dHTT_server/get_parameters"),
    ("/dHTT_server/get_type_description", "dHTT_server/get_type_description"),
    ("/dHTT_server/list_parameters", "dHTT_server/list_parameters"),
    ("/dHTT_server/set_parameters", "dHTT_server/set_parameters"),
    ("/dHTT_server/set_parameters_atomically", "dHTT_server/set_parameters_atomically"),
    ("/dhtt/control", "dhtt/control"),
    ("/dhtt/resource", "dhtt/resource"),
    ("/eval/run", "eval/run"),
    ("/fetch_service", "fetch_service"),
    ("/history_service", "history_service"),
    ("/modify_service", "modify_service"),
    ("/resource_service", "resource_service"),
    ("/root_status", "root_status"),
    ("/status", "status"),
}


def _int_from_config(context, lc: LaunchConfiguration, default: int) -> int:
    """Safely convert a LaunchConfiguration to int with a default fallback."""
    try:
        return int(lc.perform(context))
    except Exception:
        return int(default)


def _str_from_config(context, lc: LaunchConfiguration, default: str) -> str:
    s = lc.perform(context)
    return s if s is not None and s != "" else default


def _make_group_for_namespace(ns: str):
    """Create the group of nodes under a given namespace."""

    remaps = [SetRemap(src=l, dst=r) for l, r in REMAPPINGS]

    return GroupAction([PushRosNamespace(ns), ] + remaps + [
        # Equivalent to: ros2 run dhtt start_server
        Node(
            package="dhtt",
            executable="start_server",
            output="screen",
            respawn=True,
            respawn_delay=2,
        ),

        # Equivalent to: ros2 run dhtt_cooking dhtt_cooking
        Node(
            package="dhtt_cooking",
            executable="dhtt_cooking",
            output="screen",
            respawn=True,
            respawn_delay=2,
        ),

        # Equivalent to: ros2 run dhtt_cooking eval
        Node(
            package="dhtt_genetic_optimizer",
            executable="eval.py",
            output="screen",
            respawn=True,
            respawn_delay=2,
        ),
    ])


def _expand(context, *args, **kwargs):
    """Build all namespaced groups based on user-provided args."""
    num_instances = _int_from_config(context, LaunchConfiguration("num_instances"), 1)
    start_index = _int_from_config(context, LaunchConfiguration("start_index"), 1)
    ns_prefix = _str_from_config(context, LaunchConfiguration("ns_prefix"), "ns")

    actions = [
        # Node(
        #     package="dhtt_genetic_optimizer",
        #     executable="dhtt_genetic_optimizer.py",
        #     output="screen",
        #     parameters=[{
        #         'num_namespaces': num_instances,
        #     }]
        # )
    ]
    for i in range(start_index, start_index + num_instances):
        ns = f"{ns_prefix}{i}"
        actions.append(_make_group_for_namespace(ns))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "num_instances",
            default_value="1",
            description="How many namespace groups to launch (each group runs 3 nodes).",
        ),
        DeclareLaunchArgument(
            "ns_prefix",
            default_value="ns",
            description="Prefix for namespaces; groups will be ns_prefix + index, e.g., ns1, ns2.",
        ),
        DeclareLaunchArgument(
            "start_index",
            default_value="1",
            description="Starting index for namespace numbering.",
        ),
        OpaqueFunction(function=_expand),
    ])
