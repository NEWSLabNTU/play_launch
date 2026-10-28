# Issue #0078: a description built empty and filled, nav2_bringup's way.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def node(name):
    return Node(package="demo_nodes_cpp", executable="talker", name=name)


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_b", default_value="true"))
    ld.add_action(node("a"))
    ld.add_action(GroupAction([node("b_if")], condition=IfCondition(LaunchConfiguration("use_b"))))
    ld.add_action(
        GroupAction([node("b_unless")], condition=UnlessCondition(LaunchConfiguration("use_b")))
    )
    ld.add_entity(node("c_entity"))
    # Any iterable, and the keyword stock takes.
    ld.add_action(LaunchDescription((node("d_tuple"),)))
    ld.add_action(LaunchDescription(node(n) for n in ["e_generator"]))
    ld.add_action(LaunchDescription(initial_entities=[node("f_keyword")], deprecated_reason=None))
    assert len(ld.entities) == 8
    return ld
