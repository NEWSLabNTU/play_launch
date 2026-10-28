# Issue #0078: a PythonExpression built before its launch configurations
# exist is performed when its condition is evaluated, as nav2_bringup's
# `IfCondition(PythonExpression(['not ', use_composition]))`.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def node(name, expression):
    return Node(
        package="demo_nodes_cpp",
        executable="talker",
        name=name,
        condition=IfCondition(PythonExpression(expression)),
    )


def generate_launch_description():
    use_composition = LaunchConfiguration("use_composition")
    mode = LaunchConfiguration("mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("mode", default_value="sim"),
            node("not_composed", ["not ", use_composition]),
            node("composed", [use_composition]),
            node("sim_mode", ["'", mode, "' == 'sim'"]),
            node("real_mode", ["'", mode, "' == 'real'"]),
            node("literal", "1 + 1 == 2"),
        ]
    )
