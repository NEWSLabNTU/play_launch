#!/usr/bin/env python3
"""Test OpaqueFunction with conditional logic.

Edge case from Autoware: OpaqueFunction that creates different nodes based
on runtime LaunchConfiguration values (simple_planning_simulator pattern).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Create nodes conditionally based on launch configuration."""
    mode = LaunchConfiguration("mode").perform(context)

    nodes = []

    # Always create base node
    base_node = Node(
        package="test_pkg",
        executable="base_exe",
        name="conditional_base",
        namespace="test",
    )
    nodes.append(base_node)

    # Conditionally create additional node
    if mode == "full":
        extra_node = Node(
            package="test_pkg",
            executable="extra_exe",
            name="conditional_extra",
            namespace="test",
        )
        nodes.append(extra_node)

    return nodes


def generate_launch_description():
    """Generate launch description with conditional OpaqueFunction."""
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="minimal",
        description="Launch mode: minimal or full",
    )

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_setup),
    ])
