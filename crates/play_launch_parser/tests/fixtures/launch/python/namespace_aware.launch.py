"""Test that Python launch files can access ROS namespace from XML context."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node_with_context_ns(context):
    """Create a node using the namespace from the context."""
    # This should get the namespace pushed by XML <push-ros-namespace>
    # We don't explicitly set namespace - it will inherit from the ROS namespace stack
    # The XML <push-ros-namespace> will have already pushed the namespace
    return [
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='context_aware_node',
            # No namespace parameter - inherits from context
        )
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('expected_ns', default_value=''),
        OpaqueFunction(function=create_node_with_context_ns),
    ])
