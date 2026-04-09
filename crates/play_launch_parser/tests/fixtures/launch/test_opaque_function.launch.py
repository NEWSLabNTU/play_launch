#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """
    This function is called by OpaqueFunction.
    It demonstrates dynamic node generation based on launch arguments.
    """
    # Get launch configurations
    node_count = int(LaunchConfiguration('node_count').perform(context))

    # Create nodes dynamically
    nodes = []
    for i in range(node_count):
        node = Node(
            package=f'pkg_{i}',
            executable='exec',
            name=f'dynamic_node_{i}',
            namespace='/dynamic',
        )
        nodes.append(node)

    # Return list of nodes
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_count',
            default_value='3',
            description='Number of nodes to create'
        ),
        OpaqueFunction(function=launch_setup),
    ])
