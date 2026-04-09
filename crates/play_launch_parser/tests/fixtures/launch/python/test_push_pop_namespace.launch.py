"""Test PushRosNamespace and PopRosNamespace actions."""

import launch
from launch_ros.actions import Node, PushRosNamespace, PopRosNamespace


def generate_launch_description():
    return launch.LaunchDescription([
        # Test 1: Node with default namespace
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_default_ns',
        ),

        # Test 2: Push namespace and create node
        PushRosNamespace('/pushed_ns'),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_in_pushed_ns',
        ),

        # Test 3: Push another namespace (nested)
        PushRosNamespace('nested'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_in_nested_ns',
        ),

        # Test 4: Pop namespace (back to /pushed_ns)
        PopRosNamespace(),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_after_first_pop',
        ),

        # Test 5: Pop namespace (back to default)
        PopRosNamespace(),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_after_second_pop',
        ),
    ])
