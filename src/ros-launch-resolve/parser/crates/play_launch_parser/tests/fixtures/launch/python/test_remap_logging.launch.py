"""Test remapping and logging configuration actions."""

import launch
from launch.actions import OpaqueCoroutine
from launch_ros.actions import Node, SetRemap, SetROSLogDir


# Simple coroutine for testing
async def example_coroutine(context):
    """Example coroutine function."""
    # In static analysis, this doesn't execute
    pass


def generate_launch_description():
    return launch.LaunchDescription([
        # Test 1: SetROSLogDir
        SetROSLogDir('/test_config/ros_logs'),

        # Test 2: SetRemap for topic remapping
        SetRemap(src='/old_topic', dst='/new_topic'),

        # Test 3: Node with remapping in effect
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_remap',
        ),

        # Test 4: OpaqueCoroutine
        OpaqueCoroutine(
            coroutine=example_coroutine,
        ),

        # Test 5: Another SetRemap
        SetRemap(src='/cmd_vel', dst='/robot/cmd_vel'),

        # Test 6: Node using remapped topics
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_node',
        ),

        # Test 7: Node with explicit remapping
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_with_remap',
            remappings=[('/chatter', '/my_chatter')],
        ),
    ])
