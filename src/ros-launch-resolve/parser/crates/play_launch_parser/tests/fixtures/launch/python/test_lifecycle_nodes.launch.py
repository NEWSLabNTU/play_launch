"""Test LifecycleNode and LifecycleTransition actions for managed node lifecycle."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('node_name', default_value='my_lifecycle_node'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Test 1: Basic LifecycleNode with minimal parameters
        LifecycleNode(
            package='lifecycle_pkg',
            executable='lifecycle_node',
            name='lifecycle_node_1',
            namespace='/test',
        ),

        # Test 2: LifecycleNode with parameters and remappings
        LifecycleNode(
            package='lifecycle_pkg',
            executable='lifecycle_node',
            name='lifecycle_node_2',
            namespace='/test',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'param1': 'value1'},
                {'param2': 42},
            ],
            remappings=[
                ('/input', '/remapped_input'),
                ('/output', '/remapped_output'),
            ],
        ),

        # Test 3: LifecycleNode with arguments
        LifecycleNode(
            package='lifecycle_pkg',
            executable='lifecycle_node',
            name=LaunchConfiguration('node_name'),
            namespace='/test',
            arguments=['--ros-args', '--log-level', 'debug'],
        ),

        # Test 4: LifecycleNode with output specification
        LifecycleNode(
            package='lifecycle_pkg',
            executable='lifecycle_node',
            name='lifecycle_node_3',
            namespace='/test',
            output='both',  # screen, log, or both
        ),

        # Test 5: LifecycleTransition with transition_id
        LifecycleTransition(
            lifecycle_node_names=['lifecycle_node_1', 'lifecycle_node_2'],
            transition_id=1,  # 1 = configure
        ),

        # Test 6: LifecycleTransition with transition_label
        LifecycleTransition(
            lifecycle_node_names=['lifecycle_node_1', 'lifecycle_node_2'],
            transition_label='configure',
        ),

        # Test 7: LifecycleTransition with activate
        LifecycleTransition(
            lifecycle_node_names=['lifecycle_node_1', 'lifecycle_node_2'],
            transition_label='activate',
        ),

        # Test 8: LifecycleTransition for single node
        LifecycleTransition(
            lifecycle_node_names=['lifecycle_node_3'],
            transition_label='activate',
        ),
    ])
