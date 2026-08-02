"""Test SetParametersFromFile action for loading parameters from YAML files."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, SetParametersFromFile


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('config_dir', default_value='/test_config'),
        DeclareLaunchArgument('params_file', default_value='params.yaml'),

        # Test 1: SetParametersFromFile with simple string path (applies to all nodes)
        SetParametersFromFile('/test_config/global_params.yaml'),

        # Test 2: SetParametersFromFile with specific node_name
        SetParametersFromFile(
            '/test_config/node1_params.yaml',
            node_name='node1'
        ),

        # Test 3: SetParametersFromFile with substitutions in path
        SetParametersFromFile(
            PathJoinSubstitution([
                LaunchConfiguration('config_dir'),
                'node2_params.yaml'
            ]),
            node_name='node2'
        ),

        # Test 4: SetParametersFromFile with LaunchConfiguration for filename
        SetParametersFromFile(
            PathJoinSubstitution([
                TextSubstitution(text='/test_config/'),
                LaunchConfiguration('params_file')
            ])
        ),

        # Create some nodes that would receive these parameters
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node1',
            namespace='/test',
        ),

        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node2',
            namespace='/test',
        ),

        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node3',
            namespace='/test',
        ),
    ])
