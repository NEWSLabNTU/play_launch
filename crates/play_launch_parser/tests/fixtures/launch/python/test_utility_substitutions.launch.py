"""Test utility substitutions: BooleanSubstitution, FindExecutable, LaunchLogDir, ThisLaunchFile."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    BooleanSubstitution,
    FindExecutable,
    LaunchLogDir,
    ThisLaunchFile,
)
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('bool_value', default_value='true'),
        DeclareLaunchArgument('exec_name', default_value='python3'),

        # Test 1: BooleanSubstitution with static "true" value
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_bool_true',
            namespace=BooleanSubstitution('true'),
        ),

        # Test 2: BooleanSubstitution with static "false" value
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_bool_false',
            namespace=BooleanSubstitution('false'),
        ),

        # Test 3: BooleanSubstitution with "1" (should convert to "true")
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_bool_1',
            namespace=BooleanSubstitution('1'),
        ),

        # Test 4: BooleanSubstitution with "0" (should convert to "false")
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_bool_0',
            namespace=BooleanSubstitution('0'),
        ),

        # Test 5: BooleanSubstitution with LaunchConfiguration
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_bool_dynamic',
            namespace=BooleanSubstitution(LaunchConfiguration('bool_value')),
        ),

        # Test 6: FindExecutable with static value
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_find_exec_static',
            namespace=FindExecutable('python3'),
        ),

        # Test 7: FindExecutable with LaunchConfiguration
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_find_exec_dynamic',
            namespace=FindExecutable(LaunchConfiguration('exec_name')),
        ),

        # Test 8: LaunchLogDir (no arguments)
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_log_dir',
            namespace=LaunchLogDir(),
        ),

        # Test 9: ThisLaunchFile (no arguments)
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_this_file',
            namespace=ThisLaunchFile(),
        ),

        # Test 10: Combined - BooleanSubstitution with multiple values
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_combined_bool_yes',
            namespace=BooleanSubstitution('yes'),
        ),
    ])
