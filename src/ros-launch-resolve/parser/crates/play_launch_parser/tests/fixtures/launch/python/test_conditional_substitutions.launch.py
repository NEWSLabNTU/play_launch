"""Test conditional substitutions: EqualsSubstitution, NotEqualsSubstitution, IfElseSubstitution, FileContent."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    NotEqualsSubstitution,
    IfElseSubstitution,
    TextSubstitution,
    PathJoinSubstitution,
    FileContent,
)
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('mode', default_value='debug'),
        DeclareLaunchArgument('enable_feature', default_value='true'),
        DeclareLaunchArgument('config_dir', default_value='.'),

        # Test EqualsSubstitution
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_equals_true',
            # mode == 'debug' -> true
            namespace=EqualsSubstitution(
                LaunchConfiguration('mode'),
                'debug'
            ),
        ),

        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_equals_false',
            # mode == 'release' -> false
            namespace=EqualsSubstitution(
                LaunchConfiguration('mode'),
                'release'
            ),
        ),

        # Test NotEqualsSubstitution
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_not_equals_true',
            # mode != 'release' -> true
            namespace=NotEqualsSubstitution(
                LaunchConfiguration('mode'),
                'release'
            ),
        ),

        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_not_equals_false',
            # mode != 'debug' -> false
            namespace=NotEqualsSubstitution(
                LaunchConfiguration('mode'),
                'debug'
            ),
        ),

        # Test IfElseSubstitution with EqualsSubstitution condition
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_ifelse_true',
            # if mode == 'debug' then '/debug_ns' else '/release_ns'
            namespace=IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration('mode'), 'debug'),
                '/debug_ns',
                '/release_ns'
            ),
        ),

        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_ifelse_with_config',
            # if enable_feature == 'true' then 'enabled' else 'disabled'
            namespace=IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration('enable_feature'), 'true'),
                TextSubstitution(text='/enabled'),
                TextSubstitution(text='/disabled')
            ),
        ),

        # Test nested IfElseSubstitution
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_nested_ifelse',
            # Nested: if mode == 'debug' then (if enable_feature == 'true' then 'debug_enabled' else 'debug_disabled') else 'release'
            namespace=IfElseSubstitution(
                EqualsSubstitution(LaunchConfiguration('mode'), 'debug'),
                IfElseSubstitution(
                    EqualsSubstitution(LaunchConfiguration('enable_feature'), 'true'),
                    '/debug_enabled',
                    '/debug_disabled'
                ),
                '/release'
            ),
        ),

        # Test FileContent with simple path using config_dir
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_file_content_simple',
            namespace=FileContent(
                PathJoinSubstitution([
                    LaunchConfiguration('config_dir'),
                    'test_content.txt'
                ])
            ),
        ),

        # Test FileContent with PathJoinSubstitution
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_file_content_path_join',
            namespace=FileContent(
                PathJoinSubstitution([
                    LaunchConfiguration('config_dir'),
                    'namespace.txt'
                ])
            ),
        ),
    ])
