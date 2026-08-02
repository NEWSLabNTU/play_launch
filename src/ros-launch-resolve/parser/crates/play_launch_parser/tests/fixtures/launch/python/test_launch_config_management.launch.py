"""Test launch configuration management actions: PushLaunchConfigurations, PopLaunchConfigurations, ResetLaunchConfigurations, UnsetLaunchConfiguration."""

import launch
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    PushLaunchConfigurations,
    PopLaunchConfigurations,
    ResetLaunchConfigurations,
    UnsetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare initial arguments
        DeclareLaunchArgument('config1', default_value='initial_value_1'),
        DeclareLaunchArgument('config2', default_value='initial_value_2'),
        DeclareLaunchArgument('config3', default_value='initial_value_3'),

        # Test 1: Basic PushLaunchConfigurations and PopLaunchConfigurations
        # Modify a configuration, push, modify it again, then pop to restore
        SetLaunchConfiguration('config1', 'modified_value_1'),
        PushLaunchConfigurations(),
        SetLaunchConfiguration('config1', 'temporary_value_1'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_temp_config',
            namespace=LaunchConfiguration('config1'),
        ),
        PopLaunchConfigurations(),  # config1 should be restored to 'modified_value_1'
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_restored_config',
            namespace=LaunchConfiguration('config1'),
        ),

        # Test 2: Nested push/pop
        SetLaunchConfiguration('config2', 'level_0'),
        PushLaunchConfigurations(),
        SetLaunchConfiguration('config2', 'level_1'),
        PushLaunchConfigurations(),
        SetLaunchConfiguration('config2', 'level_2'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_at_config_level_2',
            namespace=LaunchConfiguration('config2'),
        ),
        PopLaunchConfigurations(),  # Back to level_1
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_at_config_level_1',
            namespace=LaunchConfiguration('config2'),
        ),
        PopLaunchConfigurations(),  # Back to level_0
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_at_config_level_0',
            namespace=LaunchConfiguration('config2'),
        ),

        # Test 3: UnsetLaunchConfiguration with push/pop
        SetLaunchConfiguration('config3', 'value_before_unset'),
        PushLaunchConfigurations(),
        UnsetLaunchConfiguration('config3'),
        # config3 is now unset, should use default or empty
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_unset_config',
            namespace='/unset_test',
        ),
        PopLaunchConfigurations(),  # config3 should be restored
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_restored_config3',
            namespace=LaunchConfiguration('config3'),
        ),

        # Test 4: ResetLaunchConfigurations
        SetLaunchConfiguration('config1', 'before_reset'),
        SetLaunchConfiguration('config2', 'before_reset'),
        SetLaunchConfiguration('config3', 'before_reset'),
        ResetLaunchConfigurations(),
        # All configs should be reset to initial defaults
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_after_reset',
            namespace=LaunchConfiguration('config1'),
        ),

        # Test 5: Multiple UnsetLaunchConfiguration
        DeclareLaunchArgument('temp_config_1', default_value='temp1'),
        DeclareLaunchArgument('temp_config_2', default_value='temp2'),
        UnsetLaunchConfiguration('temp_config_1'),
        UnsetLaunchConfiguration('temp_config_2'),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_after_unsets',
            namespace='/after_unsets',
        ),

        # Test 6: Push/Pop with multiple modifications
        SetLaunchConfiguration('config1', 'outer_scope'),
        SetLaunchConfiguration('config2', 'outer_scope'),
        PushLaunchConfigurations(),
        SetLaunchConfiguration('config1', 'inner_scope_1'),
        SetLaunchConfiguration('config2', 'inner_scope_2'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_in_inner_scope',
            namespace=LaunchConfiguration('config1'),
        ),
        PopLaunchConfigurations(),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_in_outer_scope',
            namespace=LaunchConfiguration('config2'),
        ),
    ])
