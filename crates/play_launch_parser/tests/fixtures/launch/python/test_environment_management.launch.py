"""Test environment management actions: PushEnvironment, PopEnvironment, ResetEnvironment, AppendEnvironmentVariable."""

import launch
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
    PushEnvironment,
    PopEnvironment,
    ResetEnvironment,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('custom_path', default_value='/opt/custom'),
        DeclareLaunchArgument('lib_path', default_value='/opt/custom/lib'),

        # Test 1: Basic PushEnvironment and PopEnvironment
        # Set an environment variable, push, modify it, then pop to restore
        SetEnvironmentVariable('TEST_VAR', 'initial_value'),
        PushEnvironment(),
        SetEnvironmentVariable('TEST_VAR', 'modified_value'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_modified_env',
            namespace='/test',
        ),
        PopEnvironment(),  # TEST_VAR should be restored to 'initial_value'
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_restored_env',
            namespace='/test',
        ),

        # Test 2: AppendEnvironmentVariable with default settings (append to end)
        AppendEnvironmentVariable('PATH', LaunchConfiguration('custom_path')),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_appended_path',
            namespace='/test',
        ),

        # Test 3: AppendEnvironmentVariable with prepend
        AppendEnvironmentVariable(
            'LD_LIBRARY_PATH',
            LaunchConfiguration('lib_path'),
            prepend=True,
            separator=':',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_prepended_lib',
            namespace='/test',
        ),

        # Test 4: Nested push/pop
        SetEnvironmentVariable('NESTED_VAR', 'level_0'),
        PushEnvironment(),
        SetEnvironmentVariable('NESTED_VAR', 'level_1'),
        PushEnvironment(),
        SetEnvironmentVariable('NESTED_VAR', 'level_2'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_at_level_2',
            namespace='/test',
        ),
        PopEnvironment(),  # Back to level_1
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_at_level_1',
            namespace='/test',
        ),
        PopEnvironment(),  # Back to level_0
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_at_level_0',
            namespace='/test',
        ),

        # Test 5: ResetEnvironment
        SetEnvironmentVariable('RESET_TEST', 'should_be_reset'),
        ResetEnvironment(),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_after_reset',
            namespace='/test',
        ),

        # Test 6: AppendEnvironmentVariable with custom separator
        AppendEnvironmentVariable(
            'CUSTOM_LIST',
            'item1',
            separator=',',
        ),
        AppendEnvironmentVariable(
            'CUSTOM_LIST',
            'item2',
            separator=',',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_custom_list',
            namespace='/test',
        ),

        # Test 7: UnsetEnvironmentVariable with push/pop
        SetEnvironmentVariable('UNSET_TEST', 'value'),
        PushEnvironment(),
        UnsetEnvironmentVariable('UNSET_TEST'),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_unset_var',
            namespace='/test',
        ),
        PopEnvironment(),  # UNSET_TEST should be restored
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_restored_var',
            namespace='/test',
        ),
    ])
