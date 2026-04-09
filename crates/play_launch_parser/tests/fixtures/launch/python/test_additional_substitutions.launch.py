"""Test additional ROS substitutions: ExecutableInPackage, FindPackage, Parameter."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackage, Parameter


def generate_launch_description():
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('package_name', default_value='demo_nodes_cpp'),
        DeclareLaunchArgument('executable_name', default_value='talker'),
        DeclareLaunchArgument('param_name', default_value='use_sim_time'),

        # Test 1: ExecutableInPackage with static values
        Node(
            package='demo_nodes_cpp',
            executable=ExecutableInPackage(
                package='demo_nodes_cpp',
                executable='talker'
            ),
            name='node_with_exec_in_pkg_static',
            namespace='/test',
        ),

        # Test 2: ExecutableInPackage with LaunchConfiguration
        Node(
            package='demo_nodes_cpp',
            executable=ExecutableInPackage(
                package=LaunchConfiguration('package_name'),
                executable=LaunchConfiguration('executable_name')
            ),
            name='node_with_exec_in_pkg_dynamic',
            namespace='/test',
        ),

        # Test 3: FindPackage with static value
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_find_pkg_static',
            namespace=FindPackage('demo_nodes_cpp'),
        ),

        # Test 4: FindPackage with LaunchConfiguration
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_find_pkg_dynamic',
            namespace=FindPackage(LaunchConfiguration('package_name')),
        ),

        # Test 5: Parameter substitution with static value
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_with_param_static',
            namespace=Parameter('robot_namespace'),
        ),

        # Test 6: Parameter substitution with LaunchConfiguration
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_param_dynamic',
            namespace=Parameter(LaunchConfiguration('param_name')),
        ),

        # Test 7: Combined substitutions - ExecutableInPackage in node executable
        Node(
            package=LaunchConfiguration('package_name'),
            executable=ExecutableInPackage(
                package='demo_nodes_cpp',
                executable='talker'
            ),
            name='node_combined_exec',
            namespace='/test',
        ),

        # Test 8: Combined substitutions - FindPackage and Parameter
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='node_combined_pkg_param',
            namespace=Parameter('namespace_param'),
        ),
    ])
