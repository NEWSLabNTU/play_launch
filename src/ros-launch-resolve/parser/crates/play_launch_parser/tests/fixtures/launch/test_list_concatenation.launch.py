#!/usr/bin/env python3
"""Test list concatenation in substitutions.

This tests the pattern where lists of substitutions are concatenated into
single strings, as used in Autoware's vehicle_info_param_file argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with list concatenation."""

    # Test 1: List with FindPackageShare and string
    path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=[
            FindPackageShare('test_package'),
            '/config/test.yaml'
        ],
        description='Path constructed from list'
    )

    # Test 2: List with nested LaunchConfiguration
    vehicle_model_arg = DeclareLaunchArgument(
        'vehicle_model',
        default_value='sample_vehicle',
    )

    vehicle_config_arg = DeclareLaunchArgument(
        'vehicle_config',
        default_value=[
            FindPackageShare([LaunchConfiguration('vehicle_model'), '_description']),
            '/config/vehicle.yaml'
        ],
        description='Nested substitution in list'
    )

    # Test 3: List with multiple substitution types
    complex_path_arg = DeclareLaunchArgument(
        'complex_path',
        default_value=[
            FindPackageShare('pkg1'),
            '/',
            LaunchConfiguration('vehicle_model'),
            '/',
            TextSubstitution(text='config.yaml'),
        ],
        description='Complex list with mixed types'
    )

    # Node using these arguments
    test_node = Node(
        package='test_pkg',
        executable='test_exec',
        name='list_concat_node',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'complex_path': LaunchConfiguration('complex_path'),
        }]
    )

    return LaunchDescription([
        path_arg,
        vehicle_model_arg,
        vehicle_config_arg,
        complex_path_arg,
        test_node,
    ])
