#!/usr/bin/env python3
"""Test fixture for substitution types and list concatenation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with various substitution types."""

    # Test list concatenation - should result in single concatenated string
    pkg_path_arg = DeclareLaunchArgument(
        'pkg_path',
        default_value=[
            FindPackageShare('test_package'),
            '/config/test.yaml'
        ],
        description='Test list concatenation with FindPackageShare'
    )

    # Test nested LaunchConfiguration in list
    vehicle_model_arg = DeclareLaunchArgument(
        'vehicle_model',
        default_value='sample_vehicle',
        description='Vehicle model name'
    )

    vehicle_info_path_arg = DeclareLaunchArgument(
        'vehicle_info_path',
        default_value=[
            FindPackageShare([LaunchConfiguration('vehicle_model'), '_description']),
            '/config/vehicle_info.param.yaml'
        ],
        description='Test nested substitutions in list'
    )

    # Test simple scalar types
    string_arg = DeclareLaunchArgument('test_string', default_value='hello')
    int_arg = DeclareLaunchArgument('test_int', default_value='42')
    float_arg = DeclareLaunchArgument('test_float', default_value='3.14')
    bool_arg = DeclareLaunchArgument('test_bool', default_value='true')

    # Create a node using these substitutions
    test_node = Node(
        package='test_pkg',
        executable='test_exec',
        name='test_node',
        parameters=[{
            'string_param': LaunchConfiguration('test_string'),
            'int_param': LaunchConfiguration('test_int'),
            'float_param': LaunchConfiguration('test_float'),
            'bool_param': LaunchConfiguration('test_bool'),
            'path_param': LaunchConfiguration('pkg_path'),
            'vehicle_info': LaunchConfiguration('vehicle_info_path'),
        }]
    )

    return LaunchDescription([
        pkg_path_arg,
        vehicle_model_arg,
        vehicle_info_path_arg,
        string_arg,
        int_arg,
        float_arg,
        bool_arg,
        test_node,
    ])
