#!/usr/bin/env python3
"""Test IncludeLaunchDescription with list-based arguments.

This tests the pattern where arguments passed to included files contain
lists that need to be concatenated, as seen in Autoware's global_params.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare


def generate_launch_description():
    """Generate launch description with complex include arguments."""

    vehicle_model_arg = DeclareLaunchArgument(
        'vehicle_model',
        default_value='sample_vehicle',
        description='Vehicle model name'
    )

    # Include another launch file with list-based argument
    load_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('test_package'),
            '/launch/params.launch.py'
        ]),
        launch_arguments={
            # This list should be concatenated into a single path string
            'vehicle_info_param_file': [
                FindPackageShare([LaunchConfiguration('vehicle_model'), '_description']),
                '/config/vehicle_info.param.yaml'
            ],
            'config_dir': [
                FindPackageShare('test_package'),
                '/config'
            ],
        }.items(),
    )

    return LaunchDescription([
        vehicle_model_arg,
        load_params,
    ])
