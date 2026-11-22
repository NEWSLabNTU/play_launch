#!/usr/bin/env python3
"""Test ParameterFile from launch_ros.parameter_descriptions.

This tests the pattern used in simple_planning_simulator where ParameterFile
wraps parameter file paths with allow_substs option.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import launch_ros.parameter_descriptions


def launch_setup(context, *args, **kwargs):
    """Create node with ParameterFile."""
    # Test ParameterFile with substitution
    param_file = launch_ros.parameter_descriptions.ParameterFile(
        param_file=LaunchConfiguration('param_file').perform(context),
        allow_substs=True
    )

    node = Node(
        package='test_pkg',
        executable='test_exec',
        name='param_file_node',
        parameters=[param_file],
    )

    return [node]


def generate_launch_description():
    """Generate launch description with ParameterFile."""
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        # A file that exists: with allow_substs=True launch_ros opens it
        # (ParameterFile.evaluate), and a missing one fails the launch.
        default_value=[ThisLaunchFileDir(), '/test_parameter_file.params.yaml'],
        description='Parameter file path'
    )

    return LaunchDescription([
        param_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
