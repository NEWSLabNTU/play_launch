#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    """
    This function demonstrates file I/O in OpaqueFunction.
    It reads a YAML config file and uses the parameters.
    """
    # Create a temporary config file for testing
    import tempfile
    import os

    # Create temp file with YAML content
    yaml_content = """
/**:
  ros__parameters:
    test_param: 42
    test_string: "hello"
"""

    # Write to a temp file
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    temp_file.write(yaml_content)
    temp_file.close()

    try:
        # Read the YAML file
        with open(temp_file.name, 'r') as f:
            params = yaml.safe_load(f)["/**"]["ros__parameters"]

        # Create a node with parameters from the file
        node = Node(
            package='test_pkg',
            executable='test_exec',
            name='test_node_with_file_params',
            namespace='/test',
            parameters=[params],
        )

        return [node]
    finally:
        # Clean up temp file
        os.unlink(temp_file.name)


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
