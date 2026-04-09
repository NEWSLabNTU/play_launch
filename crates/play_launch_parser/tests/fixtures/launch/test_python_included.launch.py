"""
Test fixture: Python launch file to be included by another launch file.
This file demonstrates basic node creation that will be included.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description with a node that uses launch configurations."""
    return LaunchDescription([
        Node(
            package='included_pkg',
            executable='included_node',
            name=LaunchConfiguration('node_name', default='default_included_node'),
            namespace='/included',
            parameters=[{
                'included_param': LaunchConfiguration('param_value', default='default_value'),
                'static_param': 'from_included_file',
            }]
        )
    ])
