"""
Test fixture: Python launch file that includes another Python launch file.
This demonstrates the IncludeLaunchDescription functionality.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with includes and nodes."""
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'node_name',
            default_value='custom_node',
            description='Name for the included node'
        ),

        DeclareLaunchArgument(
            'param_value',
            default_value='custom_value',
            description='Parameter value to pass to included file'
        ),

        # Include another Python launch file
        # Using relative path (ThisLaunchFileDir() would require substitution resolution)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('test_python_included.launch.py'),
            launch_arguments={
                'node_name': LaunchConfiguration('node_name'),
                'param_value': LaunchConfiguration('param_value'),
            }.items()
        ),

        # Add a node in the main file
        Node(
            package='main_pkg',
            executable='main_node',
            name='main_node',
            namespace='/main',
            parameters=[{
                'main_param': 'from_main_file',
            }]
        )
    ])
