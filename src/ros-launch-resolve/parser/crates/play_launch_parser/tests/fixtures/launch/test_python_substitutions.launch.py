"""Python launch file testing advanced substitutions."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindPackageShare,
    EnvironmentVariable,
    ThisLaunchFileDir,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with various substitutions."""
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('package_name', default_value='demo_nodes_cpp'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Node with PathJoinSubstitution for config path
        Node(
            package=LaunchConfiguration('package_name'),
            executable='talker',
            name='talker_with_config',
            parameters=[{
                'config_path': PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('package_name')),
                    'config',
                    'default.yaml'
                ]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),

        # Node with EnvironmentVariable
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_with_env',
            parameters=[{
                'log_level': EnvironmentVariable('ROS_LOG_LEVEL', default_value='INFO'),
                'workspace': EnvironmentVariable('ROS_WORKSPACE'),
            }]
        ),

        # Node with ThisLaunchFileDir
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_with_dir',
            parameters=[{
                'launch_dir': ThisLaunchFileDir(),
            }]
        ),

        # Node with complex nested substitutions
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_complex',
            parameters=[{
                'data_path': PathJoinSubstitution([
                    FindPackageShare('demo_nodes_cpp'),
                    TextSubstitution(text='data'),
                    LaunchConfiguration('package_name'),
                    TextSubstitution(text='dataset.txt'),
                ]),
            }]
        ),
    ])
