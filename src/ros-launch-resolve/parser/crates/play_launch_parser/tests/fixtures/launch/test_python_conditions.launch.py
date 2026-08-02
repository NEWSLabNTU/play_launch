"""Python launch file testing conditions."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with conditional nodes."""
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('enable_debug', default_value='true'),

        # Log messages (for testing LogInfo action)
        LogInfo(msg='Starting launch with conditional nodes'),
        LogInfo(msg='Simulation mode controlled by use_sim argument'),

        # Node that launches only when use_sim is true
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='sim_talker',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),

        # Node that launches only when use_sim is false
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='real_talker',
            condition=UnlessCondition(LaunchConfiguration('use_sim'))
        ),

        # Node that launches only when enable_debug is true
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='debug_listener',
            condition=IfCondition(LaunchConfiguration('enable_debug'))
        ),

        # Node that always launches (no condition)
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='main_listener'
        ),
    ])
