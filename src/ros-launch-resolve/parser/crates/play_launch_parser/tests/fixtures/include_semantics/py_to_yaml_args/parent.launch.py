import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    d = os.path.dirname(os.path.abspath(__file__))
    return LaunchDescription([
        DeclareLaunchArgument('child_let', default_value='none'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.yaml')), launch_arguments=[('who', 'a'), ('flag', 'false')]),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.yaml')), launch_arguments=[('who', 'b')]),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.yaml')), launch_arguments=[]),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.yaml')), launch_arguments=[('who', 'd'), ('flag', 'true')]),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['p_py_', LaunchConfiguration('child_let')]),
    ])
