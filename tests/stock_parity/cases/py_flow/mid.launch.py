import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    d = os.path.dirname(os.path.abspath(__file__))
    return LaunchDescription([
        DeclareLaunchArgument('py_declared', default_value='decl'),
        SetLaunchConfiguration('py_set', 'one'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.xml'))),
        SetLaunchConfiguration('py_set', 'two'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.yaml'))),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['mid_', LaunchConfiguration('passed'), '_', LaunchConfiguration('py_set')]),
    ])
