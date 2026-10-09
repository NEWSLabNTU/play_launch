from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('who', default_value='default_who'),
        DeclareLaunchArgument('flag', default_value='true'),
        SetLaunchConfiguration('child_let', 'set_by_child'),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['c_py_', LaunchConfiguration('who'), '_', LaunchConfiguration('parent_let')],
             condition=IfCondition(LaunchConfiguration('flag'))),
    ])
