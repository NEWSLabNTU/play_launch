from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag'),
        DeclareLaunchArgument('off', default_value='false'),
        GroupAction([
            Node(package='demo_nodes_cpp', executable='talker', name='py_group_if_false'),
            SetLaunchConfiguration('leak_from_false_group', 'yes'),
        ], condition=IfCondition(LaunchConfiguration('off'))),
        GroupAction([
            Node(package='demo_nodes_cpp', executable='talker', name='py_group_unless_false'),
        ], condition=UnlessCondition(LaunchConfiguration('off'))),
        SetLaunchConfiguration('cond_set', 'should_not', condition=IfCondition('false')),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['py_', LaunchConfiguration('tag')]),
        Node(package='demo_nodes_cpp', executable='talker', name='py_eq',
             condition=LaunchConfigurationEquals('tag', 'py_if_true')),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['py_cs_', LaunchConfiguration('cond_set', default='unset')]),
    ])
