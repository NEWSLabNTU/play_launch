from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace('py_pushed'),
        Node(package='demo_nodes_cpp', executable='talker', name='in_py'),
        GroupAction([
            PushRosNamespace('py_group'),
            Node(package='demo_nodes_cpp', executable='talker', name='in_py_group'),
        ]),
        Node(package='demo_nodes_cpp', executable='talker', name='in_py_after_group'),
    ])
