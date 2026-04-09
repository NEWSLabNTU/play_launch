from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='python_talker',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='python_listener',
        ),
    ])
