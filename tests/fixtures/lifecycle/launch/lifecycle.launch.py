"""A lifecycle node with no `autostart` (phase 86) — valid on Humble and Jazzy.

Both parsers must model `lc_talker` as a lifecycle node with no autostart, and
the plain `talker` as neither.
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle',
            executable='lifecycle_talker',
            name='lc_talker',
            namespace='',
        ),
        Node(package='demo_nodes_cpp', executable='talker', name='talker'),
    ])
