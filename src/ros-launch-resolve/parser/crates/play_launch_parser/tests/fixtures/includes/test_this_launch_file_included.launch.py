"""Included by `launch/test_this_launch_file.launch.py`.

It lives in `includes/`, one directory over from its includer, so its
`ThisLaunchFile()` must be ITS OWN path — the check that resolution happens
against the file that DECLARED the node rather than the root launch file.
"""

from launch import LaunchDescription
from launch.substitutions import ThisLaunchFile
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='this_launch_file_included_probe',
            parameters=[{'launch_file': ThisLaunchFile()}],
        ),
    ])
