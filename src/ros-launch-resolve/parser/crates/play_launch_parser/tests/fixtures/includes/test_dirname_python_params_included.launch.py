"""Included by `launch/test_dirname_python_params.launch.py`.

It lives in `includes/`, one directory over from its includer, so its
`$(dirname)` must be `fixtures/includes` — the check that resolution happens
against the file that DECLARED the node rather than the root launch file.
"""

from launch.substitutions import ThisLaunchFileDir
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='dirname_included_probe',
            parameters=[{'launch_dir': ThisLaunchFileDir()}],
        ),
    ])
