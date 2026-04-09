"""Python launch file that doesn't use imports."""

import sys

def generate_launch_description():
    """Generate launch description without imports."""
    # Access modules directly from sys.modules
    launch = sys.modules['launch']
    launch_ros_actions = sys.modules['launch_ros.actions']

    LaunchDescription = launch.LaunchDescription
    Node = launch_ros_actions.Node

    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='sys_modules_test',
        ),
    ])
