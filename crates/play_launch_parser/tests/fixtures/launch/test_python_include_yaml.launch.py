import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    yaml_file = os.path.join(
        os.path.dirname(__file__),
        'test_python_include_yaml_target.launch.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(yaml_file),
        ),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='after_yaml_include',
        ),
    ])
