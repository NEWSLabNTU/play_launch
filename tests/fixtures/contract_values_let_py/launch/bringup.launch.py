# Phase 94 W2: contract_values_let through the Python frontend. `timeout` is
# reassigned, `inner_hz` is set only inside a scoped group, and `child_hz` is
# set by an included file and reaches this one.
import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    d = os.path.dirname(os.path.abspath(__file__))
    return LaunchDescription([
        SetLaunchConfiguration('timeout', '0.5'),
        Node(package='demo_nodes_cpp', executable='listener', name='gate'),
        SetLaunchConfiguration('timeout', '2.0'),
        Node(package='demo_nodes_cpp', executable='listener', name='other'),
        GroupAction(scoped=True, actions=[
            SetLaunchConfiguration('inner_hz', '5'),
            Node(package='demo_nodes_cpp', executable='talker', name='inner'),
        ]),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(d, 'child.launch.py'))),
        Node(package='demo_nodes_cpp', executable='talker', name='source'),
    ])
