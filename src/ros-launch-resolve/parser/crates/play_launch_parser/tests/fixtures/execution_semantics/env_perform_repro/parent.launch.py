# Issue #0086's reproduction, plus a node that carries the value performed.
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def _setup(context):
    enabled = context.perform_substitution(
        EnvironmentVariable("PL0086_ENABLE_AGNOCAST", default_value="0"))
    return [LogInfo(msg="agnocast=" + enabled),
            Node(package='demo_nodes_cpp', executable='talker', name='agnocast_' + enabled)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
