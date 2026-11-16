# Included by parent.launch.py: the configuration its EnvironmentVariable
# default set, and the variable its SetEnvironmentVariable set, both reach an
# included file.

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    v = context.perform_substitution(LaunchConfiguration('use_agnocast'))
    e = context.perform_substitution(EnvironmentVariable('PL0086_SET'))
    return [Node(package='demo_nodes_cpp', executable='talker',
                 name='child_' + v + '_' + e)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
