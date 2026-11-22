from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='parameter_blackboard',
             name='opted_in',
             parameters=[ParameterFile(params_file, allow_substs=True)]),
        Node(package='demo_nodes_cpp', executable='parameter_blackboard',
             name='as_written',
             parameters=[ParameterFile(params_file)]),
    ])
