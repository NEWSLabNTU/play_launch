from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, SetParameter, SetRemap


def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker', name='py_before'),
        SetParameter(name='p_py', value='4'),
        SetRemap(src='r_py', dst='w'),
        Node(package='demo_nodes_cpp', executable='talker', name='py_after'),
        GroupAction([
            SetParameter(name='p_py_group', value=5.5),
            Node(package='demo_nodes_cpp', executable='talker', name='py_in_group'),
        ]),
        Node(package='demo_nodes_cpp', executable='talker', name='py_after_group'),
    ])
