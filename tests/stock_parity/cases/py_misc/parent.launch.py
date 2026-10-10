import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap


def setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    nodes = [Node(package='demo_nodes_cpp', executable='talker', name='opaque_' + mode)]
    if mode == 'b':
        nodes.append(Node(package='demo_nodes_cpp', executable='talker', name='opaque_only_b'))
    return nodes


def generate_launch_description():
    d = os.path.dirname(os.path.abspath(__file__))
    child = os.path.join(d, 'child.launch.xml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='a', choices=['a', 'b']),
        DeclareLaunchArgument('gate', default_value='false'),
        OpaqueFunction(function=setup),
        GroupAction([
            SetLaunchConfiguration('unscoped_set', 'leaked'),
            PushRosNamespace('unscoped_ns'),
        ], scoped=False),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['after_unscoped_', LaunchConfiguration('unscoped_set', default='no')]),
        GroupAction([
            SetLaunchConfiguration('scoped_set', 'leaked'),
        ]),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['after_scoped_', LaunchConfiguration('scoped_set', default='no')]),
        GroupAction([
            SetParameter(name='gp', value='g'),
            SetRemap(src='rf', dst='rt'),
            SetEnvironmentVariable('PL_PY_ENV', 'pyenv'),
            Node(package='demo_nodes_cpp', executable='talker', name='in_setter_group'),
        ]),
        Node(package='demo_nodes_cpp', executable='talker', name='after_setter_group'),
        # forwarding=False: only the listed configurations reach the include
        GroupAction([
            IncludeLaunchDescription(AnyLaunchDescriptionSource(child)),
        ], forwarding=False, launch_configurations={'tag': 'fwd_false'}),
        GroupAction([
            IncludeLaunchDescription(AnyLaunchDescriptionSource(child),
                                     launch_arguments={'tag': 'cond_inc'}.items(),
                                     condition=IfCondition(LaunchConfiguration('gate'))),
        ]),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(child),
                                 launch_arguments={'tag': PythonExpression(["'pe_' + '", LaunchConfiguration('mode'), "'"])}.items()),
        Node(package='demo_nodes_cpp', executable='talker',
             name=['tail_', LaunchConfiguration('tag')]),
    ])
