"""#0092: `SetEnvironmentVariable` and friends write `os.environ`, as launch's do.

Each node's name records what `os.environ` held where it was built. Stock
Humble starts: top_x, a1_7_home_d, a2_8_nohome_d, a3_7_home_d.
"""
import os

from launch import LaunchDescription
from launch.actions import (GroupAction, OpaqueFunction, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch_ros.actions import Node

# Set by the including XML file's <set_env>.
TOP = os.environ.get('PL_XML', 'none')

N = [0]


def _read(_context):
    # A fresh Node each call, so an action the walk dropped frees its
    # address for the next one: the walk must not read that as executing
    # one action twice.
    N[0] += 1
    home = 'home' if os.environ.get('HOME') is not None else 'nohome'
    name = 'a{}_{}_{}_{}'.format(N[0], os.environ.get('PL_SET', 'none'), home,
                                 os.environ.get('PL_DIRECT', 'none'))
    return [Node(package='demo_nodes_cpp', executable='talker', name=name)]


def _direct(_context):
    # A launch file's own write to os.environ stays.
    os.environ['PL_DIRECT'] = 'd'
    return []


def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker', name='top_' + TOP),
        SetEnvironmentVariable('PL_SET', '7'),
        OpaqueFunction(function=_direct),
        OpaqueFunction(function=_read),
        GroupAction([
            SetEnvironmentVariable('PL_SET', '8'),
            UnsetEnvironmentVariable('HOME'),
            OpaqueFunction(function=_read),
        ]),
        OpaqueFunction(function=_read),
    ])
