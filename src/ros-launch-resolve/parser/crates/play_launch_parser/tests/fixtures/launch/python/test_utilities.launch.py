"""Test launch_ros.utilities functions (Autoware uses these)."""

import launch
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace


def test_utilities(context):
    """Test namespace utility functions."""
    # Test make_namespace_absolute
    abs1 = make_namespace_absolute("robot1")  # Should be "/robot1"
    abs2 = make_namespace_absolute("/robot1")  # Should be "/robot1"
    abs3 = make_namespace_absolute("")  # Should be "/"

    # Test prefix_namespace
    prefixed1 = prefix_namespace("/system", "monitor")  # Should be "/system/monitor"
    prefixed2 = prefix_namespace(None, "monitor")  # Should be "/monitor"
    prefixed3 = prefix_namespace("/", "monitor")  # Should be "/monitor"
    prefixed4 = prefix_namespace("/system", "/monitor")  # Should be "/monitor" (absolute override)

    # Create nodes with computed namespaces
    return [
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_abs',
            namespace=abs1,
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='test_prefixed',
            namespace=prefixed1,
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=test_utilities),
    ])
