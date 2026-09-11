# Included by test_required_py_outer_*.launch.xml and test_required_py_outer.launch.py.
# `required` has no default; `opaque_required` is declared inside an OpaqueFunction,
# which launch's include-time check cannot see, so it is only demanded when it
# executes unset.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    return [DeclareLaunchArgument("opaque_required", description="declared opaquely")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("required", description="must be passed on the include"),
            DeclareLaunchArgument("optional", default_value="1"),
            OpaqueFunction(function=launch_setup),
            Node(package="demo_nodes_cpp", executable="talker", name=LaunchConfiguration("required")),
        ]
    )
