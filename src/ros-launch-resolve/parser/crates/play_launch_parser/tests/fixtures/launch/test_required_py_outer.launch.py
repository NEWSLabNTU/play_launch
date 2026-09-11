# Python including Python without launch_arguments while the value sits in scope:
# launch refuses this too (issue 0030).
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

here = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("required", default_value="from_parent_scope"),
            DeclareLaunchArgument("opaque_required", default_value="from_parent_scope"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(here, "test_required_py_inner.launch.py"))
            ),
        ]
    )
