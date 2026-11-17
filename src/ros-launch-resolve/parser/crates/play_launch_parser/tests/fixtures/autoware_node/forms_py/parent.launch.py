"""#0095, the Python spelling of ../forms/parent.launch.xml.

The action is the upstream class, imported from ../python (a copy of
autoware_agnocast_wrapper's launch extension); in stock launch it runs on
the real `launch`, under play_launch on its stand-ins.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python"))

from autoware_agnocast_wrapper.actions import AutowareNode  # noqa: E402
from launch import LaunchDescription  # noqa: E402
from launch.actions import SetEnvironmentVariable  # noqa: E402
from launch.substitutions import EnvironmentVariable  # noqa: E402
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace  # noqa: E402

PREFIX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "prefix")


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "AMENT_PREFIX_PATH",
                [PREFIX, ":", EnvironmentVariable("AMENT_PREFIX_PATH", default_value="")],
            ),
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container",
                name="c",
                namespace="",
                composable_node_descriptions=[],
            ),
            PushRosNamespace("aw"),
            AutowareNode(
                package="demo_nodes_cpp",
                executable="talker",
                name="talker_comp",
                target="/c",
                output="screen",
                parameters=[{"negative": True}],
                remappings=[("chatter", "chat")],
                additional_env={"FOO": "bar"},
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            AutowareNode(
                package="demo_nodes_cpp",
                executable="listener",
                name="listener_proc",
                output="screen",
                parameters=[{"negative": True}],
                remappings=[("chatter", "chat")],
                additional_env={"FOO": "bar", "ENABLE_AGNOCAST": "1"},
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ]
    )
