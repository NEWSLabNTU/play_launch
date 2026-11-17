"""#0095: the Python spelling of ../awnode, the upstream AutowareNode class."""

from autoware_agnocast_wrapper.actions import AutowareNode
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, PushRosNamespace


def generate_launch_description():
    return LaunchDescription(
        [
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
            ),
            AutowareNode(package="demo_nodes_cpp", executable="talker", mode="rclcpp"),
        ]
    )
