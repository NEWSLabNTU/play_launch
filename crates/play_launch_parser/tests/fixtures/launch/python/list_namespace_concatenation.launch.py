"""Test list concatenation in namespace field (Autoware pattern)."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return launch.LaunchDescription([
        # Container with list-concatenated namespace (Autoware pattern)
        # This is valid Python: namespace=["/", "my_container"]
        # The list gets concatenated into a single string
        ComposableNodeContainer(
            namespace=["/", "my_container"],
            name='container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='demo_nodes_cpp',
                    plugin='demo_nodes_cpp::Talker',
                    # Node namespace also supports list concatenation
                    namespace=["my", "_", "container"],
                    name='talker',
                )
            ],
        ),
    ])
