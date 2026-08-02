"""Python launch file with container and composable nodes."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a container and composable nodes."""
    return LaunchDescription([
        ComposableNodeContainer(
            name='my_component_container',
            namespace='/test_ns',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='demo_nodes_cpp',
                    plugin='demo_nodes_cpp::Talker',
                    name='talker_node',
                ),
                ComposableNode(
                    package='demo_nodes_cpp',
                    plugin='demo_nodes_cpp::Listener',
                    name='listener_node',
                    namespace='/custom_ns',
                ),
            ]
        ),
    ])
