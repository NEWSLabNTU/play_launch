"""Minimal test: OpaqueFunction returns XML include with load_composable_node."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def create_nodes(context):
    """Create a container and return XML include with load_node."""
    # Get current ROS namespace from context
    target_namespace = context.launch_configurations.get('ros_namespace', None)

    # Create container in 'container_ns' namespace
    container = ComposableNodeContainer(
        namespace='container_ns',
        name='my_container',
        package='rclcpp_components',
        executable='component_container',
    )

    # Expected target: /test_ns/container_ns/my_container (if namespace context works)
    # Actual: need to check what gets captured
    target_container_name = '/container_ns/my_container'
    if target_namespace:
        target_container_name = f'{target_namespace}/container_ns/my_container'

    # Return XML include that should add load_node with correct namespace
    xml_include = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                LaunchConfiguration('fixture_dir'),
                'debug_ns_loadnode.launch.xml'
            ])
        ),
        launch_arguments=[
            ('target_container', target_container_name),
        ]
    )

    return [container, xml_include]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('fixture_dir'),
        OpaqueFunction(function=create_nodes),
    ])
