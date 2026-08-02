#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='my_container',
        description='Name of the container'
    )

    full_container_path_arg = DeclareLaunchArgument(
        'full_container_path',
        default_value='/test/my_container',
        description='Full path to container'
    )

    # Create a container
    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='/test',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='initial_pkg',
                plugin='InitialPlugin',
                name='initial_node',
            )
        ],
    )

    # Load additional nodes into the container using LaunchConfiguration with full path
    load_nodes_with_config = LoadComposableNodes(
        target_container=LaunchConfiguration('full_container_path'),
        composable_node_descriptions=[
            ComposableNode(
                package='dynamic_pkg1',
                plugin='DynamicPlugin1',
                name='dynamic_node_1',
                parameters=[{'param1': 'value1'}],
                remappings=[('input', '/test/input')],
            ),
            ComposableNode(
                package='dynamic_pkg2',
                plugin='DynamicPlugin2',
                name='dynamic_node_2',
                namespace='/custom_ns',
                parameters=[{'param2': 'value2'}],
            ),
        ],
    )

    # Load nodes using string container reference
    load_nodes_with_string = LoadComposableNodes(
        target_container='/test/my_container',
        composable_node_descriptions=[
            ComposableNode(
                package='string_pkg',
                plugin='StringPlugin',
                name='string_node',
            ),
        ],
    )

    return LaunchDescription([
        container_name_arg,
        full_container_path_arg,
        container,
        load_nodes_with_config,
        load_nodes_with_string,
    ])
