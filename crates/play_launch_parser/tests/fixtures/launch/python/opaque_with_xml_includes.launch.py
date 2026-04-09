"""Test OpaqueFunction that returns XML includes with namespace context."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace


def create_monitor_nodes(context):
    """Create a container and load nodes via XML includes (Autoware pattern)."""
    # Get current ROS namespace from context
    target_namespace = context.launch_configurations.get('ros_namespace', None)

    # Build container name with namespace (Autoware pattern)
    target_container = make_namespace_absolute(
        prefix_namespace(target_namespace, 'monitor/container')
    )

    # Create the container
    container = ComposableNodeContainer(
        namespace='monitor',
        name='container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Talker',
                name='monitor_component',
                namespace='monitor',
            )
        ],
    )

    # Create XML includes that should get the namespace context
    xml_include_1 = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                LaunchConfiguration('fixture_dir'),
                'composable_node_simple.launch.xml'
            ])
        ),
        launch_arguments=[
            ('target_container', target_container),
            ('node_name', 'xml_node_1'),
        ]
    )

    xml_include_2 = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                LaunchConfiguration('fixture_dir'),
                'composable_node_simple.launch.xml'
            ])
        ),
        launch_arguments=[
            ('target_container', target_container),
            ('node_name', 'xml_node_2'),
        ]
    )

    return [container, xml_include_1, xml_include_2]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('fixture_dir'),
        OpaqueFunction(function=create_monitor_nodes),
    ])
