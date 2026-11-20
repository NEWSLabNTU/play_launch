# Issue #0101: `target_container` given as a tuple of substitutions, as
# nav2_bringup's navigation_launch.py passes
# `(namespace, '/', container_name)`. Stock performs the tuple and joins it.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    name = LaunchConfiguration('container_name')
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value=''),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        ComposableNodeContainer(name='nav2_container', namespace='', package='rclcpp_components',
                                executable='component_container'),
        LoadComposableNodes(target_container=(ns, '/', name), composable_node_descriptions=[
            ComposableNode(package='demo_nodes_cpp', plugin='demo_nodes_cpp::Talker',
                           name='talker')]),
    ])
