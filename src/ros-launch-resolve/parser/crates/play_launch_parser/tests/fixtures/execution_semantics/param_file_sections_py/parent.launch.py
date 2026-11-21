# The shape of Nav2's bringup on Humble: the container is a plain Node
# running component_container, composables are loaded into it by name, and
# every composable is given the whole nav2_params.yaml. launch_ros's
# to_parameters_list selects each node's own section from it (keys without
# a leading slash, a costmap's nested under its namespace).

import os

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    params = os.path.join(HERE, 'params.yaml')
    return LaunchDescription([
        Node(package='rclcpp_components', executable='component_container',
             name='nav2_container', output='screen'),
        LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                ComposableNode(package='nav2_map_server',
                               plugin='nav2_map_server::MapServer',
                               name='map_server', parameters=[params]),
                ComposableNode(package='nav2_costmap_2d',
                               plugin='nav2_costmap_2d::Costmap2DROS',
                               name='local_costmap', namespace='local_costmap',
                               parameters=[params]),
            ]),
    ])
