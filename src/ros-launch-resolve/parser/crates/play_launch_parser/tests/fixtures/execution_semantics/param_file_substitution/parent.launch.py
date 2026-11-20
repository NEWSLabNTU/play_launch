# A composable node's ParameterFile over a substitution that performs to a
# file with no YAML extension -- nav2_common's RewrittenYaml writes
# tempfile.NamedTemporaryFile(mode='w', delete=False), which has none
# (issue #0102). LoadComposableNodes opens the file whatever its name and
# sends the values matched to the node, so stock gives /planning/map_server
# topic_name: wild_map. The bare-substitution form is checked beside it.

import os
import tempfile

import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile

HERE = os.path.dirname(os.path.abspath(__file__))


class CopiedYaml(launch.Substitution):
    """Copies a YAML file to a temporary file with no extension."""

    def __init__(self, source_file):
        super().__init__()
        self.__source_file = source_file

    def describe(self):
        return ''

    def perform(self, context):
        copy = tempfile.NamedTemporaryFile(mode='w', delete=False)
        with open(self.__source_file) as f:
            copy.write(f.read())
        copy.close()
        return copy.name


def generate_launch_description():
    source = os.path.join(HERE, 'wild.yaml')
    return LaunchDescription([
        ComposableNodeContainer(name='container', namespace='',
                                package='rclcpp_components',
                                executable='component_container'),
        LoadComposableNodes(target_container='/container', composable_node_descriptions=[
            ComposableNode(package='nav2_map_server', plugin='nav2_map_server::MapServer',
                           name='map_server', namespace='planning',
                           parameters=[ParameterFile(CopiedYaml(source), allow_substs=True)]),
            ComposableNode(package='nav2_map_server', plugin='nav2_map_server::MapServer',
                           name='map_server', namespace='bare',
                           parameters=[CopiedYaml(source)]),
        ]),
    ])
