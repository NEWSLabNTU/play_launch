# Substitutions the launch file defines itself, in the shape of
# nav2_common.launch (RewrittenYaml, HasNodeParams) -- issue #0073. The class
# bodies and perform() use exactly the `launch` surface nav2_common uses:
# launch.Substitution, launch.SomeSubstitutionsType, launch.LaunchContext,
# launch.Condition, launch.utilities.normalize_to_list_of_substitutions and
# launch.utilities.perform_substitutions.

import os
import tempfile
from typing import List, Optional, Text

import launch
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


class RewrittenYaml(launch.Substitution):
    """Rewrites leaf parameters of a YAML file into a temporary file."""

    def __init__(self, source_file: launch.SomeSubstitutionsType,
                 param_rewrites, root_key: Optional[launch.SomeSubstitutionsType] = None,
                 condition: Optional[launch.Condition] = None) -> None:
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {
            k: normalize_to_list_of_substitutions(v) for k, v in param_rewrites.items()
        }

    @property
    def name(self) -> List[launch.Substitution]:
        return self.__source_file

    def describe(self) -> Text:
        return ''

    def perform(self, context: launch.LaunchContext) -> Text:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        rewrites = {
            k: launch.utilities.perform_substitutions(context, v)
            for k, v in self.__param_rewrites.items()
        }
        with open(yaml_filename) as f:
            data = yaml.safe_load(f)
        for node in data.values():
            for key, value in rewrites.items():
                if key in node['ros__parameters']:
                    node['ros__parameters'][key] = yaml.safe_load(value)
        yaml.dump(data, rewritten)
        rewritten.close()
        return rewritten.name


class HasNodeParams(launch.Substitution):
    """'True' when the YAML file has parameters for `node_name`."""

    def __init__(self, source_file: launch.SomeSubstitutionsType,
                 node_name: Text) -> None:
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__node_name = node_name

    @property
    def name(self) -> List[launch.Substitution]:
        return self.__source_file

    def perform(self, context: launch.LaunchContext) -> Text:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        with open(yaml_filename) as f:
            data = yaml.safe_load(f)
        return str(self.__node_name in data)


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    configured = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(HERE, 'params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='planner',
            parameters=[configured],
        ),
        # Present in the file: started.
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='has_params',
            condition=IfCondition(HasNodeParams(params_file, 'planner')),
        ),
        # Absent from the file: not started.
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='lacks_params',
            condition=IfCondition(HasNodeParams(params_file, 'controller')),
        ),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='unless_lacks',
            condition=UnlessCondition(HasNodeParams(params_file, 'controller')),
        ),
    ])
