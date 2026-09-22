"""`ThisLaunchFileDir()` in a node's own fields, from a root `.launch.py`.

Issue 0034's residual. The include path in
`test_dirname_python_include.launch.py` resolves because the include machinery
performs substitutions on it; a node's parameters, parameter FILES, arguments
and remappings were captured as plain strings and handed to
`NodeCapture::to_record`, which takes no context — so a literal `$(dirname)`
reached the record AND the spawned command line, and the parameter file was
never read.

The last parameter is the control: a `LaunchConfiguration` must STILL be
preserved as `$(var ...)` for replay-time resolution.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='dirname_param_probe',
            parameters=[
                # Case 1: a substitution as a parameter VALUE.
                {'launch_dir': ThisLaunchFileDir()},
                {'config_path': PathJoinSubstitution([
                    ThisLaunchFileDir(), 'sub', 'thing.txt',
                ])},
                # Case 2: a substitution naming a parameter FILE.
                PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'includes', 'test_dirname_params.yaml',
                ]),
                # Control: preserved on purpose, not resolved here.
                {'replay_var': LaunchConfiguration('an_unset_argument')},
            ],
            # Case 3: the same through arguments / remappings.
            arguments=[PathJoinSubstitution([ThisLaunchFileDir(), 'arg.txt'])],
            remappings=[('in', PathJoinSubstitution([ThisLaunchFileDir(), 'topic']))],
        ),
        # An INCLUDED `.launch.py` must resolve against its OWN directory, not
        # this one — which is why the resolution happens per execution and not
        # at record-conversion time.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'includes',
                    'test_dirname_python_params_included.launch.py',
                ])
            )
        ),
    ])
