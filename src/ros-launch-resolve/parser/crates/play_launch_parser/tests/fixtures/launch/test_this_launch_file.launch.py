"""`ThisLaunchFile()` in a node's own fields, from a root `.launch.py`.

Issue 0041. The `pyexec` stand-in used to return the literal
`$(this-launch-file)`, a token no grammar on either side of the boundary knew,
so it reached the record, the model and the spawned command line on every path.
ROS 2 substitutes the launch file's ABSOLUTE PATH — and exposes this very class
to the frontends as `filename` (`@expose_substitution('filename')` in
`launch/substitutions/this_launch_file.py`), which is why the token the mock
emits is `$(filename)` and not a third one.

The marker below is load-bearing: the node names THIS FILE as its parameter
file, so the record must hold this file's own text. That is the silent case —
`NodeCapture::to_record` stores a params file's CONTENT via
`fs::read_to_string(path).unwrap_or(path)`, so an unresolved path is stored
where content belongs and the file is never read, with no diagnostic.
this_launch_file_probe_loaded

The last parameter is the control: a `LaunchConfiguration` must STILL be
preserved as `$(var ...)` for replay-time resolution.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFile,
    ThisLaunchFileDir,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='this_launch_file_probe',
            parameters=[
                # Case 1: a substitution as a parameter VALUE.
                {'launch_file': ThisLaunchFile()},
                # Case 2: a substitution naming a parameter FILE — the silent
                # one. `ParameterFile` and not a bare `ThisLaunchFile()`,
                # because a bare substitution is classified as a parameter file
                # only when its UNRESOLVED string ends in `.yaml`
                # (`node.rs::parse_parameters` case 5), and any `.yaml` path
                # built from `ThisLaunchFile()` would have to treat the launch
                # file as a directory (`…/f.launch.py/../x.yaml`, ENOTDIR). So
                # this wrapper is the only form in which the case is
                # realizable. Nothing at this layer parses a parameter file —
                # `to_record` stores its raw bytes — so naming this file itself
                # is a legitimate probe: if the path resolved the record holds
                # the marker above, and if it did not it holds the token.
                ParameterFile(ThisLaunchFile()),
                # Control: preserved on purpose, not resolved here.
                {'replay_var': LaunchConfiguration('an_unset_argument')},
            ],
            # Case 3: the same through arguments / remappings.
            arguments=[ThisLaunchFile()],
            remappings=[('in', PathJoinSubstitution([ThisLaunchFile(), 'topic']))],
        ),
        # An INCLUDED `.launch.py` must resolve against ITSELF, not the root
        # launch file — which is why the captures are resolved per execution
        # and not at record-conversion time, where the context has been
        # restored to the root.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(), '..', 'includes',
                    'test_this_launch_file_included.launch.py',
                ])
            )
        ),
    ])
