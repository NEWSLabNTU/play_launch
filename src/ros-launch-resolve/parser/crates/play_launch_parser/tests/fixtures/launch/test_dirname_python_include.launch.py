"""`ThisLaunchFileDir()` from a root `.launch.py`.

The mock captures `ThisLaunchFileDir()` as the string `$(dirname)`, which the
host resolves against the context's current file — and the Python execution
path was the one frontend that never set one, so this include had no directory
to resolve against at all (issue 0034, second half).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    ThisLaunchFileDir(),
                    'test_dirname_bare_invocation.launch.xml',
                ])
            )
        ),
    ])
