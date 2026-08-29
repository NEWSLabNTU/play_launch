"""A node given as an absolute executable, with no package.

`launch_ros.actions.Node` allows this — it is how a plain program is run under the
launch system — and both the record type (`package: str | None`) and the Rust spawn
path (`from_raw_executable`) model it. The dump visitor did not: it substituted
`node.node_package` unconditionally and died with
`TypeError: 'NoneType' object is not iterable`, naming neither the node nor the
field (issue 0026).

Python, not XML, because the XML frontend requires a package attribute — this shape
is only reachable from a Python launch file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(executable="/bin/sleep", arguments=["3600"], output="screen"),
        ]
    )
