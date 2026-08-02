"""RFC-0060 layer-2 isolation fixture (Python frontend).

Self-contained on purpose -- see the XML sibling. `import launch` FAILS in the
stripped environment this fixture is resolved under; the parser supplies its
own `launch` / `launch_ros` API through the pyo3 mock modules in
`parser/crates/play_launch_parser/src/python/api/`. That is the property under
test: layer 2 needs CPython, not a ROS installation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="isolation_py_talker",
                namespace="isolation",
            )
        ]
    )
