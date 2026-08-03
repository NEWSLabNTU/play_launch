"""Attribute access on mock submodules: `import launch` + `launch.actions.X`.

pyo3's `add_submodule` binds a child under its `__name__`, which for a module
created as `PyModule::new(py, "launch.actions")` is the DOTTED name — so
`getattr(launch, "actions")` did not exist even though `from launch.actions
import X` worked, because that path goes through sys.modules instead.

`demo_nodes_cpp add_two_ints.launch.py` is the stock file that hit this:
"module 'launch' has no attribute 'actions'". This fixture is the same shape,
minus the ROS dependency.
"""

import launch
import launch_ros.actions


def generate_launch_description():
    client = launch_ros.actions.Node(
        package='demo_nodes_cpp', executable='add_two_ints_client', output='screen')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='add_two_ints_server', output='screen'),
        client,
        # Every dotted access below was unreachable before the fix.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=client,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
