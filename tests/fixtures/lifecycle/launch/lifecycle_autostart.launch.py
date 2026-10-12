"""`LifecycleNode(autostart=...)` — Jazzy only (phase 86).

The Python twin of `lifecycle_autostart.launch.xml`: the two must resolve to
the same model, `lc_on` with `lifecycle_autostart: active`, `lc_off` and
`lc_unsaid` with none.
"""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='lifecycle',
            executable='lifecycle_talker',
            name='lc_on',
            namespace='',
            autostart=True,
        ),
        LifecycleNode(
            package='lifecycle',
            executable='lifecycle_talker',
            name='lc_off',
            namespace='',
            autostart=False,
        ),
        LifecycleNode(
            package='lifecycle',
            executable='lifecycle_talker',
            name='lc_unsaid',
            namespace='',
        ),
    ])
