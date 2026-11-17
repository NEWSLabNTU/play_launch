# A respawning process with a conditional exit handler (#0090): it exits with
# the `code` launch argument, and the handler shuts the launch down when that
# code is 2 (a usage error) and lets it respawn otherwise. Each start appends a
# line to $PL_PROBE_OUT/starts.
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

STARTS = os.path.join(os.environ["PL_PROBE_OUT"], "starts")


def on_exit(event, _context):
    if event.returncode != 2:
        return []
    return [LogInfo(msg="usage error: shutting down"),
            EmitEvent(event=Shutdown(reason="usage error"))]


def generate_launch_description():
    stub = ExecuteProcess(
        cmd=["python3", "-c",
             f"import sys; open({STARTS!r}, 'a').write('x\\n'); sys.exit(int(sys.argv[1]))",
             LaunchConfiguration("code")],
        name="stub", respawn=True, respawn_delay=1.0,
    )
    return LaunchDescription([
        DeclareLaunchArgument("code", default_value="2"),
        stub,
        RegisterEventHandler(OnProcessExit(target_action=stub, on_exit=on_exit)),
    ])
