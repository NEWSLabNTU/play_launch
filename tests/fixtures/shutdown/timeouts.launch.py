# Two processes that ignore SIGINT and SIGTERM (#0084): `declared` gives
# launch 2 s before the SIGTERM and 2 s more before the SIGKILL; `default`
# declares nothing and gets launch's 5 s and 5 s. Each writes what it receives
# to $PL_PROBE_OUT/<name>.
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ["PL_PROBE_OUT"]


def probe(name, **kwargs):
    return ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "probe.py"), os.path.join(OUT, name)],
        name=name,
        **kwargs,
    )


def generate_launch_description():
    return LaunchDescription([
        probe("declared", sigterm_timeout="2", sigkill_timeout="2"),
        probe("default"),
    ])
