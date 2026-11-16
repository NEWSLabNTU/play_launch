# The Python frontend's spelling of spawn.launch.xml (#0085, #0087;
# tests/tests/process_spawn.rs).
import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import ComposableNodeContainer, Node


def generate_launch_description():
    d = os.environ["PL_PROBE_DIR"]
    probe = f"{d}/probe.sh"
    return LaunchDescription(
        [
            GroupAction(
                [
                    UnsetEnvironmentVariable("PL_PROBE_INHERITED"),
                    UnsetEnvironmentVariable("DISPLAY"),
                    ExecuteProcess(cmd=[probe, "g_in"], name="g_in"),
                    Node(
                        package="demo_nodes_cpp",
                        executable="talker",
                        name="pl_probe_talker",
                        namespace="/pl_probe",
                        prefix=f"{probe} node_wrap",
                        cwd=f"{d}/work",
                    ),
                    ComposableNodeContainer(
                        name="pl_probe_c",
                        namespace="/pl_probe",
                        package="rclcpp_components",
                        executable="component_container",
                        composable_node_descriptions=[],
                        prefix=f"{probe} cont_wrap",
                    ),
                ],
                scoped=True,
            ),
            ExecuteProcess(cmd=[probe, "g_after"], name="g_after"),
            SetEnvironmentVariable("PL_PROBE_SET", "1"),
            UnsetEnvironmentVariable("PL_PROBE_SET"),
            ExecuteProcess(cmd=[probe, "set_unset"], name="set_unset"),
            UnsetEnvironmentVariable("PL_PROBE_TOP"),
            ExecuteProcess(cmd=[probe, "top"], name="top"),
            ExecuteProcess(cmd=[probe, "exe_pfx"], name="exe_pfx", prefix="env -u PL_PROBE_PFX"),
            ExecuteProcess(
                cmd=[probe, "exe_inner"], name="exe_wrapped", prefix=f"{probe} exe_wrap"
            ),
            ExecuteProcess(
                cmd=[probe, "cwd_exe"],
                name="cwd_exe",
                cwd=[EnvironmentVariable("PL_PROBE_DIR"), "/work"],
            ),
            ExecuteProcess(
                cmd=["PL_PROBE_SHELL=yes", probe, "shell_exe"], name="shell_exe", shell=True
            ),
        ]
    )
