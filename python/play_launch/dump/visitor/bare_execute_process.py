"""Record a plain `ExecuteProcess` (one that is not a ROS `Node`) instead of running it.

Issue #0066: the action visitor had no branch for `ExecuteProcess`, so it fell
through to stock `Action.execute()`, which STARTS the process. Every plain
`ExecuteProcess` in a `.launch.py` ran during `dump`/`resolve --parser python`
and was absent from the record. This mirrors the Rust parser's
`ExecuteProcess` capture (pyexec `simple_actions.rs`): a record with no
package, the expanded command split into executable and arguments, the given
name, the additional environment, and whether `on_exit` shuts the launch down.
"""

import launch
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

from ..launch_dump import LaunchDump, NodeRecord
from .execute_process import visit_execute_process
from .node import _classify_on_exit


def _strip_counter(final_name: str) -> str:
    """`required-2` -> `required`: launch appends a process counter to every name."""
    stem, sep, counter = final_name.rpartition("-")
    if sep and counter.isdigit():
        return stem
    return final_name


def visit_bare_execute_process(
    process: ExecuteProcess, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    # Registers the process's event handlers and expands its description
    # (`prepare`), exactly as for a Node -- and, like it, starts nothing.
    ret = visit_execute_process(process, context, dump)

    description = process._ExecuteLocal__process_description
    cmd = [str(part) for part in (description.final_cmd or [])]
    if not cmd:
        return ret
    if process.shell:
        # launch joins the command line with spaces and runs it through the
        # shell (osrf_pycommon's async_execute_process, shell=True).
        cmd = ["/bin/sh", "-c", " ".join(cmd)]

    exec_name = _strip_counter(description.final_name)
    # The NAME the author gave, or None: an unnamed process is keyed by its
    # executable, as the Rust parser does.
    name = exec_name if description.name is not None else None

    env_vars = []
    if description.additional_env is not None:
        for key, value in description.additional_env:
            env_vars.append(
                (
                    "".join(context.perform_substitution(x) for x in key),
                    "".join(context.perform_substitution(x) for x in value),
                )
            )

    on_exit_shutdown = None
    if hasattr(process, "_ExecuteLocal__on_exit"):
        shutdown_handler, unsupported = _classify_on_exit(process._ExecuteLocal__on_exit)
        if shutdown_handler:
            on_exit_shutdown = True
        if unsupported:
            launch.logging.get_logger("dump_launch").warning(
                f"ExecuteProcess '{exec_name}' has on_exit handlers other than Shutdown; "
                "play_launch replays only on_exit=Shutdown and ignores the rest."
            )

    # Same rule as visit_node: only an explicit True is kept.
    respawn = None
    respawn_value = getattr(process, "_ExecuteLocal__respawn", None)
    if isinstance(respawn_value, bool):
        respawn = True if respawn_value else None
    elif respawn_value is not None:
        text = "".join(context.perform_substitution(x) for x in respawn_value)
        respawn = True if text.lower() in ("true", "1", "yes") else None
    respawn_delay = getattr(process, "_ExecuteLocal__respawn_delay", None) if respawn else None

    dump.node.append(
        NodeRecord(
            executable=cmd[0],
            package=None,
            name=name,
            namespace=None,
            exec_name=exec_name,
            params=[],
            params_files=[],
            remaps=[],
            ros_args=None,
            args=cmd[1:] or None,
            cmd=cmd,
            env=env_vars or None,
            respawn=respawn,
            respawn_delay=respawn_delay,
            on_exit_shutdown=on_exit_shutdown,
            scope=dump.current_scope_id,
        )
    )
    return ret
