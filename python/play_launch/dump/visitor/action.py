import types

import launch.logging
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
    RegisterEventHandler,
    SetLaunchConfiguration,
    UnsetLaunchConfiguration,
)
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import ExecutionComplete  # noqa
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a, normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions.load_composable_nodes import LoadComposableNodes
from launch_ros.actions.node import Node

from .. import exit_probe
from ..launch_dump import LaunchDump
from .bare_execute_process import visit_bare_execute_process
from .composable_node_container import visit_composable_node_container
from .include_launch_description import visit_include_launch_description
from .lifecycle_node import visit_lifecycle_node
from .load_composable_nodes import visit_load_composable_nodes
from .node import _classify_on_exit, visit_node


def visit_action(
    action: Action, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    condition = action.condition

    if condition is None or condition.evaluate(context):
        try:
            return visit_action_by_class(action, context, dump)
        finally:
            event = ExecutionComplete(action=action)
            if context.would_handle_event(event):
                future = action.get_asyncio_future()
                if future is not None:
                    future.add_done_callback(lambda _: context.emit_event_sync(event))
                else:
                    context.emit_event_sync(event)
    return None


try:  # absent from older `launch` releases
    from launch.actions import ResetLaunchConfigurations
except ImportError:  # pragma: no cover
    ResetLaunchConfigurations = None


def visit_action_by_class(
    action: Action, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    # Phase 94: a member records where in the configuration log it was
    # declared, for a contract's `$(var x @<node>)`.
    seq = len(dump.config_log)
    marks = (len(dump.node), len(dump.container), len(dump.load_node))
    try:
        return _visit_action_by_class(action, context, dump)
    finally:
        for records, mark in zip((dump.node, dump.container, dump.load_node), marks, strict=True):
            for record in records[mark:]:
                if record.config_seq is None:
                    record.config_seq = seq
        if is_a(action, ExecuteProcess):
            # Node, LifecycleNode and ComposableNodeContainer are ExecuteProcess.
            records = dump.node[marks[0] :] + dump.container[marks[1] :]
            _note_how_it_ends(action, context, records)
            dump.note_process(action, records)
        elif is_a(action, RegisterEventHandler):
            _note_exit_shutdown_handler(action.event_handler, dump, context)


def _on_process_start_entities(
    handler: OnProcessStart, context: LaunchContext
) -> list[LaunchDescriptionEntity] | None:
    """What an `OnProcessStart` handler runs as the launch comes up (#0101).

    A running launch executes `on_start` when its target starts, so the dump
    walks it in place, as the Rust parser does. A list is walked as it is; a
    callable is called as launch calls it, with a `ProcessStarted`-shaped
    event (no process yet, so no pid) and the context, and what it returns
    is walked. Nothing emits a start event during a dump, so without this
    both forms were dropped.
    """
    actions = getattr(handler, "_OnActionEventBase__actions_on_event", None)
    if actions:
        return list(actions)
    on_event = getattr(handler, "_OnActionEventBase__on_event", None)
    if not callable(on_event):
        return None
    target = getattr(handler, "_OnActionEventBase__action_matcher", None)
    target = target if isinstance(target, Action) else None
    event = types.SimpleNamespace(
        action=target,
        name=getattr(target, "name", None),
        pid=None,
        cmd=[],
        cwd=None,
        env=None,
    )
    try:
        returned = on_event(event, context)
    except Exception as e:  # noqa: BLE001 -- reported, as the Rust parser does
        launch.logging.get_logger("dump_launch").warning(
            "The on_start function of an OnProcessStart handler raised when called at "
            f"parse time ({e}); the actions it would return are not part of this launch."
        )
        return None
    if returned is None:
        return None
    if isinstance(returned, LaunchDescriptionEntity):
        return [returned]
    return list(returned)


def _note_how_it_ends(action, context: LaunchContext, records) -> None:
    """A process's shutdown timeouts (#0084) and its own `on_exit` function (#0090)."""
    if not records:
        return
    sigterm, sigkill = exit_probe.stop_timeouts(action, context)
    on = None
    on_exit = getattr(action, "_ExecuteLocal__on_exit", None)
    if callable(on_exit):
        on = exit_probe.probe(on_exit, action, context, f"'{records[0].exec_name}'")
    for record in records:
        record.sigterm_timeout = sigterm
        record.sigkill_timeout = sigkill
        if on is not None:
            exit_probe.mark(record, on)


def _note_exit_shutdown_handler(handler, dump: LaunchDump, context: LaunchContext) -> None:
    """`OnProcessExit(target_action=X, on_exit=<shutdown>)` makes X required (#0081).

    `ros2 launch` ends when X exits, exactly as for `X(on_exit=Shutdown())`.
    A callable `on_exit` is evaluated for every exit code (#0090, `exit_probe`);
    other exit actions are not modelled.
    """
    if not isinstance(handler, OnProcessExit):
        return
    actions = getattr(handler, "_OnActionEventBase__actions_on_event", None)
    target = getattr(handler, "_OnActionEventBase__action_matcher", None)
    on_event = getattr(handler, "_OnActionEventBase__on_event", None)
    if not actions and callable(on_event):
        # #0090: an `on_exit` function, evaluated for every exit code.
        targeted = isinstance(target, ExecuteProcess)
        what = "an OnProcessExit handler" + ("" if targeted else " with no target_action")
        on = exit_probe.probe(on_event, target if targeted else None, context, what)
        if on is None:
            return
        if targeted:
            dump.shutdown_on_exit_of(target, on)
            return
    if not actions and not callable(on_event):
        return
    shutdown, _others = _classify_on_exit(actions) if actions else (True, [])
    if not shutdown:
        return
    if isinstance(target, ExecuteProcess):
        dump.shutdown_on_exit_of(target)
    else:
        launch.logging.get_logger("dump_launch").warning(
            "An OnProcessExit handler without an ExecuteProcess/Node target_action shuts "
            "the launch down when any process exits; play_launch does not model that, so "
            "no exit ends this launch. Give the handler a target_action."
        )


def _execute_logging_configurations(
    action: Action, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    """Run an action launch runs itself, logging what it does to the launch
    configurations (phase 94): the same events the Rust parser records."""
    configs = context.launch_configurations
    if is_a(action, SetLaunchConfiguration):
        result = action.execute(context)
        name = perform_substitutions(context, action.name)
        origin, file = getattr(action, "_play_launch_origin", ("let", dump.current_file))
        dump.log_config("set", name, context.launch_configurations.get(name), origin, file)
        return result
    if is_a(action, DeclareLaunchArgument):
        had = action.name in configs
        result = action.execute(context)
        if not had and action.name in context.launch_configurations:
            dump.log_config(
                "set",
                action.name,
                context.launch_configurations[action.name],
                "arg",
                dump.current_file,
            )
        return result
    if is_a(action, UnsetLaunchConfiguration):
        result = action.execute(context)
        name = perform_substitutions(context, normalize_to_list_of_substitutions(action.name))
        dump.log_config("unset", name, file=dump.current_file)
        return result
    if is_a(action, PushLaunchConfigurations):
        dump.log_config("push", file=dump.current_file)
        return action.execute(context)
    if is_a(action, PopLaunchConfigurations):
        dump.log_config("pop", file=dump.current_file)
        return action.execute(context)
    if ResetLaunchConfigurations is not None and is_a(action, ResetLaunchConfigurations):
        result = action.execute(context)
        dump.log_config("reset", file=dump.current_file)
        for name, value in sorted(context.launch_configurations.items()):
            dump.log_config("set", name, value, "group", dump.current_file)
        return result
    result = action.execute(context)
    if is_a(action, GroupAction):
        # A group's own `launch_configurations=` are its sets; a
        # SetLaunchConfiguration among its actions is the file's.
        own = {id(a) for a in action.get_sub_entities()}
        for sub in result or []:
            if is_a(sub, SetLaunchConfiguration) and id(sub) not in own:
                sub._play_launch_origin = ("group", dump.current_file)
    return result


def _visit_action_by_class(
    action: Action, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    if is_a(action, LoadComposableNodes):
        return visit_load_composable_nodes(action, context, dump)

    elif is_a(action, ComposableNodeContainer):
        return visit_composable_node_container(action, context, dump)

    elif is_a(action, LifecycleNode):
        return visit_lifecycle_node(action, context, dump)

    elif is_a(action, Node):
        return visit_node(action, context, dump)

    elif is_a(action, IncludeLaunchDescription):
        return visit_include_launch_description(action, context, dump)

    # After the Node branches: Node subclasses ExecuteProcess. Without this
    # branch a plain ExecuteProcess fell through to stock `execute()` below,
    # which starts the process during the dump and records nothing (#0066).
    elif is_a(action, ExecuteProcess):
        return visit_bare_execute_process(action, context, dump)

    elif is_a(action, RegisterEventHandler) and isinstance(action.event_handler, OnProcessStart):
        return _on_process_start_entities(action.event_handler, context)

    else:
        return _execute_logging_configurations(action, context, dump)
