from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
    SetLaunchConfiguration,
    UnsetLaunchConfiguration,
)
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.events import ExecutionComplete  # noqa
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a, normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions.load_composable_nodes import LoadComposableNodes
from launch_ros.actions.node import Node

from ..launch_dump import LaunchDump
from .bare_execute_process import visit_bare_execute_process
from .composable_node_container import visit_composable_node_container
from .include_launch_description import visit_include_launch_description
from .lifecycle_node import visit_lifecycle_node
from .load_composable_nodes import visit_load_composable_nodes
from .node import visit_node


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

    else:
        return _execute_logging_configurations(action, context, dump)
