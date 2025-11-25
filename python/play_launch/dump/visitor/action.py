from typing import List, Optional

from launch.action import Action
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.events import ExecutionComplete  # noqa
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions.load_composable_nodes import LoadComposableNodes
from launch_ros.actions.node import Node

from ..launch_dump import LaunchDump
from .composable_node_container import visit_composable_node_container
from .include_launch_description import visit_include_launch_description
from .lifecycle_node import visit_lifecycle_node
from .load_composable_nodes import visit_load_composable_nodes
from .node import visit_node


def visit_action(
    action: Action, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
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


def visit_action_by_class(
    action: Action, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
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

    else:
        return action.execute(context)
