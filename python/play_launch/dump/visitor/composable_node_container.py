from typing import List, Optional

from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions.composable_node_container import ComposableNodeContainer

from ..launch_dump import ComposableNodeContainerRecord, LaunchDump
from .node import visit_node


def visit_composable_node_container(
    container: ComposableNodeContainer, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Execute the action.

    Most work is delegated to :meth:`launch_ros.actions.Node.execute`, except for the
    composable nodes load action if it applies.
    """
    load_actions = None  # type: Optional[List[Action]]
    valid_composable_nodes = []

    descriptions = container._ComposableNodeContainer__composable_node_descriptions

    if descriptions:
        for node_object in descriptions:
            if node_object.condition() is None or node_object.condition().evaluate(
                context
            ):
                valid_composable_nodes.append(node_object)

    if valid_composable_nodes is not None and len(valid_composable_nodes) > 0:
        from .load_composable_nodes import LoadComposableNodes

        # Perform load action once the container has started.
        load_actions = [
            LoadComposableNodes(
                composable_node_descriptions=valid_composable_nodes,
                target_container=container,
            )
        ]
    container_actions = visit_node(container, context, dump)  # type: Optional[List[Action]]

    # Save a record in dump
    node_name = container._Node__expanded_node_name
    if container.expanded_node_namespace == container.UNSPECIFIED_NODE_NAMESPACE:
        namespace = None
    else:
        namespace = container.expanded_node_namespace

    record = ComposableNodeContainerRecord(name=node_name, namespace=namespace)
    dump.container.append(record)

    if container_actions is not None and load_actions is not None:
        return container_actions + load_actions
    if container_actions is not None:
        return container_actions
    if load_actions is not None:
        return load_actions
    return None
