from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import type_utils
from launch_ros.actions.lifecycle_node import LifecycleNode

from ..launch_dump import LaunchDump
from .node import visit_node


def _resolved_autostart(node: LifecycleNode, context: LaunchContext) -> bool | None:
    """`autostart` as `LifecycleNode.execute` resolves it, or None.

    Jazzy added `autostart` (`node_autostart`, default False); Humble's
    LifecycleNode has no such attribute, so there it is None. Resolving it
    here is what `execute` does before deciding whether to append the
    configure + activate `LifecycleTransition` (play_launch phase 86).
    """
    if not hasattr(node, "node_autostart"):
        return None
    return bool(type_utils.perform_typed_substitution(context, node.node_autostart, bool))


def visit_lifecycle_node(
    node: LifecycleNode, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    """
    Execute the action.

    Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
    """
    node._perform_substitutions(context)  # ensure node.node_name is expanded
    if "<node_name_unspecified>" in node.node_name:
        raise RuntimeError("node_name unexpectedly incomplete for lifecycle node")

    # Record the lifecycle node name
    node_name = node._Node__expanded_node_name
    dump.lifecycle_node.append(node_name)

    # The record itself carries the marker and the resolved autostart, so the
    # model sees a lifecycle node per instance (phase 86). The transitions
    # stock `execute` would request are not run here: nothing is spawned.
    return visit_node(
        node,
        context,
        dump,
        lifecycle=True,
        lifecycle_autostart=_resolved_autostart(node, context),
    )
