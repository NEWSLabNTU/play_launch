"""Module for entity visitors."""

import asyncio
from typing import List, Tuple

from launch.action import Action
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a_subclass

from ..launch_dump import LaunchDump
from .action import visit_action


def visit_entity(
    entity: LaunchDescriptionEntity, context: LaunchContext, dump: LaunchDump
) -> List[Tuple[LaunchDescriptionEntity, asyncio.Future]]:
    """
    Visit given entity, as well as all sub-entities, and collect any futures.

    Sub-entities are visited recursively and depth-first.
    The future is collected from each entity (unless it returns None) before
    continuing on to more sub-entities.

    This function may call itself to traverse the sub-entities recursively.
    """

    if is_a_subclass(entity, Action):
        sub_entities = visit_action(entity, context, dump)
    else:
        sub_entities = entity.visit(context)

    entity_future = entity.get_asyncio_future()

    futures_to_return = []
    if entity_future is not None:
        futures_to_return.append((entity, entity_future))
    if sub_entities is not None:
        for sub_entity in sub_entities:
            futures_to_return += visit_entity(sub_entity, context, dump)
    return [
        future_pair for future_pair in futures_to_return if future_pair[1] is not None
    ]
