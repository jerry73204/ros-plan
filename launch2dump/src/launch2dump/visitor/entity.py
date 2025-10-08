"""
Entity visitor for traversing launch description entities
"""

import asyncio
from typing import List, Tuple

from launch.action import Action
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a_subclass

from .action import visit_action
from .session import CollectionSession


def visit_entity(
    entity: LaunchDescriptionEntity,
    context: LaunchContext,
    session: CollectionSession,
) -> List[Tuple[LaunchDescriptionEntity, asyncio.Future]]:
    """
    Visit an entity and collect metadata.

    Recursively visits sub-entities depth-first and collects futures.

    :param entity: The entity to visit
    :param context: The launch context
    :param session: The collection session
    :return: List of (entity, future) pairs
    """
    if is_a_subclass(entity, Action):
        sub_entities = visit_action(entity, context, session)
    else:
        sub_entities = entity.visit(context)

    entity_future = entity.get_asyncio_future()

    futures_to_return = []
    if entity_future is not None:
        futures_to_return.append((entity, entity_future))

    if sub_entities is not None:
        for sub_entity in sub_entities:
            futures_to_return += visit_entity(sub_entity, context, session)

    return [pair for pair in futures_to_return if pair[1] is not None]
