"""
Load composable nodes visitor
"""

from typing import List, Optional

from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions.load_composable_nodes import LoadComposableNodes

from .session import CollectionSession


def visit_load_composable_nodes(
    action: LoadComposableNodes,
    context: LaunchContext,
    session: CollectionSession,
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit a LoadComposableNodes action.

    This action loads composable nodes into an existing container.
    For metadata extraction, we'll track this but not spawn anything.

    :param action: The LoadComposableNodes action
    :param context: The launch context
    :param session: The collection session
    :return: None
    """
    # For now, we'll just note that composable nodes are being loaded
    # A more complete implementation would extract the composable node descriptions
    # and associate them with the target container

    # Don't spawn process
    return None
