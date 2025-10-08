"""
Lifecycle node visitor
"""

from typing import List, Optional

from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions.lifecycle_node import LifecycleNode

from .node import visit_node as visit_regular_node
from .session import CollectionSession


def visit_lifecycle_node(
    node: LifecycleNode, context: LaunchContext, session: CollectionSession
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit a LifecycleNode action.

    LifecycleNode is a subclass of Node, so we can reuse the regular node visitor
    but add it to the lifecycle_nodes list instead.

    :param node: The LifecycleNode action
    :param context: The launch context
    :param session: The collection session
    :return: None
    """
    # Visit as regular node to extract metadata
    result = visit_regular_node(node, context, session)

    # Move the last added node from nodes to lifecycle_nodes
    if session.nodes:
        last_node = session.nodes.pop()
        session.add_lifecycle_node(last_node)

    return result
