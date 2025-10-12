"""
Include launch description visitor
"""

from typing import List, Optional

from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

from .session import CollectionSession


def visit_include_launch_description(
    action: IncludeLaunchDescription,
    context: LaunchContext,
    session: CollectionSession,
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit an IncludeLaunchDescription action.

    This recursively includes another launch file's entities.

    :param action: The IncludeLaunchDescription action
    :param context: The launch context
    :param session: The collection session
    :return: List of entities from the included launch description
    """
    # Execute the action to get the included launch description
    # This will trigger the OnIncludeLaunchDescription event handler
    return action.execute(context)
