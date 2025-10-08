"""
Event handlers for launch inspection
"""

from launch.event_handler import EventHandler
from launch.events import IncludeLaunchDescription


class OnIncludeLaunchDescription(EventHandler):
    """Event handler for IncludeLaunchDescription events."""

    def __init__(self, **kwargs):
        """Create OnIncludeLaunchDescription event handler."""
        super().__init__(
            matcher=lambda event: isinstance(event, IncludeLaunchDescription), **kwargs
        )

    def handle(self, event: IncludeLaunchDescription, context):
        """
        Handle the event.

        :param event: The IncludeLaunchDescription event
        :param context: The launch context
        :return: List of entities to visit
        """
        return event.launch_description.entities
