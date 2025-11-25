"""Module for OnIncludeLaunchDescription class."""

from typing import Text

from launch.actions import OpaqueFunction
from launch.event_handler import EventHandler
from launch.events import IncludeLaunchDescription
from launch.utilities import is_a_subclass


class OnIncludeLaunchDescription(EventHandler):
    """Event handler used to handle asynchronous requests to include LaunchDescriptions."""

    def __init__(self, **kwargs):
        """Create an OnIncludeLaunchDescription event handler."""

        super().__init__(
            matcher=lambda event: is_a_subclass(event, IncludeLaunchDescription),
            entities=OpaqueFunction(
                function=lambda context: [context.locals.event.launch_description]
            ),
            **kwargs,
        )

    @property
    def handler_description(self) -> Text:
        """Return the string description of the handler."""
        return "returns the launch_description in the event"

    @property
    def matcher_description(self) -> Text:
        """Return the string description of the matcher."""
        return "event issubclass of launch.events.IncludeLaunchDescription"
