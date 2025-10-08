"""
Launch inspector for metadata extraction without process execution
"""

import asyncio
import logging
import threading
from typing import Iterable, List, Optional, Text, Tuple

import launch.logging
from launch.event import Event
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity

from .event_handlers import OnIncludeLaunchDescription
from .visitor import CollectionSession, visit_entity


class LaunchInspector:
    """Inspector that visits launch entities without executing processes."""

    def __init__(
        self,
        *,
        argv: Optional[Iterable[Text]] = None,
        noninteractive: bool = False,
        debug: bool = False,
    ) -> None:
        """
        Create a LaunchInspector.

        :param argv: Arguments stored in the context, None results in []
        :param noninteractive: If True, assume no terminal interaction
        :param debug: If True, enable debug logging
        """
        # Setup logging
        launch.logging.launch_config.level = logging.DEBUG if debug else logging.INFO
        self.__debug = debug
        self.__argv = argv if argv is not None else []
        self.__logger = launch.logging.get_logger("launch2plan")

        # Setup context
        self.__context = LaunchContext(argv=self.__argv, noninteractive=noninteractive)
        self.__context.register_event_handler(OnIncludeLaunchDescription())
        self.__context.register_event_handler(OnProcessExit(on_exit=self.__on_process_exit))
        self.__context.register_event_handler(OnShutdown(on_shutdown=self.__on_shutdown))

        # Setup state storage
        self._entity_future_pairs: List[Tuple[LaunchDescriptionEntity, asyncio.Future]] = []

        self.__loop_from_run_thread_lock = threading.RLock()
        self.__loop_from_run_thread = None
        self.__shutting_down = False
        self.__shutdown_when_idle = False
        self.__return_code = 0

        # Session for collecting nodes
        self.__session = CollectionSession()

    def emit_event(self, event: Event) -> None:
        """
        Emit an event synchronously.

        :param event: The Event to emit
        """
        self.__context.emit_event_sync(event)

    def include_launch_description(self, launch_description: LaunchDescription) -> None:
        """
        Include a LaunchDescription.

        :param launch_description: LaunchDescription to include
        """
        self.__context._push_locals()
        for entity in launch_description.entities:
            pairs = visit_entity(entity, self.__context, self.__session)
            for entity, future in pairs:
                self._entity_future_pairs.append((entity, future))
        self.__context._pop_locals()

    def __on_process_exit(self, event, context) -> None:
        """Handle process exit events (no-op for inspection)."""
        pass

    def __on_shutdown(self, event: Event, context: LaunchContext) -> None:
        """Handle shutdown event."""
        self.__logger.debug("shutdown event received")
        self.__shutting_down = True

    def run(self, *, shutdown_when_idle: bool = True) -> int:
        """
        Run the inspector synchronously.

        :param shutdown_when_idle: If True, shutdown when idle
        :return: Return code (0 for success)
        """
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            return_code = loop.run_until_complete(
                self.run_async(shutdown_when_idle=shutdown_when_idle)
            )
        finally:
            loop.close()
        return return_code

    async def run_async(self, *, shutdown_when_idle: bool = True) -> int:
        """
        Visit all entities asynchronously.

        :param shutdown_when_idle: If True, shutdown when idle
        :return: Return code (0 for success)
        """
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("LaunchInspector can only be run in the main thread.")

        self.__shutdown_when_idle = shutdown_when_idle

        # Process all pending entity futures
        while self._entity_future_pairs:
            # Wait for all pending futures
            if self._entity_future_pairs:
                await asyncio.gather(
                    *[pair[1] for pair in self._entity_future_pairs], return_exceptions=True
                )
            self._entity_future_pairs.clear()

        # Emit shutdown if requested
        if shutdown_when_idle and not self.__shutting_down:
            self.emit_event(Shutdown(reason="launch inspector idle"))

        return self.__return_code

    @property
    def context(self) -> LaunchContext:
        """Get the launch context."""
        return self.__context

    @property
    def session(self) -> "CollectionSession":
        """Get the collection session."""
        return self.__session
