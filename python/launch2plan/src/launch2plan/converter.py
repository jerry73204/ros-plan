"""
Launch file to plan conversion.

This module provides the main entry point for converting launch files to plans.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from .visitor import (
    BranchExplorerSession,
    IncludeMetadata,
    NodeMetadata,
    visit_action,
)


@dataclass
class ConversionResult:
    """Result of converting a launch file."""

    nodes: List[NodeMetadata]
    includes: List[IncludeMetadata]
    errors: List[str]


def convert_launch_file(
    launch_file: Path, launch_arguments: Optional[Dict[str, str]] = None
) -> ConversionResult:
    """
    Convert a launch file to plan format.

    This is Phase 12.1 - basic visitor that discovers nodes and includes.
    Future phases will add introspection, plan generation, etc.

    Args:
        launch_file: Path to the launch file to convert
        launch_arguments: Arguments to pass to launch file (key-value pairs)

    Returns:
        ConversionResult with discovered nodes, includes, and errors
    """
    if launch_arguments is None:
        launch_arguments = {}

    # Convert arguments dict to list of (name, value) tuples
    launch_args = list(launch_arguments.items())

    # Prepare argv for launch arguments (needed for context)
    argv = [f"{name}:={value}" for name, value in launch_args]

    # Create launch service with arguments
    launch_service = LaunchService(argv=argv)
    context = launch_service.context

    # Create an IncludeLaunchDescription to properly handle launch_arguments
    include_action = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(launch_file)),
        launch_arguments=launch_args,
    )

    # Execute the include to get the expanded actions
    # This properly handles launch arguments
    expanded_actions = include_action.execute(context)

    # Create branch explorer session
    session = BranchExplorerSession()

    # Visit all expanded actions
    # These are the actions from the included launch file
    # Note: execute() can return Actions, Events, or LaunchDescriptions
    from launch import LaunchDescription

    import logging

    logger = logging.getLogger(__name__)
    logger.info(f"Processing {len(expanded_actions)} expanded actions")

    for entity in expanded_actions:
        # If it's a LaunchDescription, visit its entities
        if isinstance(entity, LaunchDescription):
            logger.info(f"Found LaunchDescription with {len(entity.entities)} entities")
            for action in entity.entities:
                try:
                    visit_action(action, context, session)
                except Exception as e:
                    session.errors.append(f"Error visiting action {type(action).__name__}: {e}")
                    logger.error(f"Error visiting {type(action).__name__}: {e}")
        # If it's an Action, visit it directly
        elif hasattr(entity, "condition"):
            logger.info(f"Found action: {type(entity).__name__}")
            try:
                visit_action(entity, context, session)
            except Exception as e:
                session.errors.append(f"Error visiting entity {type(entity).__name__}: {e}")
                logger.error(f"Error visiting {type(entity).__name__}: {e}")
        # Otherwise skip (e.g., events)

    # Return the collected data
    return ConversionResult(nodes=session.nodes, includes=session.includes, errors=session.errors)
