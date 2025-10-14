"""
Launch file to plan conversion.

This module provides the main entry point for converting launch files to plans.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List

from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource

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


def convert_launch_file(launch_file: Path) -> ConversionResult:
    """
    Convert a launch file to plan format.

    This is Phase 12.1 - basic visitor that discovers nodes and includes.
    Future phases will add introspection, plan generation, etc.

    Args:
        launch_file: Path to the launch file to convert

    Returns:
        ConversionResult with discovered nodes, includes, and errors
    """
    # Create launch service and context first
    launch_service = LaunchService()
    context = launch_service.context

    # Load the launch description with proper context
    launch_description_source = PythonLaunchDescriptionSource(str(launch_file))
    launch_description = launch_description_source.get_launch_description(context)

    # Create branch explorer session
    session = BranchExplorerSession()

    # Visit all actions in the launch description
    for action in launch_description.entities:
        visit_action(action, context, session)

    # Return the collected data
    return ConversionResult(nodes=session.nodes, includes=session.includes, errors=session.errors)
