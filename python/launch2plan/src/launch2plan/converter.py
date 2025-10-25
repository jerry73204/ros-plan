"""
Launch file to plan conversion.

This module provides the main entry point for converting launch files to plans.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from .file_registry import FileRegistry
from .package_detector import detect_package
from .visitor import (
    BranchExplorerSession,
    IncludeMetadata,
    LaunchArgumentMetadata,
    NodeMetadata,
    visit_action,
)


@dataclass
class ConversionResult:
    """Result of converting a single launch file."""

    nodes: List[NodeMetadata]
    includes: List[IncludeMetadata]
    launch_arguments: List[LaunchArgumentMetadata]
    errors: List[str]


@dataclass
class MultiFileConversionResult:
    """Result of converting a launch file tree (Phase 9.3+)."""

    # Map from source file path -> ConversionResult
    file_results: Dict[Path, ConversionResult]
    # File registry tracking all generated files
    file_registry: FileRegistry
    # Root launch file
    root_file: Path
    # Errors across all files
    errors: List[str]


def convert_launch_file(
    launch_file: Path, launch_arguments: Optional[Dict[str, str]] = None
) -> ConversionResult:
    """
    Convert a single launch file to plan format.

    Phase 9.3+: This only processes a single file without recursing into includes.
    Use convert_launch_file_tree() for multi-file conversion.

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
    for entity in expanded_actions:
        # If it's a LaunchDescription, visit its entities
        if isinstance(entity, LaunchDescription):
            for action in entity.entities:
                try:
                    returned_entities = visit_action(action, context, session)
                    # Recursively visit any returned entities (e.g., from GroupAction)
                    if returned_entities:
                        for child in returned_entities:
                            if hasattr(child, "condition"):  # It's an Action
                                visit_action(child, context, session)
                except Exception as e:
                    error_msg = f"Error visiting action {type(action).__name__}: {e}"
                    session.errors.append(error_msg)
        # If it's an Action, visit it directly
        elif hasattr(entity, "condition"):
            try:
                visit_action(entity, context, session)
            except Exception as e:
                error_msg = f"Error visiting entity {type(entity).__name__}: {e}"
                session.errors.append(error_msg)
        # Otherwise skip (e.g., events)

    # Return the collected data
    return ConversionResult(
        nodes=session.nodes,
        includes=session.includes,
        launch_arguments=session.launch_arguments,
        errors=session.errors,
    )


def convert_launch_file_tree(
    root_launch_file: Path,
    output_dir: Path,
    launch_arguments: Optional[Dict[str, str]] = None,
) -> MultiFileConversionResult:
    """
    Convert a launch file tree to plan format (Phase 9.3+).

    Two-pass processing:
    1. Pass 1: Visit root file, collect nodes and include references
    2. Pass 2: Recursively process each unique include separately

    Phase 9.5: Implements file deduplication - each unique launch file
    generates exactly one plan file, regardless of how many times included
    or with what arguments.

    Args:
        root_launch_file: Path to the root launch file to convert
        output_dir: Output directory for plan files
        launch_arguments: Arguments to pass to root launch file (key-value pairs)

    Returns:
        MultiFileConversionResult with all processed files
    """
    if launch_arguments is None:
        launch_arguments = {}

    # Initialize file registry
    file_registry = FileRegistry(output_dir)

    # Track all files to process (queue)
    to_process: List[Tuple[Path, Dict[str, str]]] = [(root_launch_file, launch_arguments)]

    # Map from canonical path -> ConversionResult
    file_results: Dict[Path, ConversionResult] = {}

    # Track visited files to prevent infinite loops (Phase 9.5)
    visited = set()

    # Track arguments used for each file (Phase 9.5)
    # Map from canonical path -> list of argument dicts
    file_arguments: Dict[Path, List[Dict[str, str]]] = {}

    # Collect errors across all files
    all_errors = []

    # Process queue
    while to_process:
        current_file, current_args = to_process.pop(0)

        # Canonicalize path (Phase 9.5: deduplication key)
        canonical_path = current_file.resolve()

        # Phase 9.5: Track all argument sets for this file
        if canonical_path not in file_arguments:
            file_arguments[canonical_path] = []
        file_arguments[canonical_path].append(current_args)

        # Skip if already visited (Phase 9.5: deduplication)
        if canonical_path in visited:
            # Phase 9.5: Check if arguments differ and warn
            previous_args = file_arguments[canonical_path]
            if len(previous_args) > 1:
                # Check if all argument sets are identical
                first_args = previous_args[0]
                for args in previous_args[1:]:
                    if args != first_args:
                        # Warn about different arguments
                        warning = (
                            f"Warning: {canonical_path.name} is included multiple times "
                            f"with different arguments. Using first argument set: {first_args}"
                        )
                        all_errors.append(warning)
                        break  # Only warn once per file
            continue

        visited.add(canonical_path)

        # Convert this single file
        try:
            result = convert_launch_file(canonical_path, current_args)
            file_results[canonical_path] = result

            # Register this file in the registry
            pkg_info = detect_package(canonical_path)
            file_registry.register_file(canonical_path, package_info=pkg_info)

            # Collect errors
            all_errors.extend(result.errors)

            # Add includes to processing queue
            for include in result.includes:
                # Extract include path
                include_path = include.file_path

                # Add to queue (note: arguments are already resolved in the include metadata)
                to_process.append((include_path, include.arguments))

        except Exception as e:
            error_msg = f"Error converting {canonical_path}: {e}"
            all_errors.append(error_msg)

    return MultiFileConversionResult(
        file_results=file_results,
        file_registry=file_registry,
        root_file=root_launch_file.resolve(),
        errors=all_errors,
    )
