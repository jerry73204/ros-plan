"""
File writing utilities for multi-file plan generation (Phase 9.6).

This module handles writing plan files to disk with proper directory creation.
"""

from pathlib import Path
from typing import Dict

from .builder import PlanBuilder
from .converter import MultiFileConversionResult
from .inference import infer_sockets_for_nodes
from .introspection import IntrospectionService


def write_all_plans(
    result: MultiFileConversionResult,
    introspection_service: IntrospectionService,
) -> Dict[Path, Path]:
    """
    Write all plan files from a multi-file conversion result.

    Phase 9.6: Creates output directory structure and writes plan files.

    Args:
        result: Multi-file conversion result
        introspection_service: Introspection service for socket inference

    Returns:
        Dictionary mapping source path -> written output path
    """
    written_files: Dict[Path, Path] = {}
    builder = PlanBuilder()

    # Get file mapping from registry
    file_mapping = result.file_registry.get_all_files()

    for source_path, output_path in file_mapping.items():
        # Get conversion result for this file
        file_result = result.file_results.get(source_path)
        if not file_result:
            continue

        # Phase 9.6: Create output directory if needed
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Perform socket inference for this file's nodes
        inferred_sockets = infer_sockets_for_nodes(file_result.nodes, introspection_service)

        # Build plan for this file
        plan = builder.build_plan(
            nodes=file_result.nodes,
            inferred_sockets=inferred_sockets,
            launch_arguments=file_result.launch_arguments,
            includes=file_result.includes,
        )

        # Write plan file
        builder.write_plan(plan, output_path)

        written_files[source_path] = output_path

    return written_files


def ensure_output_directory(output_dir: Path) -> None:
    """
    Ensure output directory exists and is writable.

    Args:
        output_dir: Path to output directory

    Raises:
        OSError: If directory cannot be created or is not writable
    """
    # Check if path exists as a file (not a directory)
    if output_dir.exists() and not output_dir.is_dir():
        raise OSError(f"Output path exists but is not a directory: {output_dir}")

    # Create directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)

    # Test write permission by creating and removing a test file
    test_file = output_dir / ".write_test"
    try:
        test_file.touch()
        test_file.unlink()
    except OSError as e:
        raise OSError(f"Output directory is not writable: {output_dir}") from e
