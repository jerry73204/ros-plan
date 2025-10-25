"""
Reporting utilities for multi-file conversion (Phase 9.6).

This module provides conversion summary and file tree visualization.
"""

from pathlib import Path
from typing import Dict, List

from .converter import MultiFileConversionResult
from .file_registry import FileRegistry


def generate_conversion_summary(result: MultiFileConversionResult) -> str:
    """
    Generate a summary report for multi-file conversion.

    Args:
        result: Multi-file conversion result

    Returns:
        Formatted summary string
    """
    lines = []

    lines.append("=" * 70)
    lines.append("CONVERSION SUMMARY")
    lines.append("=" * 70)
    lines.append("")

    # File statistics
    total_files = len(result.file_registry)
    lines.append(f"Files Generated: {total_files}")
    lines.append("")

    # Count nodes, includes, and errors across all files
    total_nodes = 0
    total_includes = 0
    total_launch_args = 0

    for file_result in result.file_results.values():
        total_nodes += len(file_result.nodes)
        total_includes += len(file_result.includes)
        total_launch_args += len(file_result.launch_arguments)

    lines.append("Structure:")
    lines.append(f"  Nodes:           {total_nodes}")
    lines.append(f"  Includes:        {total_includes}")
    lines.append(f"  Launch Arguments: {total_launch_args}")
    lines.append("")

    # Error summary
    if result.errors:
        warnings = [e for e in result.errors if e.lower().startswith("warning")]
        errors = [e for e in result.errors if not e.lower().startswith("warning")]

        if errors:
            lines.append(f"Errors: {len(errors)}")
            for error in errors[:5]:  # Show first 5 errors
                lines.append(f"  - {error}")
            if len(errors) > 5:
                lines.append(f"  ... and {len(errors) - 5} more")
            lines.append("")

        if warnings:
            lines.append(f"Warnings: {len(warnings)}")
            for warning in warnings[:5]:  # Show first 5 warnings
                lines.append(f"  - {warning}")
            if len(warnings) > 5:
                lines.append(f"  ... and {len(warnings) - 5} more")
            lines.append("")

    # File mapping
    lines.append("Generated Files:")
    file_mapping = result.file_registry.get_all_files()

    # Sort by output path for readability
    sorted_files = sorted(file_mapping.items(), key=lambda x: str(x[1]))

    for source_path, output_path in sorted_files:
        # Get relative paths for cleaner display
        try:
            rel_source = source_path.relative_to(Path.cwd())
        except ValueError:
            rel_source = source_path

        try:
            rel_output = output_path.relative_to(result.file_registry.output_dir)
        except ValueError:
            rel_output = output_path

        lines.append(f"  {rel_output}")
        lines.append(f"    ← {rel_source}")

    lines.append("")
    lines.append("=" * 70)

    return "\n".join(lines)


def generate_file_tree(registry: FileRegistry, output_dir: Path) -> str:
    """
    Generate a file tree visualization of generated plan files.

    Args:
        registry: File registry with all generated files
        output_dir: Output directory base path

    Returns:
        Formatted file tree string
    """
    lines = []

    # Get all output paths relative to output_dir
    file_mapping = registry.get_all_files()
    output_paths = sorted(file_mapping.values())

    # Build tree structure
    tree_dict: Dict[str, List[str]] = {}

    for output_path in output_paths:
        try:
            rel_path = output_path.relative_to(output_dir)
        except ValueError:
            rel_path = output_path

        parts = list(rel_path.parts)

        # Add to tree
        current_level = tree_dict
        for i, part in enumerate(parts[:-1]):
            path_key = "/".join(parts[: i + 1])
            if path_key not in current_level:
                current_level[path_key] = []

        if len(parts) > 1:
            parent_key = "/".join(parts[:-1])
            if parent_key not in tree_dict:
                tree_dict[parent_key] = []
            tree_dict[parent_key].append(str(rel_path))
        else:
            if "" not in tree_dict:
                tree_dict[""] = []
            tree_dict[""].append(str(rel_path))

    # Render tree
    lines.append(f"{output_dir}/")

    # Simple tree rendering
    for output_path in output_paths:
        try:
            rel_path = output_path.relative_to(output_dir)
        except ValueError:
            rel_path = output_path

        depth = len(rel_path.parts) - 1
        indent = "  " * depth
        prefix = "├── " if depth > 0 else ""
        lines.append(f"{indent}{prefix}{rel_path.name}")

    return "\n".join(lines)


def print_conversion_summary(result: MultiFileConversionResult) -> None:
    """
    Print conversion summary to stdout.

    Args:
        result: Multi-file conversion result
    """
    print(generate_conversion_summary(result))
    print()
    print("File Tree:")
    print(generate_file_tree(result.file_registry, result.file_registry.output_dir))
    print()
