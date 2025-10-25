"""
CLI entry point for launch2plan.
"""

import argparse
import hashlib
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List

from .builder import PlanBuilder
from .converter import convert_launch_file, convert_launch_file_tree
from .inference import infer_sockets_for_nodes
from .introspection import IntrospectionService
from .metadata import (
    METADATA_VERSION,
    ConversionMetadata,
    ConversionStats,
    MetadataManager,
    NodeSource,
    TodoStatusUpdater,
)
from .progress import get_progress_tracker
from .reporting import print_conversion_summary
from .statistics import calculate_stats
from .writer import ensure_output_directory, write_all_plans


def parse_launch_arguments(arg_list: List[str]) -> Dict[str, str]:
    """
    Parse launch arguments from list of 'key:=value' strings.

    Args:
        arg_list: List of argument strings

    Returns:
        Dictionary of launch arguments
    """
    arguments = {}
    if not arg_list:
        return arguments

    for arg in arg_list:
        if ":=" not in arg:
            print(f"Error: Invalid argument format '{arg}'. Expected 'key:=value'")
            sys.exit(1)

        key, value = arg.split(":=", 1)
        arguments[key] = value

    return arguments


def handle_multi_file_convert(
    launch_file: Path,
    launch_arguments: Dict[str, str],
    output_dir_str: str,
) -> int:
    """
    Handle multi-file conversion (Phase 9.6).

    Args:
        launch_file: Root launch file to convert
        launch_arguments: Launch arguments
        output_dir_str: Output directory path

    Returns:
        Exit code (0 for success, 1 for error)
    """
    output_dir = Path(output_dir_str)
    progress = get_progress_tracker()

    print(f"Converting (multi-file): {launch_file}")
    if launch_arguments:
        print(f"Arguments: {launch_arguments}")
    print(f"Output directory: {output_dir}")
    print()

    try:
        # Phase 9.6: Ensure output directory exists
        ensure_output_directory(output_dir)

        # Convert launch file tree
        print("Phase 1: Discovering files and nodes...")
        progress.start_phase("Discovering launch files")
        result = convert_launch_file_tree(launch_file, output_dir, launch_arguments)
        progress.end_phase(f"✓ Discovered {len(result.file_registry)} launch files")
        print()

        # Phase 2: Introspection and plan generation
        total_nodes = sum(len(fr.nodes) for fr in result.file_results.values())
        print(f"Phase 2: Introspecting {total_nodes} nodes and generating plans...")
        progress.start_phase("Introspecting nodes", total=total_nodes)
        introspection_service = IntrospectionService()
        written_files = write_all_plans(result, introspection_service)
        progress.end_phase(f"✓ Generated {len(written_files)} plan files")
        print()

        # Phase 3: Display summary (Phase 9.6)
        print_conversion_summary(result)

        # Success message
        print()
        print("✓ Multi-file conversion complete!")
        print()
        print("To compile the root plan:")
        root_output = result.file_registry.get_output_path(result.root_file)
        if root_output:
            print(f"  ros2plan compile {root_output}")

        return 0

    except OSError as e:
        print(f"Error: {e}")
        return 1
    except Exception as e:
        print(f"Error during conversion: {e}")
        import traceback

        traceback.print_exc()
        return 1


def main() -> int:
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="launch2plan",
        description="Convert ROS 2 launch files to ROS-Plan format",
    )

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # convert command
    convert_parser = subparsers.add_parser("convert", help="Convert launch file to plan")
    convert_parser.add_argument("launch_file", help="Path to launch file")
    convert_parser.add_argument(
        "launch_args",
        nargs="*",
        help="Launch arguments in key:=value format",
    )
    convert_parser.add_argument(
        "-o", "--output", help="Output plan file (default: <input>.plan.yaml)"
    )
    convert_parser.add_argument(
        "--output-dir",
        help="Output directory for multi-file conversion (Phase 9.6)",
    )

    # validate command
    validate_parser = subparsers.add_parser("validate", help="Validate plan file")
    validate_parser.add_argument("plan_file", help="Path to plan file")

    # status command
    status_parser = subparsers.add_parser("status", help="Show TODO completion status")
    status_parser.add_argument("plan_file", help="Path to plan file")

    # refine command
    refine_parser = subparsers.add_parser("refine", help="Apply pattern learning")
    refine_parser.add_argument("plan_file", help="Path to plan file")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    if args.command == "convert":
        return handle_convert(args)
    elif args.command == "status":
        return handle_status(args)
    else:
        # Other commands not yet implemented
        print(f"launch2plan: {args.command} command")
        print()
        print("STATUS: Not Yet Implemented")
        print()
        print(f"Requested command '{args.command}' is not yet implemented.")
        return 1


def handle_convert(args) -> int:
    """Handle the convert command (with Phase 8 metadata generation)."""
    launch_file = Path(args.launch_file)

    if not launch_file.exists():
        print(f"Error: Launch file not found: {launch_file}")
        return 1

    # Parse launch arguments
    launch_arguments = parse_launch_arguments(args.launch_args)

    # Phase 9.6: Check if multi-file conversion is requested
    if args.output_dir:
        return handle_multi_file_convert(launch_file, launch_arguments, args.output_dir)

    # Single-file conversion (original behavior)
    # Determine output file
    if args.output:
        output_file = Path(args.output)
    else:
        output_file = launch_file.with_suffix(".plan.yaml")

    print(f"Converting: {launch_file}")
    if launch_arguments:
        print(f"Arguments: {launch_arguments}")
    print(f"Output: {output_file}")
    print()

    try:
        # Phase 1: Discover nodes and includes
        result = convert_launch_file(launch_file, launch_arguments=launch_arguments)

        print(f"✓ Discovered {len(result.nodes)} nodes")
        print(f"✓ Discovered {len(result.includes)} includes")

        if result.errors:
            print(f"⚠ {len(result.errors)} errors encountered:")
            for error in result.errors:
                print(f"  - {error}")

        # Show discovered nodes
        if result.nodes:
            print()
            print("Nodes found:")
            for node in result.nodes:
                condition = f" (when: {node.condition_expr})" if node.condition_expr else ""
                # Show package::executable for clarity
                print(f"  - {node.package}::{node.executable}{condition}")
                if node.remappings:
                    for from_topic, to_topic in node.remappings:
                        print(f"    remap: {from_topic} -> {to_topic}")

        if result.includes:
            print()
            print("Includes found:")
            for inc in result.includes:
                condition = f" (when: {inc.condition_expr})" if inc.condition_expr else ""
                print(f"  - {inc.file_path}{condition}")

        print()

        # Phase 2: Introspection and socket inference
        introspection_service = IntrospectionService()
        inferred_sockets = infer_sockets_for_nodes(result.nodes, introspection_service)

        # Phase 3: Build plan
        builder = PlanBuilder()
        plan = builder.build_plan(
            nodes=result.nodes,
            inferred_sockets=inferred_sockets,
            launch_arguments=None,  # TODO: Extract launch arguments from visitor
            includes=result.includes,
        )

        # Phase 4: Write plan YAML
        builder.write_plan(plan, output_file)
        print(f"✓ Generated plan: {output_file}")

        # Phase 5: Create metadata (F67)
        source_hash = hashlib.sha256(launch_file.read_bytes()).hexdigest()

        # Build node_sources mapping
        node_sources = {}
        for node in result.nodes:
            node_id = node.name if node.name else f"{node.package}::{node.executable}"
            node_sources[node_id] = NodeSource(
                launch_file=str(launch_file),
                condition=node.condition_expr,
            )

        # Create metadata with TODOs collected during build
        metadata = ConversionMetadata(
            source_file=str(launch_file),
            source_hash=source_hash,
            generated_at=datetime.now().isoformat(),
            converter_version=METADATA_VERSION,
            todos=builder.get_todos(),
            stats=ConversionStats(),  # Will be calculated below
            node_sources=node_sources,
        )

        # Calculate statistics
        metadata.stats = calculate_stats(metadata, plan)

        # Save metadata
        manager = MetadataManager()
        manager.save_metadata(metadata, output_file)

        # Determine metadata path for display (same logic as MetadataManager)
        if output_file.suffix == ".yaml" and output_file.stem.endswith(".plan"):
            meta_path = output_file.with_suffix(".meta.json")
        else:
            meta_path = output_file.with_suffix(".plan.meta.json")
        print(f"✓ Generated metadata: {meta_path}")
        print()

        # Display TODO summary
        if metadata.todos:
            print("TODO Status:")
            print(f"  Total TODOs: {len(metadata.todos)}")
            print(f"  Run 'launch2plan status {output_file}' for details")
        else:
            print("✓ No TODOs - conversion complete!")

        print()
        print(f"Run 'ros2plan compile {output_file}' to compile the plan.")

        return 0

    except Exception as e:
        print(f"Error during conversion: {e}")
        import traceback

        traceback.print_exc()
        return 1


def handle_status(args) -> int:
    """Handle the status command (F72)."""
    plan_file = Path(args.plan_file)

    if not plan_file.exists():
        print(f"Error: Plan file not found: {plan_file}")
        return 1

    # Load metadata
    manager = MetadataManager()
    metadata = manager.load_metadata(plan_file)

    if not metadata:
        print(f"Error: No metadata found for {plan_file}")
        print(f"Expected: {plan_file.with_suffix('.plan.meta.json')}")
        print()
        print("Metadata is generated when you run 'launch2plan convert'.")
        return 1

    # Check if source file has changed
    source_path = Path(metadata.source_file)
    if source_path.exists():
        updater = TodoStatusUpdater()
        staleness_warning = updater.check_metadata_staleness(metadata, source_path)
        if staleness_warning:
            print(staleness_warning)
            print()

    # Update metadata with current plan state
    updater = TodoStatusUpdater()
    metadata = updater.update_metadata_from_plan(metadata, plan_file)

    # Save updated metadata
    manager.save_metadata(metadata, plan_file)

    # Display status
    stats = metadata.stats

    print(f"Conversion Status: {plan_file}")
    print(f"Source: {metadata.source_file}")
    print(f"Generated: {metadata.generated_at}")
    print()

    print("Structure:")
    print(f"  Nodes:     {stats.total_nodes}")
    print(f"  Includes:  {stats.total_includes}")
    print(f"  Links:     {stats.total_links}")
    print(f"  Arguments: {stats.total_arguments}")
    print()

    print("TODO Status:")
    print(f"  Total:     {stats.total_todos}")
    print(f"  Pending:   {stats.pending_todos}")
    print(f"  Completed: {stats.completed_todos}")
    print(f"  Progress:  {stats.completion_rate:.1%}")
    print()

    print("Introspection:")
    print(f"  Nodes introspected:         {stats.nodes_introspected}")
    print(f"  Nodes failed introspection: {stats.nodes_failed_introspection}")
    print(f"  Sockets from introspection: {stats.sockets_from_introspection}")
    print(f"  Sockets requiring input:    {stats.sockets_requiring_user_input}")
    print()

    if stats.pending_todos > 0:
        print("Pending TODOs:")
        for todo in metadata.todos:
            if todo.status == "pending":
                print(f"  - {todo.location}")
                print(f"    Field: {todo.field}")
                print(f"    Value: {todo.current_value}")
                if todo.context.hint:
                    print(f"    Hint: {todo.context.hint}")
        print()
        print(f"Run 'launch2plan validate {plan_file}' to check if plan compiles.")
    else:
        print("✓ All TODOs completed!")
        print(f"Run 'ros2plan compile {plan_file}' to compile the plan.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
