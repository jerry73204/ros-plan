"""
CLI entry point for launch2plan.
"""

import argparse
import sys
from pathlib import Path

from .converter import convert_launch_file


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
        "-o", "--output", help="Output plan file (default: <input>.plan.yaml)"
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
    else:
        # Other commands not yet implemented
        print(f"launch2plan: {args.command} command")
        print()
        print("STATUS: Not Yet Implemented")
        print()
        print("Implementation progress:")
        print("  [x] Phase 12.1: Foundation & Basic Visitor")
        print("  [ ] Phase 12.2: RMW Introspection Integration")
        print("  [ ] Phase 12.3: Socket Inference & TODO Generation")
        print("  [ ] Phase 12.4: Plan Builder & Link Generation")
        print("  [ ] Phase 12.5: Argument & Parameter Conversion")
        print("  [ ] Phase 12.6: Conditional Branch Exploration")
        print("  [ ] Phase 12.7: Include Handling & Plan Hierarchy")
        print("  [ ] Phase 12.8: Metadata Tracking & Pattern Learning")
        print("  [ ] Phase 12.9: Validation & Compilation")
        print("  [ ] Phase 12.10: End-to-End Testing & Examples")
        print()
        print(f"Requested command '{args.command}' is not yet implemented.")
        return 1


def handle_convert(args) -> int:
    """Handle the convert command."""
    launch_file = Path(args.launch_file)

    if not launch_file.exists():
        print(f"Error: Launch file not found: {launch_file}")
        return 1

    # Determine output file
    if args.output:
        output_file = Path(args.output)
    else:
        output_file = launch_file.with_suffix(".plan.yaml")

    print(f"Converting: {launch_file}")
    print(f"Output: {output_file}")
    print()

    try:
        result = convert_launch_file(launch_file)

        print(f"✓ Discovered {len(result.nodes)} nodes")
        print(f"✓ Discovered {len(result.includes)} includes")

        if result.errors:
            print(f"⚠ {len(result.errors)} errors encountered:")
            for error in result.errors:
                print(f"  - {error}")

        print()
        print("Nodes found:")
        for node in result.nodes:
            condition = f" (when: {node.condition_expr})" if node.condition_expr else ""
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
        print("Note: Phase 12.1 complete - basic visitor working!")
        print("Next phases will add:")
        print("  - RMW introspection for socket inference")
        print("  - Plan YAML generation with links")
        print("  - Argument and parameter conversion")

        return 0

    except Exception as e:
        print(f"Error during conversion: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
