"""
CLI tool for loading ROS 2 launch files and dumping metadata.

Usage:
    launch2dump PACKAGE LAUNCH_FILE [key:=value...] [options]
    launch2dump LAUNCH_FILE [key:=value...] [options]

Examples:
    launch2dump autoware_launch planning_simulator.launch.xml map_path:=/path
    launch2dump my_robot.launch.py key:=value -f json
    launch2dump /path/to/file.launch.py -o output.yaml
"""

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .loader import LaunchLoader
from .serialization import serialize_launch_result


def resolve_package_launch_file(package: str, launch_file: str) -> Optional[Path]:
    """
    Resolve launch file path from package name.

    :param package: ROS 2 package name
    :param launch_file: Launch file name
    :return: Absolute path to launch file, or None if not found
    """
    try:
        # Get package prefix using ros2 pkg prefix
        result = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            capture_output=True,
            text=True,
            check=True,
        )
        pkg_prefix = Path(result.stdout.strip())

        # Look in share/PACKAGE/launch directory
        launch_dir = pkg_prefix / "share" / package / "launch"
        launch_path = launch_dir / launch_file

        if launch_path.exists():
            return launch_path

        # Also check directly in share/PACKAGE
        launch_path = pkg_prefix / "share" / package / launch_file
        if launch_path.exists():
            return launch_path

        return None
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def parse_arguments(args: List[str]) -> Tuple[argparse.Namespace, List[str]]:
    """
    Parse command line arguments in ros2 launch style.

    Returns tuple of (parsed_args, positional_launch_args)
    """
    parser = argparse.ArgumentParser(
        description="Load ROS 2 launch files and dump metadata without spawning processes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s autoware_launch planning_simulator.launch.xml map_path:=/path
  %(prog)s my_robot.launch.py key:=value -f json
  %(prog)s /path/to/file.launch.py -o output.yaml
        """,
    )

    parser.add_argument(
        "package_or_file",
        type=str,
        help="ROS 2 package name or path to launch file",
    )

    parser.add_argument(
        "launch_file_or_args",
        type=str,
        nargs="?",
        help="Launch file name (if first arg is package) or launch argument",
    )

    parser.add_argument(
        "-f",
        "--format",
        type=str,
        choices=["json", "yaml"],
        default="yaml",
        help="Output format (default: yaml)",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=str,
        help="Output file path (default: print to stdout)",
    )

    # Parse known args to separate flags from positional launch arguments
    parsed, remaining = parser.parse_known_args(args)

    return parsed, remaining


def parse_launch_arguments(arg_list: List[str]) -> Dict[str, str]:
    """
    Parse launch arguments from list of 'key:=value' strings.

    :param arg_list: List of argument strings
    :return: Dictionary of launch arguments
    """
    arguments = {}
    if arg_list is None:
        return arguments

    for arg in arg_list:
        if ":=" not in arg:
            print(f"Error: Invalid argument format '{arg}'. Expected 'key:=value'", file=sys.stderr)
            sys.exit(1)

        key, value = arg.split(":=", 1)
        arguments[key] = value

    return arguments


def main(argv: List[str] = None) -> int:
    """
    Main CLI entry point.

    :param argv: Command line arguments (defaults to sys.argv[1:])
    :return: Exit code (0 for success, 1 for error)
    """
    if argv is None:
        argv = sys.argv[1:]

    args, remaining_args = parse_arguments(argv)

    # Determine if first arg is package or file
    package_or_file = args.package_or_file
    launch_file_path = None

    # Check if it's a file path
    if Path(package_or_file).exists():
        # It's a file path
        launch_file_path = Path(package_or_file)
        # All remaining args + launch_file_or_args are launch arguments
        positional_launch_args = []
        if args.launch_file_or_args:
            positional_launch_args.append(args.launch_file_or_args)
        positional_launch_args.extend(remaining_args)
    else:
        # Assume it's a package name
        if not args.launch_file_or_args:
            print(
                f"Error: If '{package_or_file}' is a package, launch file name is required",
                file=sys.stderr,
            )
            return 1

        # Try to resolve package + launch file
        resolved = resolve_package_launch_file(package_or_file, args.launch_file_or_args)
        if resolved:
            launch_file_path = resolved
            positional_launch_args = remaining_args
        else:
            # Maybe first arg is actually a non-existent file?
            print(
                f"Error: Could not resolve package '{package_or_file}' "
                f"or find file '{package_or_file}'",
                file=sys.stderr,
            )
            return 1

    # Validate launch file exists
    if not launch_file_path.exists():
        print(f"Error: Launch file not found: {launch_file_path}", file=sys.stderr)
        return 1

    if not launch_file_path.is_file():
        print(f"Error: Not a file: {launch_file_path}", file=sys.stderr)
        return 1

    # Parse launch arguments from positional args (ros2 launch style)
    launch_arguments = {}
    if positional_launch_args:
        launch_arguments = parse_launch_arguments(positional_launch_args)

    # Load launch file
    try:
        loader = LaunchLoader()
        result = loader.load_launch_file(str(launch_file_path), launch_arguments=launch_arguments)
    except Exception as e:
        print(f"Error loading launch file: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1

    # Check for errors
    if result.errors:
        print("Warnings/Errors encountered during loading:", file=sys.stderr)
        for error in result.errors:
            print(f"  - {error}", file=sys.stderr)
        print("", file=sys.stderr)

    # Serialize result
    try:
        output = serialize_launch_result(result, format=args.format)
    except Exception as e:
        print(f"Error serializing result: {e}", file=sys.stderr)
        return 1

    # Write output
    if args.output:
        output_path = Path(args.output)
        try:
            output_path.write_text(output)
            print(f"Output written to: {output_path}", file=sys.stderr)
        except Exception as e:
            print(f"Error writing output file: {e}", file=sys.stderr)
            return 1
    else:
        print(output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
