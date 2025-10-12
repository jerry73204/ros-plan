"""Tests for LaunchLoader."""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from launch2dump import LaunchLoader


def test_simple_launch_file():
    """Test loading a simple launch file with one node."""
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print("=== Launch Load Result ===")
    print(f"Nodes: {len(result.nodes)}")
    for node in result.nodes:
        print(f"  - {node.package}/{node.executable} (name={node.name})")

    print(f"Containers: {len(result.containers)}")
    print(f"Lifecycle nodes: {len(result.lifecycle_nodes)}")
    print(f"Errors: {len(result.errors)}")
    for error in result.errors:
        print(f"  - {error}")
    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Assertions
    assert len(result.nodes) == 1, f"Expected 1 node, got {len(result.nodes)}"
    assert result.nodes[0].package == "demo_nodes_cpp"
    assert result.nodes[0].executable == "talker"
    assert result.nodes[0].name == "talker"  # Default value
    assert len(result.errors) == 0, f"Expected no errors, got: {result.errors}"

    print("\n✅ Test passed!")


def test_launch_file_with_arguments():
    """Test loading a launch file with custom arguments."""
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    result = loader.load_launch_file(str(launch_file), launch_arguments={"node_name": "my_talker"})

    print("=== Launch Load Result (with arguments) ===")
    print(f"Nodes: {len(result.nodes)}")
    for node in result.nodes:
        print(f"  - {node.package}/{node.executable} (name={node.name})")

    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Assertions
    assert len(result.nodes) == 1
    assert result.nodes[0].name == "my_talker"  # Custom argument value
    assert "node_name" in result.parameter_dependencies

    print("\n✅ Test passed!")


if __name__ == "__main__":
    test_simple_launch_file()
    test_launch_file_with_arguments()
