"""Comprehensive tests for LaunchLoader - Phase 6.2."""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from launch2dump import LaunchLoader


def test_multiple_nodes():
    """Test loading a launch file with multiple nodes."""
    print("\n=== Test: Multiple Nodes ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "multiple_nodes.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Nodes found: {len(result.nodes)}")
    for node in result.nodes:
        print(f"  - {node.package}/{node.executable} (name={node.name}, ns={node.namespace})")

    assert len(result.nodes) == 3, f"Expected 3 nodes, got {len(result.nodes)}"
    assert result.nodes[0].package == "demo_nodes_cpp"
    assert result.nodes[0].executable == "talker"
    assert result.nodes[0].name == "talker"

    assert result.nodes[1].package == "demo_nodes_cpp"
    assert result.nodes[1].executable == "listener"
    assert result.nodes[1].name == "listener"

    assert result.nodes[2].package == "demo_nodes_py"
    assert result.nodes[2].executable == "talker"
    assert result.nodes[2].name == "py_talker"
    assert result.nodes[2].namespace == "/python"

    assert len(result.errors) == 0, f"Expected no errors, got: {result.errors}"
    print("✅ Test passed!")


def test_parameters():
    """Test extracting node parameters."""
    print("\n=== Test: Parameters ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_parameters.launch.py"
    result = loader.load_launch_file(str(launch_file), launch_arguments={"publish_rate": "20"})

    print(f"Nodes found: {len(result.nodes)}")
    print(f"Parameters: {result.nodes[0].parameters}")
    print(f"Parameter dependencies: {result.parameter_dependencies}")

    assert len(result.nodes) == 1
    assert len(result.nodes[0].parameters) > 0, "Expected parameters to be extracted"

    # Check parameter dependencies
    assert "publish_rate" in result.parameter_dependencies, (
        f"Expected 'publish_rate' in dependencies, got: {result.parameter_dependencies}"
    )

    print("✅ Test passed!")


def test_remappings():
    """Test extracting topic remappings."""
    print("\n=== Test: Remappings ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_remappings.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Nodes found: {len(result.nodes)}")
    print(f"Remappings: {result.nodes[0].remappings}")

    assert len(result.nodes) == 1
    remappings = result.nodes[0].remappings
    assert len(remappings) == 2, f"Expected 2 remappings, got {len(remappings)}"

    # Check specific remappings
    remapping_dict = dict(remappings)
    assert "chatter" in remapping_dict or "/chatter" in remapping_dict
    assert "/rosout" in remapping_dict or "rosout" in remapping_dict

    print("✅ Test passed!")


def test_conditionals():
    """Test launch file with conditionals (GroupAction with conditions)."""
    print("\n=== Test: Conditionals ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_conditionals.launch.py"

    # Test with use_talker=true (default), use_listener=false (default)
    result1 = loader.load_launch_file(str(launch_file))
    print(f"Default args - Nodes found: {len(result1.nodes)}")
    for node in result1.nodes:
        print(f"  - {node.name}")

    # Should have talker but not listener
    assert len(result1.nodes) == 1, f"Expected 1 node with defaults, got {len(result1.nodes)}"
    assert result1.nodes[0].name == "talker"

    # Test with use_talker=false, use_listener=true
    result2 = loader.load_launch_file(
        str(launch_file), launch_arguments={"use_talker": "false", "use_listener": "true"}
    )
    print(f"Custom args - Nodes found: {len(result2.nodes)}")
    for node in result2.nodes:
        print(f"  - {node.name}")

    # Should have listener but not talker
    assert len(result2.nodes) == 1, f"Expected 1 node with custom args, got {len(result2.nodes)}"
    assert result2.nodes[0].name == "listener"

    # Test with both enabled
    result3 = loader.load_launch_file(
        str(launch_file), launch_arguments={"use_talker": "true", "use_listener": "true"}
    )
    print(f"Both enabled - Nodes found: {len(result3.nodes)}")
    assert len(result3.nodes) == 2, f"Expected 2 nodes, got {len(result3.nodes)}"

    # Check parameter dependencies
    assert "use_talker" in result1.parameter_dependencies
    assert "use_listener" in result1.parameter_dependencies

    print("✅ Test passed!")


def test_include_launch_description():
    """Test loading a launch file that includes another launch file."""
    print("\n=== Test: Include Launch Description ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_include.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Nodes found: {len(result.nodes)}")
    for node in result.nodes:
        print(f"  - {node.package}/{node.executable} (name={node.name})")

    # Should have both the included node and the main node
    assert len(result.nodes) == 2, (
        f"Expected 2 nodes (1 included + 1 main), got {len(result.nodes)}"
    )

    node_names = [node.name for node in result.nodes]
    assert "included_talker" in node_names, "Expected included_talker from base launch file"
    assert "main_listener" in node_names, "Expected main_listener from main launch file"

    assert len(result.errors) == 0, f"Expected no errors, got: {result.errors}"
    print("✅ Test passed!")


def test_composable_nodes():
    """Test loading a launch file with composable node container."""
    print("\n=== Test: Composable Node Container ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "composable_nodes.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Containers found: {len(result.containers)}")
    for container in result.containers:
        print(f"  - {container.package}/{container.executable} (name={container.name})")
        print(f"    Composable nodes: {len(container.composable_nodes)}")
        for comp_node in container.composable_nodes:
            print(f"      - {comp_node.plugin} (name={comp_node.name})")

    assert len(result.containers) == 1, f"Expected 1 container, got {len(result.containers)}"

    container = result.containers[0]
    assert container.package == "rclcpp_components"
    assert container.executable == "component_container"
    assert container.name == "my_container"

    assert len(container.composable_nodes) == 2, (
        f"Expected 2 composable nodes, got {len(container.composable_nodes)}"
    )

    # Check composable nodes
    assert container.composable_nodes[0].plugin == "composition::Talker"
    assert container.composable_nodes[0].name == "talker_component"

    assert container.composable_nodes[1].plugin == "composition::Listener"
    assert container.composable_nodes[1].name == "listener_component"

    # Check remappings on second composable node
    assert len(container.composable_nodes[1].remappings) > 0

    assert len(result.errors) == 0, f"Expected no errors, got: {result.errors}"
    print("✅ Test passed!")


def test_error_handling_missing_file():
    """Test error handling for missing launch file."""
    print("\n=== Test: Error Handling - Missing File ===")
    loader = LaunchLoader()

    launch_file = "/nonexistent/path/missing.launch.py"
    result = loader.load_launch_file(launch_file)

    print(f"Errors: {len(result.errors)}")
    for error in result.errors:
        print(f"  - {error}")

    assert len(result.errors) > 0, "Expected errors for missing file"
    assert len(result.nodes) == 0, "Expected no nodes for missing file"

    print("✅ Test passed!")


def test_no_processes_spawned():
    """Verify that no processes are spawned during loading."""
    print("\n=== Test: No Processes Spawned ===")
    try:
        import os

        import psutil

        loader = LaunchLoader()

        # Get initial process count
        parent = psutil.Process(os.getpid())
        initial_children = len(parent.children(recursive=True))

        # Load a launch file
        launch_file = Path(__file__).parent / "fixtures" / "multiple_nodes.launch.py"
        result = loader.load_launch_file(str(launch_file))

        # Get final process count
        final_children = len(parent.children(recursive=True))

        print(f"Initial child processes: {initial_children}")
        print(f"Final child processes: {final_children}")
        print(f"Nodes extracted: {len(result.nodes)}")

        # Should have extracted nodes but not spawned any processes
        assert len(result.nodes) > 0, "Should have extracted nodes"
        assert final_children == initial_children, (
            f"Processes were spawned: {final_children - initial_children} new child processes"
        )

        print("✅ Test passed!")
    except ImportError:
        print("⚠️  Skipped (psutil not installed)")


if __name__ == "__main__":
    print("=" * 60)
    print("Phase 6.2 Comprehensive Tests")
    print("=" * 60)

    try:
        test_multiple_nodes()
        test_parameters()
        test_remappings()
        test_conditionals()
        test_include_launch_description()
        test_composable_nodes()
        test_error_handling_missing_file()
        test_no_processes_spawned()

        print("\n" + "=" * 60)
        print("✅ ALL TESTS PASSED!")
        print("=" * 60)
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
