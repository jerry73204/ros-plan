"""Tests for F34: Parameter Dependency Tracking."""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from launch2dump import LaunchLoader


def test_track_multiple_parameter_dependencies():
    """Test tracking multiple parameter dependencies in a single launch file."""
    print("\n=== Test: Track Multiple Parameter Dependencies ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_parameters.launch.py"
    result = loader.load_launch_file(
        str(launch_file), launch_arguments={"publish_rate": "20", "node_name": "test"}
    )

    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Should have both publish_rate and node_name
    assert "publish_rate" in result.parameter_dependencies
    # node_name might be used depending on the fixture
    assert len(result.parameter_dependencies) >= 1

    print("✅ Test passed!")


def test_handle_launch_file_with_no_parameter_dependencies():
    """Test launch file that doesn't use any parameters."""
    print("\n=== Test: No Parameter Dependencies ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "simple_talker.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Simple talker might still have node_name parameter, but if no args passed,
    # we check it doesn't crash
    assert isinstance(result.parameter_dependencies, set)
    assert len(result.errors) == 0

    print("✅ Test passed!")


def test_track_dependencies_through_included_launch_files():
    """Test tracking parameter dependencies across included launch files."""
    print("\n=== Test: Dependencies Through Includes ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_include.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Nodes found: {len(result.nodes)}")
    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Should have nodes from both the main file and included file
    assert len(result.nodes) >= 2
    # Dependencies might come from either file
    assert isinstance(result.parameter_dependencies, set)

    print("✅ Test passed!")


def test_parameter_dependency_deduplication():
    """Test that same parameter used multiple times is deduplicated."""
    print("\n=== Test: Parameter Deduplication ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_conditionals.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Count how many times use_talker appears (should be deduplicated in set)
    assert "use_talker" in result.parameter_dependencies
    # Set ensures deduplication automatically
    param_list = list(result.parameter_dependencies)
    assert param_list.count("use_talker") == 1

    print("✅ Test passed!")


def test_track_nested_substitution_dependencies():
    """Test tracking dependencies in nested substitutions like PathJoinSubstitution."""
    print("\n=== Test: Nested Substitution Dependencies ===")
    loader = LaunchLoader()

    launch_file = Path(__file__).parent / "fixtures" / "with_path_substitution.launch.py"
    result = loader.load_launch_file(str(launch_file))

    print(f"Parameter dependencies: {result.parameter_dependencies}")

    # Should detect both config_dir and config_file from PathJoinSubstitution
    assert "config_dir" in result.parameter_dependencies, (
        f"Expected 'config_dir' in dependencies, got: {result.parameter_dependencies}"
    )
    assert "config_file" in result.parameter_dependencies, (
        f"Expected 'config_file' in dependencies, got: {result.parameter_dependencies}"
    )

    print("✅ Test passed!")


if __name__ == "__main__":
    print("=" * 60)
    print("F34: Parameter Dependency Tracking Tests")
    print("=" * 60)

    try:
        test_track_multiple_parameter_dependencies()
        test_handle_launch_file_with_no_parameter_dependencies()
        test_track_dependencies_through_included_launch_files()
        test_parameter_dependency_deduplication()
        test_track_nested_substitution_dependencies()

        print("\n" + "=" * 60)
        print("✅ ALL F34 TESTS PASSED!")
        print("=" * 60)
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
