"""
Tests for writer module (Phase 9.6).

These tests verify:
1. Output directory creation
2. Multi-file plan writing
3. Directory structure preservation
"""

from launch2plan.converter import ConversionResult, MultiFileConversionResult
from launch2plan.file_registry import FileRegistry
from launch2plan.introspection import IntrospectionService
from launch2plan.visitor import NodeMetadata
from launch2plan.writer import ensure_output_directory, write_all_plans


def test_ensure_output_directory_creates_directory(tmp_path):
    """Test that output directory is created if it doesn't exist."""
    output_dir = tmp_path / "output" / "nested" / "dir"

    assert not output_dir.exists()

    ensure_output_directory(output_dir)

    assert output_dir.exists()
    assert output_dir.is_dir()


def test_ensure_output_directory_existing_directory(tmp_path):
    """Test that existing directory is accepted."""
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    # Should not raise
    ensure_output_directory(output_dir)

    assert output_dir.exists()


def test_ensure_output_directory_not_a_directory(tmp_path):
    """Test that error is raised if path is a file."""
    output_path = tmp_path / "output"
    output_path.touch()  # Create as file

    try:
        ensure_output_directory(output_path)
        assert False, "Should have raised OSError"
    except OSError as e:
        assert "not a directory" in str(e).lower()


def test_write_all_plans_creates_files(tmp_path):
    """Test that write_all_plans creates plan files."""
    output_dir = tmp_path / "output"
    registry = FileRegistry(output_dir)

    # Create source file
    source = tmp_path / "test.launch.py"
    source.write_text("""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker', name='talker')
    ])
""")

    # Register source
    registry.register_file(source)

    # Create conversion result
    nodes = [
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="talker",
            name="talker",
            namespace="/",
            remappings=[],
            parameters={},
            condition_expr=None,
        ),
    ]

    result = MultiFileConversionResult(
        file_results={
            source.resolve(): ConversionResult(
                nodes=nodes,
                includes=[],
                launch_arguments=[],
                errors=[],
            )
        },
        file_registry=registry,
        root_file=source,
        errors=[],
    )

    # Write plans
    introspection_service = IntrospectionService()
    written_files = write_all_plans(result, introspection_service)

    # Verify plan file was created
    assert len(written_files) == 1
    output_path = list(written_files.values())[0]
    assert output_path.exists()
    assert output_path.is_file()
    assert output_path.suffix == ".yaml"


def test_write_all_plans_preserves_directory_structure(tmp_path):
    """Test that nested directory structure is preserved."""
    output_dir = tmp_path / "output"
    registry = FileRegistry(output_dir)

    # Create package structure
    pkg_dir = tmp_path / "my_package"
    pkg_dir.mkdir()
    (pkg_dir / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>1.0.0</version>
  <description>Test</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

    # Create nested launch file
    launch_dir = pkg_dir / "launch" / "config"
    launch_dir.mkdir(parents=True)
    source = launch_dir / "robot.launch.py"
    source.write_text("""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='demo_nodes_cpp', executable='talker', name='talker')
    ])
""")

    # Register with package detection
    from launch2plan.package_detector import detect_package

    pkg_info = detect_package(source)
    registry.register_file(source, package_info=pkg_info)

    # Create conversion result
    nodes = [
        NodeMetadata(
            package="demo_nodes_cpp",
            executable="talker",
            name="talker",
            namespace="/",
            remappings=[],
            parameters={},
            condition_expr=None,
        ),
    ]

    result = MultiFileConversionResult(
        file_results={
            source.resolve(): ConversionResult(
                nodes=nodes,
                includes=[],
                launch_arguments=[],
                errors=[],
            )
        },
        file_registry=registry,
        root_file=source,
        errors=[],
    )

    # Write plans
    introspection_service = IntrospectionService()
    written_files = write_all_plans(result, introspection_service)

    # Verify directory structure
    output_path = list(written_files.values())[0]
    assert output_path.exists()

    # Should be under output_dir/my_package/launch/config/
    assert "my_package" in str(output_path)
    assert "launch" in str(output_path)
    assert "config" in str(output_path)


def test_write_all_plans_multiple_files(tmp_path):
    """Test writing multiple plan files."""
    output_dir = tmp_path / "output"
    registry = FileRegistry(output_dir)

    # Create multiple source files
    source1 = tmp_path / "robot.launch.py"
    source2 = tmp_path / "sensor.launch.py"
    source1.touch()
    source2.touch()

    registry.register_file(source1)
    registry.register_file(source2)

    # Create conversion results
    nodes1 = [
        NodeMetadata(
            package="pkg1",
            executable="exe1",
            name="node1",
            namespace="/",
            remappings=[],
            parameters={},
            condition_expr=None,
        ),
    ]

    nodes2 = [
        NodeMetadata(
            package="pkg2",
            executable="exe2",
            name="node2",
            namespace="/",
            remappings=[],
            parameters={},
            condition_expr=None,
        ),
    ]

    result = MultiFileConversionResult(
        file_results={
            source1.resolve(): ConversionResult(
                nodes=nodes1,
                includes=[],
                launch_arguments=[],
                errors=[],
            ),
            source2.resolve(): ConversionResult(
                nodes=nodes2,
                includes=[],
                launch_arguments=[],
                errors=[],
            ),
        },
        file_registry=registry,
        root_file=source1,
        errors=[],
    )

    # Write plans
    introspection_service = IntrospectionService()
    written_files = write_all_plans(result, introspection_service)

    # Verify both files were created
    assert len(written_files) == 2

    for output_path in written_files.values():
        assert output_path.exists()
        assert output_path.is_file()
