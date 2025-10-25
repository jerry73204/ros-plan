"""
Tests for reporting module (Phase 9.6).

These tests verify:
1. Conversion summary generation
2. File tree visualization
3. Statistics reporting
"""

from pathlib import Path

from launch2plan.converter import ConversionResult, MultiFileConversionResult
from launch2plan.file_registry import FileRegistry
from launch2plan.reporting import generate_conversion_summary, generate_file_tree


def test_generate_conversion_summary_single_file(tmp_path):
    """Test conversion summary with a single file."""
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    registry = FileRegistry(output_dir)
    source = tmp_path / "test.launch.py"
    source.touch()

    registry.register_file(source)

    result = MultiFileConversionResult(
        file_results={
            source.resolve(): ConversionResult(
                nodes=[],
                includes=[],
                launch_arguments=[],
                errors=[],
            )
        },
        file_registry=registry,
        root_file=source,
        errors=[],
    )

    summary = generate_conversion_summary(result)

    assert "CONVERSION SUMMARY" in summary
    assert "Files Generated: 1" in summary
    assert "test.plan.yaml" in summary


def test_generate_conversion_summary_with_errors(tmp_path):
    """Test conversion summary with errors and warnings."""
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    registry = FileRegistry(output_dir)
    source = tmp_path / "test.launch.py"
    source.touch()

    registry.register_file(source)

    errors = [
        "Error: Node not found",
        "Warning: Different arguments used",
        "Error: Invalid syntax",
    ]

    result = MultiFileConversionResult(
        file_results={
            source.resolve(): ConversionResult(
                nodes=[],
                includes=[],
                launch_arguments=[],
                errors=[],
            )
        },
        file_registry=registry,
        root_file=source,
        errors=errors,
    )

    summary = generate_conversion_summary(result)

    assert "Errors: 2" in summary
    assert "Warnings: 1" in summary
    assert "Different arguments" in summary


def test_generate_file_tree_simple(tmp_path):
    """Test file tree generation with simple structure."""
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    registry = FileRegistry(output_dir)

    # Register a few files
    source1 = tmp_path / "robot.launch.py"
    source2 = tmp_path / "sensor.launch.py"
    source1.touch()
    source2.touch()

    registry.register_file(source1)
    registry.register_file(source2)

    tree = generate_file_tree(registry, output_dir)

    assert str(output_dir) in tree
    assert "robot.plan.yaml" in tree
    assert "sensor.plan.yaml" in tree


def test_generate_file_tree_nested(tmp_path):
    """Test file tree with nested directories."""
    output_dir = tmp_path / "output"
    output_dir.mkdir()

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

    launch_dir = pkg_dir / "launch"
    launch_dir.mkdir()
    source = launch_dir / "robot.launch.py"
    source.touch()

    # Register with package detection
    from launch2plan.package_detector import detect_package

    pkg_info = detect_package(source)
    registry.register_file(source, package_info=pkg_info)

    tree = generate_file_tree(registry, output_dir)

    assert str(output_dir) in tree
    assert "robot.plan.yaml" in tree


def test_conversion_summary_statistics(tmp_path):
    """Test that summary includes node and include statistics."""
    from launch2plan.visitor import IncludeMetadata, NodeMetadata

    output_dir = tmp_path / "output"
    output_dir.mkdir()

    registry = FileRegistry(output_dir)
    source = tmp_path / "test.launch.py"
    source.touch()

    registry.register_file(source)

    # Create some nodes and includes
    nodes = [
        NodeMetadata(
            package="pkg1",
            executable="exe1",
            name="node1",
            namespace="/",
            remappings=[],
            parameters={},
            condition_expr=None,
        ),
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

    includes = [
        IncludeMetadata(
            file_path=Path("/tmp/include1.launch.py"),
            arguments={},
            condition_expr=None,
        ),
    ]

    result = MultiFileConversionResult(
        file_results={
            source.resolve(): ConversionResult(
                nodes=nodes,
                includes=includes,
                launch_arguments=[],
                errors=[],
            )
        },
        file_registry=registry,
        root_file=source,
        errors=[],
    )

    summary = generate_conversion_summary(result)

    assert "Nodes:           2" in summary
    assert "Includes:        1" in summary
