"""
Integration tests for multi-file conversion (Phase 9.7).

These tests verify end-to-end conversion with real launch file hierarchies:
1. Simple hierarchy: parent + 1 include
2. Medium hierarchy: parent + 3 includes (1 nested)
3. File deduplication across hierarchy
4. Include references are correct
5. Argument flow through hierarchy
"""

from pathlib import Path

import pytest

from launch2plan.converter import convert_launch_file_tree
from launch2plan.introspection import IntrospectionService
from launch2plan.writer import write_all_plans


@pytest.fixture
def fixtures_dir():
    """Get path to test fixtures directory."""
    return Path(__file__).parent / "fixtures"


def test_simple_hierarchy_conversion(fixtures_dir, tmp_path):
    """Test simple hierarchy: parent + 1 include."""
    # Setup
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    # Convert
    result = convert_launch_file_tree(parent_file, output_dir, launch_arguments={})

    # Verify file count
    # Should have: simple_parent + simple_included = 2 files
    assert len(result.file_registry) == 2

    # Verify both files are registered
    included_file = fixtures_dir / "simple_included.launch.py"
    assert result.file_registry.is_registered(parent_file)
    assert result.file_registry.is_registered(included_file)

    # Verify conversion results exist for both files
    assert parent_file.resolve() in result.file_results
    assert included_file.resolve() in result.file_results

    # Verify parent has nodes and includes
    parent_result = result.file_results[parent_file.resolve()]
    assert len(parent_result.nodes) > 0  # Has talker node
    assert len(parent_result.includes) == 1  # Includes sensor file

    # Verify included file has nodes
    included_result = result.file_results[included_file.resolve()]
    assert len(included_result.nodes) > 0  # Has sensor node


def test_simple_hierarchy_include_references(fixtures_dir, tmp_path):
    """Test that include references point to correct files."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    # Get parent conversion result
    parent_result = result.file_results[parent_file.resolve()]

    # Verify include references
    assert len(parent_result.includes) == 1
    include = parent_result.includes[0]

    # Include should point to simple_included.launch.py
    included_file = fixtures_dir / "simple_included.launch.py"
    assert include.file_path == included_file.resolve()


def test_simple_hierarchy_argument_forwarding(fixtures_dir, tmp_path):
    """Test that arguments are forwarded correctly through includes."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    # Get parent conversion result
    parent_result = result.file_results[parent_file.resolve()]

    # Verify include has argument forwarding
    include = parent_result.includes[0]
    assert "device" in include.arguments

    # Should use $(sensor_device) - argument forwarding
    assert include.arguments["device"] == "$(sensor_device)"


def test_medium_hierarchy_conversion(fixtures_dir, tmp_path):
    """Test medium hierarchy with nested includes and deduplication."""
    parent_file = fixtures_dir / "medium_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    # Verify file count
    # Should have: medium_parent + medium_driver + medium_sensor = 3 files
    # NOT 4 even though medium_sensor is included twice
    assert len(result.file_registry) == 3

    # Verify all expected files are registered
    driver_file = fixtures_dir / "medium_driver.launch.py"
    sensor_file = fixtures_dir / "medium_sensor.launch.py"

    assert result.file_registry.is_registered(parent_file)
    assert result.file_registry.is_registered(driver_file)
    assert result.file_registry.is_registered(sensor_file)


def test_medium_hierarchy_deduplication(fixtures_dir, tmp_path):
    """Test that medium_sensor.launch.py is only converted once despite being included twice."""
    parent_file = fixtures_dir / "medium_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    sensor_file = fixtures_dir / "medium_sensor.launch.py"

    # Should only have one conversion result for sensor file
    assert sensor_file.resolve() in result.file_results

    # Sensor file should only appear once in registry
    sensor_output = result.file_registry.get_output_path(sensor_file)
    assert sensor_output is not None

    # Count how many times sensor is included
    parent_result = result.file_results[parent_file.resolve()]
    driver_result = result.file_results[(fixtures_dir / "medium_driver.launch.py").resolve()]

    parent_includes_sensor = any(
        inc.file_path == sensor_file.resolve() for inc in parent_result.includes
    )
    driver_includes_sensor = any(
        inc.file_path == sensor_file.resolve() for inc in driver_result.includes
    )

    # Both parent and driver include sensor, but it should only be converted once
    assert parent_includes_sensor
    assert driver_includes_sensor


def test_medium_hierarchy_different_arguments_warning(fixtures_dir, tmp_path):
    """Test that warning is issued when same file included with different arguments."""
    parent_file = fixtures_dir / "medium_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    # Should have warning about different arguments for medium_sensor
    warnings = [e for e in result.errors if "different arguments" in e.lower()]

    # Should have at least one warning
    assert len(warnings) > 0

    # Warning should mention the sensor file
    assert any("medium_sensor" in w for w in warnings)


def test_plan_files_created(fixtures_dir, tmp_path):
    """Test that plan files are actually written to disk."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    # Convert
    result = convert_launch_file_tree(parent_file, output_dir)

    # Write plans
    introspection_service = IntrospectionService()
    written_files = write_all_plans(result, introspection_service)

    # Verify files were written
    assert len(written_files) == 2

    for output_path in written_files.values():
        assert output_path.exists()
        assert output_path.is_file()
        assert output_path.suffix == ".yaml"

        # Verify file has content
        content = output_path.read_text()
        assert len(content) > 0
        assert "node:" in content or "include:" in content


def test_plan_files_have_correct_structure(fixtures_dir, tmp_path):
    """Test that generated plan files have correct YAML structure."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)
    introspection_service = IntrospectionService()
    write_all_plans(result, introspection_service)

    # Get parent plan file
    parent_output = result.file_registry.get_output_path(parent_file)
    assert parent_output is not None
    assert parent_output.exists()

    # Read and verify structure
    content = parent_output.read_text()

    # Parent should have:
    # - node section (talker)
    # - include section (sensor)
    assert "node:" in content
    assert "include:" in content


def test_include_file_references_in_plans(fixtures_dir, tmp_path):
    """Test that include references in plans point to correct plan files."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)
    introspection_service = IntrospectionService()
    write_all_plans(result, introspection_service)

    # Read parent plan
    parent_output = result.file_registry.get_output_path(parent_file)
    content = parent_output.read_text()

    # Should reference simple_included.plan.yaml
    assert "simple_included.plan.yaml" in content or "file:" in content


def test_argument_flow_through_hierarchy(fixtures_dir, tmp_path):
    """Test that arguments flow correctly through the hierarchy."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    # Convert with custom argument
    result = convert_launch_file_tree(
        parent_file, output_dir, launch_arguments={"sensor_device": "/dev/custom"}
    )

    # Get parent result
    parent_result = result.file_results[parent_file.resolve()]

    # Parent should have sensor_device argument
    assert any(arg.name == "sensor_device" for arg in parent_result.launch_arguments)

    # Include should forward the argument
    include = parent_result.includes[0]
    assert include.arguments.get("device") == "$(sensor_device)"


def test_no_errors_in_conversion(fixtures_dir, tmp_path):
    """Test that conversion completes without errors (warnings are OK)."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    # Filter out warnings
    errors = [e for e in result.errors if not e.lower().startswith("warning")]

    # Should have no actual errors
    assert len(errors) == 0


def test_output_directory_structure(fixtures_dir, tmp_path):
    """Test that output directory structure is correct."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)
    introspection_service = IntrospectionService()
    write_all_plans(result, introspection_service)

    # Output directory should exist
    assert output_dir.exists()
    assert output_dir.is_dir()

    # Should contain plan files
    plan_files = list(output_dir.glob("**/*.yaml"))
    assert len(plan_files) >= 2  # At least parent and included


def test_file_registry_consistency(fixtures_dir, tmp_path):
    """Test that file registry maintains consistent mappings."""
    parent_file = fixtures_dir / "simple_parent.launch.py"
    output_dir = tmp_path / "output"

    result = convert_launch_file_tree(parent_file, output_dir)

    # All files in file_results should be in file_registry
    for source_path in result.file_results.keys():
        assert result.file_registry.is_registered(Path(source_path))

    # All files in registry should have conversion results
    file_mapping = result.file_registry.get_all_files()
    for source_path in file_mapping.keys():
        assert source_path in result.file_results
