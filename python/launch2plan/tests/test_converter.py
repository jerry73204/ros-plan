"""
Tests for converter module (Phase 9.5 - File Deduplication).

These tests verify that:
1. Each unique launch file generates exactly one plan file
2. Multiple inclusions of the same file are deduplicated
3. Warnings are issued when same file is included with different arguments
4. Canonical path resolution handles symlinks correctly
"""

from launch2plan.converter import convert_launch_file_tree
from launch2plan.file_registry import FileRegistry


def test_simple_deduplication(tmp_path):
    """Test that same file included twice generates only one plan."""
    # Create a simple included launch file
    included_file = tmp_path / "sensor.launch.py"
    included_file.write_text("""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_pkg', executable='sensor_node', name='sensor')
    ])
""")

    # Create parent launch file that includes sensor.launch.py twice
    parent_file = tmp_path / "robot.launch.py"
    parent_file.write_text(f"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}')
        ),
    ])
""")

    # Convert the launch file tree
    output_dir = tmp_path / "output"
    result = convert_launch_file_tree(parent_file, output_dir)

    # Should have exactly 2 files: parent and included (not 3!)
    assert len(result.file_registry) == 2

    # Both parent and included should be in results
    canonical_parent = parent_file.resolve()
    canonical_included = included_file.resolve()

    assert canonical_parent in result.file_results
    assert canonical_included in result.file_results

    # Included file should only be processed once
    assert result.file_registry.is_registered(included_file)
    output_path = result.file_registry.get_output_path(included_file)
    assert output_path is not None


def test_deduplication_with_same_arguments(tmp_path):
    """Test that same file included twice with same arguments doesn't warn."""
    # Create included launch file that accepts arguments
    included_file = tmp_path / "sensor.launch.py"
    included_file.write_text("""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        Node(
            package='sensor_pkg',
            executable='sensor_node',
            name='sensor',
            parameters=[{'device': LaunchConfiguration('device')}]
        )
    ])
""")

    # Create parent that includes sensor twice with SAME arguments
    parent_file = tmp_path / "robot.launch.py"
    parent_file.write_text(f"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}'),
            launch_arguments={{'device': '/dev/video0'}}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}'),
            launch_arguments={{'device': '/dev/video0'}}.items()
        ),
    ])
""")

    # Convert
    output_dir = tmp_path / "output"
    result = convert_launch_file_tree(parent_file, output_dir)

    # Should have 2 files (parent + included)
    assert len(result.file_registry) == 2

    # Should NOT have any warnings about different arguments
    warnings = [e for e in result.errors if "different arguments" in e.lower()]
    assert len(warnings) == 0


def test_deduplication_with_different_arguments(tmp_path):
    """Test that same file included with different arguments issues warning."""
    # Create included launch file
    included_file = tmp_path / "sensor.launch.py"
    included_file.write_text("""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        Node(
            package='sensor_pkg',
            executable='sensor_node',
            name='sensor',
            parameters=[{'device': LaunchConfiguration('device')}]
        )
    ])
""")

    # Create parent that includes sensor twice with DIFFERENT arguments
    parent_file = tmp_path / "robot.launch.py"
    parent_file.write_text(f"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}'),
            launch_arguments={{'device': '/dev/video0'}}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}'),
            launch_arguments={{'device': '/dev/video1'}}.items()
        ),
    ])
""")

    # Convert
    output_dir = tmp_path / "output"
    result = convert_launch_file_tree(parent_file, output_dir)

    # Should still have 2 files (parent + included once)
    assert len(result.file_registry) == 2

    # Should have warning about different arguments
    warnings = [e for e in result.errors if "different arguments" in e.lower()]
    assert len(warnings) == 1
    assert "sensor.launch.py" in warnings[0]


def test_nested_includes_deduplication(tmp_path):
    """Test deduplication with nested includes."""
    # Create deepest file
    sensor_file = tmp_path / "sensor.launch.py"
    sensor_file.write_text("""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_pkg', executable='sensor_node', name='sensor')
    ])
""")

    # Create middle file that includes sensor
    middleware_file = tmp_path / "middleware.launch.py"
    middleware_file.write_text(f"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{sensor_file}')
        ),
    ])
""")

    # Create parent that includes both middleware and sensor directly
    parent_file = tmp_path / "robot.launch.py"
    parent_file.write_text(f"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{middleware_file}')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{sensor_file}')
        ),
    ])
""")

    # Convert
    output_dir = tmp_path / "output"
    result = convert_launch_file_tree(parent_file, output_dir)

    # Should have exactly 3 files (parent, middleware, sensor)
    # NOT 4 even though sensor is referenced twice
    assert len(result.file_registry) == 3

    # All three files should be registered
    assert result.file_registry.is_registered(parent_file)
    assert result.file_registry.is_registered(middleware_file)
    assert result.file_registry.is_registered(sensor_file)


def test_registry_canonical_path_tracking(tmp_path):
    """Test that FileRegistry uses canonical paths for deduplication."""
    registry = FileRegistry(tmp_path / "output")

    # Create a file
    source_file = tmp_path / "test.launch.py"
    source_file.write_text("# test")

    # Register with non-canonical path
    non_canonical = tmp_path / "." / "test.launch.py"
    output_path1 = registry.register_file(non_canonical)

    # Register again with different non-canonical path
    another_non_canonical = tmp_path / "." / ".." / tmp_path.name / "test.launch.py"
    output_path2 = registry.register_file(another_non_canonical)

    # Should return same output path (deduplicated)
    assert output_path1 == output_path2

    # Should only have one entry
    assert len(registry) == 1


def test_include_reference_points_to_single_file(tmp_path):
    """Test that multiple includes point to the same output file."""
    # Create included file
    included_file = tmp_path / "sensor.launch.py"
    included_file.write_text("""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_pkg', executable='sensor_node', name='sensor')
    ])
""")

    # Create parent with two includes
    parent_file = tmp_path / "robot.launch.py"
    parent_file.write_text(f"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('{included_file}')
        ),
    ])
""")

    # Convert
    output_dir = tmp_path / "output"
    result = convert_launch_file_tree(parent_file, output_dir)

    # Get parent's conversion result
    parent_result = result.file_results[parent_file.resolve()]

    # Should have 2 include entries (both pointing to same file)
    assert len(parent_result.includes) == 2

    # Both should reference the same canonical path
    assert parent_result.includes[0].file_path == parent_result.includes[1].file_path

    # The canonical path should match the included file
    assert parent_result.includes[0].file_path == included_file.resolve()
