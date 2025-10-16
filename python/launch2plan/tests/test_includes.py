"""
Tests for include handling (Phase 7).

These tests verify that launch file includes are correctly converted to plan includes,
with proper argument forwarding and cycle detection.
"""

from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch2plan.builder import PlanBuilder
from launch2plan.visitor import BranchExplorerSession, IncludeMetadata, visit_action


def test_include_detection():
    """Test that includes are detected and recorded."""
    # Create a simple include action
    # Note: PythonLaunchDescriptionSource requires an actual file
    # So we create a temporary one
    import tempfile

    with tempfile.NamedTemporaryFile(mode="w", suffix=".launch.py", delete=False) as f:
        f.write("from launch import LaunchDescription\n")
        f.write("def generate_launch_description():\n")
        f.write("    return LaunchDescription([])\n")
        include_path = Path(f.name)

    try:
        include = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(include_path)))

        # Create session and context
        session = BranchExplorerSession()
        context = LaunchContext()

        # Visit the include
        visit_action(include, context, session)

        # Verify include was recorded
        assert len(session.includes) == 1
        assert session.includes[0].file_path == include_path.resolve()
        assert session.includes[0].arguments == {}
        assert session.includes[0].condition_expr is None
    finally:
        # Clean up temp file
        include_path.unlink()


def test_include_with_arguments():
    """Test that include arguments are captured."""
    # Create include with arguments
    # Note: PythonLaunchDescriptionSource requires an actual file
    import tempfile

    with tempfile.NamedTemporaryFile(mode="w", suffix=".launch.py", delete=False) as f:
        f.write("from launch import LaunchDescription\n")
        f.write("def generate_launch_description():\n")
        f.write("    return LaunchDescription([])\n")
        include_path = Path(f.name)

    try:
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(include_path)),
            launch_arguments=[
                ("device", "/dev/video0"),
                ("fps", "30"),
            ],
        )

        # Create session and context
        session = BranchExplorerSession()
        context = LaunchContext()

        # Visit the include
        visit_action(include, context, session)

        # Verify include and arguments
        assert len(session.includes) == 1
        assert session.includes[0].file_path == include_path.resolve()
        assert session.includes[0].arguments == {
            "device": "/dev/video0",
            "fps": "30",
        }
    finally:
        # Clean up temp file
        include_path.unlink()


def test_include_with_launch_configuration_arguments():
    """Test that LaunchConfiguration arguments are substituted."""
    # Create include with LaunchConfiguration arguments
    include_path = Path("/tmp/sensor.launch.py")

    # Set up launch context with a variable
    context = LaunchContext()
    context.launch_configurations["sensor_device"] = "/dev/ttyUSB0"

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(include_path)),
        launch_arguments=[
            ("device", LaunchConfiguration("sensor_device")),
        ],
    )

    # Create session
    session = BranchExplorerSession()

    # Visit the include
    visit_action(include, context, session)

    # Verify substitution occurred
    assert len(session.includes) == 1
    assert session.includes[0].arguments == {
        "device": "/dev/ttyUSB0",
    }


def test_cycle_detection_simple():
    """Test that simple cycles are detected."""
    file_a = Path("/tmp/file_a.launch.py").resolve()
    session = BranchExplorerSession()

    # Add file_a to the stack
    session.include_stack.append(file_a)

    # Check that including file_a again would be a cycle
    assert session.check_cycle(file_a) is True


def test_cycle_detection_nested():
    """Test that nested cycles are detected."""
    file_a = Path("/tmp/file_a.launch.py").resolve()
    file_b = Path("/tmp/file_b.launch.py").resolve()
    file_c = Path("/tmp/file_c.launch.py").resolve()

    session = BranchExplorerSession()

    # Simulate include chain: A -> B -> C
    session.include_stack.append(file_a)
    session.include_stack.append(file_b)
    session.include_stack.append(file_c)

    # Check that including file_a or file_b would be a cycle
    assert session.check_cycle(file_a) is True
    assert session.check_cycle(file_b) is True

    # But including a new file would not be a cycle
    file_d = Path("/tmp/file_d.launch.py").resolve()
    assert session.check_cycle(file_d) is False


def test_include_section_generation():
    """Test that include section is generated correctly."""
    # Create include metadata
    includes = [
        IncludeMetadata(
            file_path=Path("/tmp/camera.launch.py"),
            arguments={"device": "/dev/video0", "fps": "30"},
            condition_expr=None,
        ),
        IncludeMetadata(
            file_path=Path("/tmp/sensor.launch.py"),
            arguments={"port": "/dev/ttyUSB0"},
            condition_expr="$(use_sensor)",
        ),
    ]

    # Build plan
    builder = PlanBuilder()
    plan = builder.build_plan(nodes=[], inferred_sockets={}, includes=includes)

    # Verify include section
    assert "include" in plan
    assert "camera" in plan["include"]
    assert "sensor" in plan["include"]

    # Verify camera include
    camera_include = plan["include"]["camera"]
    assert camera_include["file"] == "camera.plan.yaml"
    assert "arg" in camera_include
    assert camera_include["arg"]["device"] == "/dev/video0"
    # fps is inferred as integer
    assert camera_include["arg"]["fps"] == {"!i64": 30}
    assert "when" not in camera_include

    # Verify sensor include (with condition)
    sensor_include = plan["include"]["sensor"]
    assert sensor_include["file"] == "sensor.plan.yaml"
    assert sensor_include["when"] == "$(use_sensor)"


def test_include_with_duplicate_names():
    """Test that duplicate include names are handled."""
    # Create two includes with the same stem
    includes = [
        IncludeMetadata(
            file_path=Path("/tmp/dir1/camera.launch.py"),
            arguments={},
            condition_expr=None,
        ),
        IncludeMetadata(
            file_path=Path("/tmp/dir2/camera.launch.py"),
            arguments={},
            condition_expr=None,
        ),
    ]

    # Build plan
    builder = PlanBuilder()
    plan = builder.build_plan(nodes=[], inferred_sockets={}, includes=includes)

    # Verify both includes exist with unique names
    assert "include" in plan
    assert "camera" in plan["include"]
    assert "camera_1" in plan["include"]


def test_include_argument_type_inference():
    """Test that include arguments are properly typed."""
    # Create include with various argument types
    includes = [
        IncludeMetadata(
            file_path=Path("/tmp/test.launch.py"),
            arguments={
                "enable": "true",
                "count": "42",
                "rate": "10.5",
                "name": "robot",
            },
            condition_expr=None,
        ),
    ]

    # Build plan
    builder = PlanBuilder()
    plan = builder.build_plan(nodes=[], inferred_sockets={}, includes=includes)

    # Verify argument types
    args = plan["include"]["test"]["arg"]
    assert args["enable"] == {"!bool": True}
    assert args["count"] == {"!i64": 42}
    assert args["rate"] == {"!f64": 10.5}
    assert args["name"] == "robot"
