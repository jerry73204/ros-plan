"""Tests for metadata tracking (Phase 8)."""

import json
import tempfile
from datetime import datetime
from pathlib import Path

import pytest

from launch2plan.metadata import (
    ConversionMetadata,
    ConversionStats,
    MetadataManager,
    NodeSource,
    PlanParser,
    TodoContext,
    TodoItem,
    TodoReason,
    TodoStatus,
    TodoStatusUpdater,
)

# ============================================================================
# F66: Data Structure Tests
# ============================================================================


def test_dataclass_serialization():
    """Test that dataclasses can be serialized to/from JSON."""
    context = TodoContext(
        node_package="test_pkg",
        node_executable="test_exe",
        remapping=("image", "/camera/image"),
        reason=TodoReason.INTROSPECTION_FAILED.value,
        error_message="Package not found",
        hint="Check package installation",
    )

    todo = TodoItem(
        location="node.camera.socket.image",
        field="direction",
        current_value="!todo",
        status=TodoStatus.PENDING.value,
        context=context,
    )

    stats = ConversionStats(
        total_nodes=5,
        total_todos=3,
        pending_todos=2,
        completed_todos=1,
        completion_rate=0.33,
    )

    metadata = ConversionMetadata(
        source_file="/path/to/launch.py",
        source_hash="abc123",
        generated_at=datetime.now().isoformat(),
        converter_version="0.1.0",
        todos=[todo],
        stats=stats,
        node_sources={"camera": NodeSource(launch_file="/path/to/launch.py")},
    )

    # Serialize
    from dataclasses import asdict

    data = asdict(metadata)
    json_str = json.dumps(data)

    # Deserialize
    loaded_data = json.loads(json_str)
    assert loaded_data["source_file"] == "/path/to/launch.py"
    assert loaded_data["todos"][0]["location"] == "node.camera.socket.image"
    assert loaded_data["stats"]["total_nodes"] == 5


# ============================================================================
# F68: Metadata Persistence Tests
# ============================================================================


def test_save_load_metadata():
    """Test saving and loading metadata from JSON file."""
    with tempfile.TemporaryDirectory() as tmpdir:
        plan_path = Path(tmpdir) / "test.plan.yaml"
        plan_path.touch()

        # Create test metadata
        metadata = ConversionMetadata(
            source_file="/test/launch.py",
            source_hash="test_hash",
            generated_at=datetime.now().isoformat(),
            converter_version="0.1.0",
            todos=[
                TodoItem(
                    location="node.test.socket.topic",
                    field="direction",
                    current_value="!todo",
                    status=TodoStatus.PENDING.value,
                    context=TodoContext(hint="Test hint"),
                )
            ],
            stats=ConversionStats(total_nodes=1, total_todos=1),
            node_sources={},
        )

        # Save
        manager = MetadataManager()
        manager.save_metadata(metadata, plan_path)

        # Check file exists
        meta_path = plan_path.with_suffix(".plan.meta.json")
        assert meta_path.exists()

        # Load
        loaded = manager.load_metadata(plan_path)
        assert loaded is not None
        assert loaded.source_file == "/test/launch.py"
        assert loaded.source_hash == "test_hash"
        assert len(loaded.todos) == 1
        assert loaded.todos[0].location == "node.test.socket.topic"
        assert loaded.todos[0].status == TodoStatus.PENDING.value


def test_load_missing_metadata():
    """Test loading metadata when file doesn't exist."""
    with tempfile.TemporaryDirectory() as tmpdir:
        plan_path = Path(tmpdir) / "nonexistent.plan.yaml"

        manager = MetadataManager()
        loaded = manager.load_metadata(plan_path)

        assert loaded is None


# ============================================================================
# F69: Plan YAML Parsing & TODO Discovery Tests
# ============================================================================


def test_plan_yaml_parsing():
    """Test parsing plan YAML structure."""
    with tempfile.TemporaryDirectory() as tmpdir:
        plan_path = Path(tmpdir) / "test.plan.yaml"
        plan_content = """
node:
  camera:
    pkg: sensor_pkg
    exec: camera_node
    socket:
      image: !pub

link:
  image_link: !pubsub
    type: sensor_msgs/msg/Image
    src: [camera/image]
"""
        plan_path.write_text(plan_content)

        parser = PlanParser()
        plan_data = parser.parse_plan_yaml(plan_path)

        assert "node" in plan_data
        assert "camera" in plan_data["node"]
        assert plan_data["node"]["camera"]["pkg"] == "sensor_pkg"


def test_find_todos_in_plan():
    """Test discovering TODO markers in plan."""
    plan_data = {
        "node": {
            "camera": {"socket": {"image": "!todo", "data": "!pub"}},
            "detector": {"socket": {"result": "!sub"}},
        },
        "link": {"test_link": {"type": "TODO", "src": [], "dst": []}},
        "arg": {"camera_fps": {"type": "!todo", "default": 30}},
    }

    parser = PlanParser()
    todos = parser.find_todos_in_plan(plan_data)

    assert len(todos) == 3
    locations = [t.location for t in todos]
    assert "node.camera.socket.image" in locations
    assert "link.test_link.type" in locations
    assert "arg.camera_fps.type" in locations


def test_get_value_at_path():
    """Test navigating plan structure with JSONPath."""
    plan_data = {
        "node": {
            "camera": {"pkg": "sensor_pkg", "socket": {"image": "!pub"}},
        }
    }

    parser = PlanParser()

    # Valid paths
    assert parser.get_value_at_path(plan_data, "node.camera.pkg") == "sensor_pkg"
    assert parser.get_value_at_path(plan_data, "node.camera.socket.image") == "!pub"

    # Invalid paths
    assert parser.get_value_at_path(plan_data, "node.nonexistent") is None
    assert parser.get_value_at_path(plan_data, "node.camera.socket.missing") is None


# ============================================================================
# F70: TODO Status Update Tests
# ============================================================================


def test_detect_completed_todos():
    """Test detecting TODOs that user has completed."""
    # Create metadata with pending TODOs
    metadata = ConversionMetadata(
        source_file="/test/launch.py",
        source_hash="test",
        generated_at=datetime.now().isoformat(),
        converter_version="0.1.0",
        todos=[
            TodoItem(
                location="node.camera.socket.image",
                field="direction",
                current_value="!todo",
                status=TodoStatus.PENDING.value,
                context=TodoContext(),
            ),
            TodoItem(
                location="link.test.type",
                field="message_type",
                current_value="TODO",
                status=TodoStatus.PENDING.value,
                context=TodoContext(),
            ),
        ],
        stats=ConversionStats(),
        node_sources={},
    )

    # Plan data with first TODO completed
    plan_data = {
        "node": {"camera": {"socket": {"image": "!pub"}}},  # Completed!
        "link": {"test": {"type": "TODO"}},  # Still pending
    }

    updater = TodoStatusUpdater()
    completed = updater.detect_completed_todos(metadata, plan_data)

    assert len(completed) == 1
    assert completed[0].location == "node.camera.socket.image"
    assert completed[0].old_value == "!todo"
    assert completed[0].new_value == "!pub"

    # Check status updated
    assert metadata.todos[0].status == TodoStatus.COMPLETED.value
    assert metadata.todos[1].status == TodoStatus.PENDING.value


def test_detect_completed_todos_ignores_todo_markers():
    """Test that TODO markers are not counted as completed."""
    metadata = ConversionMetadata(
        source_file="/test/launch.py",
        source_hash="test",
        generated_at=datetime.now().isoformat(),
        converter_version="0.1.0",
        todos=[
            TodoItem(
                location="node.test.socket.data",
                field="direction",
                current_value="!todo",
                status=TodoStatus.PENDING.value,
                context=TodoContext(),
            )
        ],
        stats=ConversionStats(),
        node_sources={},
    )

    # Plan still has TODO marker (different instance but same value)
    plan_data = {"node": {"test": {"socket": {"data": "!todo"}}}}

    updater = TodoStatusUpdater()
    completed = updater.detect_completed_todos(metadata, plan_data)

    assert len(completed) == 0
    assert metadata.todos[0].status == TodoStatus.PENDING.value


def test_check_metadata_staleness():
    """Test detecting when source file has changed."""
    with tempfile.TemporaryDirectory() as tmpdir:
        source_path = Path(tmpdir) / "launch.py"
        source_path.write_text("# Original content")

        import hashlib

        original_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()

        metadata = ConversionMetadata(
            source_file=str(source_path),
            source_hash=original_hash,
            generated_at=datetime.now().isoformat(),
            converter_version="0.1.0",
            todos=[],
            stats=ConversionStats(),
            node_sources={},
        )

        updater = TodoStatusUpdater()

        # No change - should be None
        warning = updater.check_metadata_staleness(metadata, source_path)
        assert warning is None

        # Modify source file
        source_path.write_text("# Modified content")

        # Should detect change
        warning = updater.check_metadata_staleness(metadata, source_path)
        assert warning is not None
        assert "Warning" in warning
        assert "Source file has changed" in warning


# ============================================================================
# F71: Statistics Calculation Tests
# ============================================================================


def test_calculate_stats():
    """Test statistics calculation from metadata and plan."""
    from launch2plan.statistics import calculate_stats

    metadata = ConversionMetadata(
        source_file="/test/launch.py",
        source_hash="test",
        generated_at=datetime.now().isoformat(),
        converter_version="0.1.0",
        todos=[
            TodoItem(
                location="node.a.socket.x",
                field="direction",
                current_value="!todo",
                status=TodoStatus.PENDING.value,
                context=TodoContext(),
            ),
            TodoItem(
                location="node.b.socket.y",
                field="direction",
                current_value="!todo",
                status=TodoStatus.COMPLETED.value,
                context=TodoContext(),
            ),
            TodoItem(
                location="link.test.type",
                field="message_type",
                current_value="TODO",
                status=TodoStatus.PENDING.value,
                context=TodoContext(),
            ),
        ],
        stats=ConversionStats(
            nodes_introspected=2,
            nodes_failed_introspection=1,
            sockets_from_introspection=5,
            sockets_requiring_user_input=3,
            introspection_time_ms=1500,
            conversion_time_ms=3000,
        ),
        node_sources={},
    )

    plan_data = {
        "node": {"a": {}, "b": {}, "c": {}},
        "link": {"link1": {}, "link2": {}},
        "arg": {"arg1": {}, "arg2": {}, "arg3": {}},
        "include": {"inc1": {}},
    }

    stats = calculate_stats(metadata, plan_data)

    assert stats.total_nodes == 3
    assert stats.total_links == 2
    assert stats.total_arguments == 3
    assert stats.total_includes == 1
    assert stats.total_todos == 3
    assert stats.pending_todos == 2
    assert stats.completed_todos == 1
    assert stats.completion_rate == pytest.approx(1 / 3)
    assert stats.nodes_introspected == 2
    assert stats.introspection_time_ms == 1500


def test_calculate_stats_empty_plan():
    """Test statistics with empty plan."""
    from launch2plan.statistics import calculate_stats

    metadata = ConversionMetadata(
        source_file="/test/launch.py",
        source_hash="test",
        generated_at=datetime.now().isoformat(),
        converter_version="0.1.0",
        todos=[],
        stats=ConversionStats(),
        node_sources={},
    )

    plan_data = {}

    stats = calculate_stats(metadata, plan_data)

    assert stats.total_nodes == 0
    assert stats.total_todos == 0
    assert stats.completion_rate == 1.0  # No TODOs = 100% complete


# ============================================================================
# F67: TODO Collection During Conversion Tests
# ============================================================================


def test_builder_collects_socket_todos():
    """Test that PlanBuilder collects TODOs for sockets with unknown direction."""
    from launch2plan.builder import PlanBuilder
    from launch2plan.inference import InferredSocket
    from launch2plan.visitor import NodeMetadata

    # Create nodes
    nodes = [
        NodeMetadata(
            package="test_pkg",
            executable="test_node",
            name="test_node",
            remappings=[("topic", "/remapped_topic")],
        )
    ]

    # Create inferred sockets with TODO
    inferred_sockets = {
        "test_node": [
            InferredSocket(
                socket_name="topic",
                direction=None,  # Unknown - should create TODO
                remapped_to="/remapped_topic",
                node_package="test_pkg",
                node_executable="test_node",
                introspection_failed=False,
            )
        ]
    }

    # Build plan
    builder = PlanBuilder()
    builder.build_plan(nodes=nodes, inferred_sockets=inferred_sockets)

    # Check that TODO was collected
    todos = builder.get_todos()
    assert len(todos) == 1
    assert todos[0].location == "node.test_node.socket.topic"
    assert todos[0].field == "direction"
    assert todos[0].current_value == "!todo"
    assert todos[0].status == TodoStatus.PENDING.value


def test_builder_collects_link_todos():
    """Test that PlanBuilder collects TODOs for links with unknown message type."""
    from launch2plan.builder import PlanBuilder
    from launch2plan.inference import InferredSocket
    from launch2plan.visitor import NodeMetadata

    # Create nodes with matching remappings
    nodes = [
        NodeMetadata(
            package="pub_pkg",
            executable="pub_node",
            name="publisher",
            remappings=[("output", "/shared_topic")],
        ),
        NodeMetadata(
            package="sub_pkg",
            executable="sub_node",
            name="subscriber",
            remappings=[("input", "/shared_topic")],
        ),
    ]

    # Create inferred sockets with direction but no message type
    inferred_sockets = {
        "publisher": [
            InferredSocket(
                socket_name="output",
                direction="!pub",
                message_type=None,  # Unknown - should create TODO in link
                remapped_to="/shared_topic",
            )
        ],
        "subscriber": [
            InferredSocket(
                socket_name="input",
                direction="!sub",
                message_type=None,  # Unknown - should create TODO in link
                remapped_to="/shared_topic",
            )
        ],
    }

    # Build plan
    builder = PlanBuilder()
    builder.build_plan(nodes=nodes, inferred_sockets=inferred_sockets)

    # Check that TODO was collected for link
    todos = builder.get_todos()
    assert len(todos) == 1
    assert todos[0].location == "link.shared_topic.type"
    assert todos[0].field == "message_type"
    assert todos[0].current_value == "TODO"
    assert todos[0].status == TodoStatus.PENDING.value
