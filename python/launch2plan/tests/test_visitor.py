"""Tests for the branch-exploring visitor (Phase 12.1)."""

from pathlib import Path

import pytest

from launch2plan.converter import convert_launch_file


@pytest.fixture
def fixtures_dir():
    """Return the fixtures directory."""
    return Path(__file__).parent / "fixtures"


def test_visit_simple_node(fixtures_dir):
    """Test visiting a launch file with a single node."""
    launch_file = fixtures_dir / "simple.launch.py"
    result = convert_launch_file(launch_file)

    assert len(result.nodes) == 1
    assert len(result.errors) == 0

    node = result.nodes[0]
    assert node.package == "demo_nodes_cpp"
    assert node.executable == "talker"
    assert node.name == "talker"
    assert node.condition_expr is None  # No condition


def test_visit_multiple_nodes(fixtures_dir):
    """Test visiting a launch file with multiple nodes."""
    launch_file = fixtures_dir / "multiple_nodes.launch.py"
    result = convert_launch_file(launch_file)

    assert len(result.nodes) == 2
    assert len(result.errors) == 0

    # Check talker
    talker = result.nodes[0]
    assert talker.package == "demo_nodes_cpp"
    assert talker.executable == "talker"
    assert talker.name == "talker"
    assert ("chatter", "/demo/chatter") in talker.remappings

    # Check listener
    listener = result.nodes[1]
    assert listener.package == "demo_nodes_cpp"
    assert listener.executable == "listener"
    assert listener.name == "listener"
    assert ("chatter", "/demo/chatter") in listener.remappings
