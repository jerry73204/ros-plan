"""Tests for RMW introspection integration (Phase 12.2)."""

import pytest

from launch2plan.introspection import IntrospectionService


@pytest.fixture
def introspection_service():
    """Create an introspection service."""
    return IntrospectionService()


def test_introspect_demo_nodes(introspection_service):
    """Test introspection of demo_nodes_cpp (talker/listener)."""
    # Introspect talker
    result = introspection_service.introspect_node_interfaces("demo_nodes_cpp", "talker")
    assert result is not None
    assert result.success is True
    assert len(result.publishers) > 0


def test_introspection_cache(introspection_service):
    """Test that introspection results are cached."""
    # First call
    result1 = introspection_service.introspect_node_interfaces("demo_nodes_cpp", "talker")
    assert result1 is not None

    # Second call should return cached result (same object)
    result2 = introspection_service.introspect_node_interfaces("demo_nodes_cpp", "talker")
    assert result2 is result1  # Same object due to caching


def test_introspection_fallback(introspection_service):
    """Test graceful handling of introspection failures."""
    # Try to introspect a non-existent package
    result = introspection_service.introspect_node_interfaces("nonexistent_pkg", "nonexistent_exe")
    assert result is None


def test_socket_direction_resolution(introspection_service):
    """Test resolution of socket directions (pub/sub)."""
    # Get socket info for talker's chatter topic
    # Note: introspection returns fully qualified topic names with leading slash
    socket_info = introspection_service.get_socket_info("demo_nodes_cpp", "talker", "/chatter")
    assert socket_info is not None
    assert socket_info.direction == "!pub"
    assert socket_info.message_type == "std_msgs/msg/String"

    # Get socket info for listener's chatter topic
    socket_info = introspection_service.get_socket_info("demo_nodes_cpp", "listener", "/chatter")
    assert socket_info is not None
    assert socket_info.direction == "!sub"
    assert socket_info.message_type == "std_msgs/msg/String"


def test_message_type_resolution(introspection_service):
    """Test extraction of message types from introspection."""
    topics = introspection_service.get_all_topics("demo_nodes_cpp", "talker")
    assert topics is not None
    assert len(topics.publishers) > 0

    # Find the chatter publisher
    chatter_pub = next(
        (p for p in topics.publishers if p.message_type == "std_msgs/msg/String"), None
    )
    assert chatter_pub is not None
    assert chatter_pub.direction == "!pub"
