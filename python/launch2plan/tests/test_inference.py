"""
Tests for socket inference module.

Phase 3: Socket Inference & TODO Generation
"""

from unittest.mock import Mock

import pytest
from ros2_introspect import IntrospectionResult, PublisherInfo, QoSProfile, SubscriptionInfo

from launch2plan.inference import (
    SocketInferenceEngine,
    infer_sockets_for_nodes,
)
from launch2plan.introspection import IntrospectionService, SocketInfo
from launch2plan.visitor import NodeMetadata


@pytest.fixture
def mock_introspection_service():
    """Create a mock introspection service."""
    service = Mock(spec=IntrospectionService)
    return service


@pytest.fixture
def sample_node():
    """Create a sample node with remappings."""
    return NodeMetadata(
        package="demo_nodes_cpp",
        executable="talker",
        name="talker_node",
        remappings=[("chatter", "/topic")],
    )


def test_resolve_from_introspection(mock_introspection_service, sample_node):
    """Test successful socket resolution via introspection."""
    # Setup: introspection returns valid socket info
    qos = QoSProfile(
        reliability="reliable",
        durability="volatile",
        history="keep_last",
        depth=10,
    )
    mock_introspection_service.introspect_node_interfaces.return_value = IntrospectionResult(
        success=True,
        publishers=[
            PublisherInfo(
                topic_name="chatter",
                message_type="std_msgs/msg/String",
                qos=qos,
                node_name="talker",
                node_namespace="",
            )
        ],
        subscriptions=[],
    )
    mock_introspection_service.get_socket_info.return_value = SocketInfo(
        name="chatter",
        direction="!pub",
        message_type="std_msgs/msg/String",
        qos={
            "reliability": "reliable",
            "durability": "volatile",
            "history": "keep_last",
            "depth": 10,
        },
    )

    # Execute: infer sockets
    engine = SocketInferenceEngine(mock_introspection_service)
    sockets = engine.infer_sockets_for_node(sample_node)

    # Verify: should have 1 successfully inferred socket
    assert len(sockets) == 1
    socket = sockets[0]
    assert socket.socket_name == "chatter"
    assert socket.direction == "!pub"
    assert socket.message_type == "std_msgs/msg/String"
    assert socket.qos == {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 10,
    }
    assert socket.remapped_to == "/topic"
    assert socket.comment is None
    assert socket.source == "introspection"


def test_introspection_not_available(mock_introspection_service, sample_node):
    """Test TODO generation when introspection completely fails."""
    # Setup: introspection returns None (failure)
    mock_introspection_service.introspect_node_interfaces.return_value = None

    # Execute: infer sockets
    engine = SocketInferenceEngine(mock_introspection_service)
    sockets = engine.infer_sockets_for_node(sample_node)

    # Verify: should have 1 TODO socket
    assert len(sockets) == 1
    socket = sockets[0]
    assert socket.socket_name == "chatter"
    assert socket.direction is None
    assert socket.message_type is None
    assert socket.remapped_to == "/topic"
    assert socket.comment is not None
    assert "TODO" in socket.comment
    assert "Introspection failed" in socket.comment
    assert "demo_nodes_cpp::talker" in socket.comment
    assert socket.source == "todo"


def test_socket_not_found_in_introspection(mock_introspection_service, sample_node):
    """Test TODO generation when socket not in introspection results."""
    # Setup: introspection succeeds but socket not found
    mock_introspection_service.introspect_node_interfaces.return_value = IntrospectionResult(
        success=True,
        publishers=[],  # Empty - socket not found
        subscriptions=[],
    )
    mock_introspection_service.get_socket_info.return_value = None

    # Execute: infer sockets
    engine = SocketInferenceEngine(mock_introspection_service)
    sockets = engine.infer_sockets_for_node(sample_node)

    # Verify: should have 1 TODO socket
    assert len(sockets) == 1
    socket = sockets[0]
    assert socket.socket_name == "chatter"
    assert socket.direction is None
    assert socket.message_type is None
    assert socket.remapped_to == "/topic"
    assert socket.comment is not None
    assert "TODO" in socket.comment
    assert "not found in introspection results" in socket.comment
    assert "chatter" in socket.comment
    assert socket.source == "todo"


def test_todo_comment_generation(mock_introspection_service):
    """Verify helpful comments are generated for TODO markers."""
    # Setup: node with multiple remappings, introspection fails
    node = NodeMetadata(
        package="my_package",
        executable="my_node",
        remappings=[("input", "/input_topic"), ("output", "/output_topic")],
    )
    mock_introspection_service.introspect_node_interfaces.return_value = None

    # Execute: infer sockets
    engine = SocketInferenceEngine(mock_introspection_service)
    sockets = engine.infer_sockets_for_node(node)

    # Verify: both sockets should have helpful TODO comments
    assert len(sockets) == 2

    for socket in sockets:
        assert socket.comment is not None
        # Comment should contain helpful information
        assert "TODO" in socket.comment
        assert "my_package::my_node" in socket.comment
        assert socket.socket_name in socket.comment
        assert socket.remapped_to in socket.comment
        assert "!pub or !sub" in socket.comment
        assert "message type" in socket.comment


def test_topic_name_matching(mock_introspection_service):
    """Test topic name normalization (with/without leading slash)."""
    # Setup: node with remapping without leading slash
    node = NodeMetadata(
        package="demo_nodes_cpp",
        executable="talker",
        remappings=[("chatter", "/topic")],
    )

    # Introspection returns topic WITH leading slash
    qos = QoSProfile(
        reliability="reliable",
        durability="volatile",
        history="keep_last",
        depth=10,
    )
    mock_introspection_service.introspect_node_interfaces.return_value = IntrospectionResult(
        success=True,
        publishers=[
            PublisherInfo(
                topic_name="/chatter",  # With leading slash
                message_type="std_msgs/msg/String",
                qos=qos,
                node_name="talker",
                node_namespace="",
            )
        ],
        subscriptions=[],
    )

    # Mock get_socket_info to simulate the normalization logic
    def mock_get_socket_info(package, executable, name):
        if name == "chatter":
            return None  # Exact match fails
        elif name == "/chatter":
            return SocketInfo(
                name="/chatter",
                direction="!pub",
                message_type="std_msgs/msg/String",
                qos={},
            )
        return None

    mock_introspection_service.get_socket_info.side_effect = mock_get_socket_info

    # Execute: infer sockets
    engine = SocketInferenceEngine(mock_introspection_service)
    sockets = engine.infer_sockets_for_node(node)

    # Verify: should successfully match despite different slash conventions
    assert len(sockets) == 1
    socket = sockets[0]
    assert socket.socket_name == "chatter"
    assert socket.direction == "!pub"
    assert socket.message_type == "std_msgs/msg/String"
    assert socket.source == "introspection"

    # Verify that get_socket_info was called multiple times for normalization
    assert mock_introspection_service.get_socket_info.call_count == 2
    # First call: exact match "chatter"
    # Second call: with leading slash "/chatter"


def test_infer_sockets_for_multiple_nodes(mock_introspection_service):
    """Test batch inference for multiple nodes."""
    # Setup: two nodes
    node1 = NodeMetadata(
        package="demo_nodes_cpp",
        executable="talker",
        name="talker_node",
        remappings=[("chatter", "/topic1")],
    )
    node2 = NodeMetadata(
        package="demo_nodes_cpp",
        executable="listener",
        name="listener_node",
        remappings=[("chatter", "/topic2")],
    )

    # Mock introspection for both nodes
    qos = QoSProfile(
        reliability="reliable",
        durability="volatile",
        history="keep_last",
        depth=10,
    )

    def mock_introspect(package, executable):
        if executable == "talker":
            return IntrospectionResult(
                success=True,
                publishers=[
                    PublisherInfo(
                        topic_name="chatter",
                        message_type="std_msgs/msg/String",
                        qos=qos,
                        node_name="talker",
                        node_namespace="",
                    )
                ],
                subscriptions=[],
            )
        elif executable == "listener":
            return IntrospectionResult(
                success=True,
                publishers=[],
                subscriptions=[
                    SubscriptionInfo(
                        topic_name="chatter",
                        message_type="std_msgs/msg/String",
                        qos=qos,
                        node_name="listener",
                        node_namespace="",
                    )
                ],
            )
        return None

    mock_introspection_service.introspect_node_interfaces.side_effect = mock_introspect

    def mock_get_socket(package, executable, name):
        if executable == "talker" and name == "chatter":
            return SocketInfo(
                name="chatter", direction="!pub", message_type="std_msgs/msg/String", qos={}
            )
        elif executable == "listener" and name == "chatter":
            return SocketInfo(
                name="chatter", direction="!sub", message_type="std_msgs/msg/String", qos={}
            )
        return None

    mock_introspection_service.get_socket_info.side_effect = mock_get_socket

    # Execute: batch inference
    result = infer_sockets_for_nodes([node1, node2], mock_introspection_service)

    # Verify: should have sockets for both nodes
    assert len(result) == 2
    assert "talker_node" in result
    assert "listener_node" in result

    # Verify talker has publisher
    talker_sockets = result["talker_node"]
    assert len(talker_sockets) == 1
    assert talker_sockets[0].direction == "!pub"
    assert talker_sockets[0].source == "introspection"

    # Verify listener has subscriber
    listener_sockets = result["listener_node"]
    assert len(listener_sockets) == 1
    assert listener_sockets[0].direction == "!sub"
    assert listener_sockets[0].source == "introspection"
