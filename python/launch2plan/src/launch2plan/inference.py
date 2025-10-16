"""
Socket inference module for launch2plan.

This module integrates introspection with node conversion to infer socket
directions and message types. When introspection fails or doesn't find a match,
it generates TODO markers with helpful comments.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

from .introspection import IntrospectionService, SocketInfo
from .visitor import NodeMetadata


@dataclass
class InferredSocket:
    """Information about an inferred socket."""

    socket_name: str  # The socket name from remapping (e.g., "chatter")
    direction: Optional[str] = None  # "!pub", "!sub", or None for TODO
    message_type: Optional[str] = None
    qos: Optional[Dict] = None
    remapped_to: Optional[str] = None  # The target topic name
    comment: Optional[str] = None  # Helpful comment for TODO markers
    source: str = "unknown"  # "introspection", "todo"
    # For TODO tracking
    node_package: Optional[str] = None
    node_executable: Optional[str] = None
    introspection_failed: bool = False
    error_message: Optional[str] = None


def socket_info_to_dict(socket_info: SocketInfo) -> Dict:
    """Convert SocketInfo to dictionary representation."""
    return {
        "direction": socket_info.direction,
        "message_type": socket_info.message_type,
        "qos": socket_info.qos,
    }


class SocketInferenceEngine:
    """Engine for inferring socket information using introspection."""

    def __init__(self, introspection_service: IntrospectionService):
        """
        Initialize the inference engine.

        Args:
            introspection_service: Service for introspecting node interfaces
        """
        self.introspection_service = introspection_service

    def infer_sockets_for_node(self, node: NodeMetadata) -> List[InferredSocket]:
        """
        Infer socket information for a node using introspection.

        Args:
            node: Node metadata from visitor

        Returns:
            List of inferred sockets (with TODO markers if needed)
        """
        sockets = []

        # Try introspection first
        introspection_result = self.introspection_service.introspect_node_interfaces(
            node.package, node.executable
        )

        # Process each remapping
        for socket_name, remapped_to in node.remappings:
            socket = self._infer_single_socket(
                node, socket_name, remapped_to, introspection_result is not None
            )
            sockets.append(socket)

        return sockets

    def _infer_single_socket(
        self,
        node: NodeMetadata,
        socket_name: str,
        remapped_to: str,
        introspection_available: bool,
    ) -> InferredSocket:
        """
        Infer information for a single socket.

        Args:
            node: Node metadata
            socket_name: Original socket name (before remapping)
            remapped_to: Target topic name (after remapping)
            introspection_available: Whether introspection succeeded

        Returns:
            InferredSocket with direction/type or TODO marker
        """
        if not introspection_available:
            # Introspection failed completely
            return InferredSocket(
                socket_name=socket_name,
                direction=None,
                message_type=None,
                remapped_to=remapped_to,
                comment=(
                    f"TODO: Introspection failed for {node.package}::{node.executable}. "
                    f"Please specify socket direction (!pub or !sub) and message type. "
                    f"Remapping: {socket_name} -> {remapped_to}"
                ),
                source="todo",
                node_package=node.package,
                node_executable=node.executable,
                introspection_failed=True,
            )

        # Try to find socket in introspection results
        # Need to normalize topic names (handle leading slash)
        socket_info = self._find_socket_in_introspection(node, socket_name)

        if socket_info:
            # Found it!
            return InferredSocket(
                socket_name=socket_name,
                direction=socket_info.direction,
                message_type=socket_info.message_type,
                qos=socket_info.qos,
                remapped_to=remapped_to,
                comment=None,  # No comment needed for successful inference
                source="introspection",
                node_package=node.package,
                node_executable=node.executable,
                introspection_failed=False,
            )

        # Socket not found in introspection results
        return InferredSocket(
            socket_name=socket_name,
            direction=None,
            message_type=None,
            remapped_to=remapped_to,
            comment=(
                f"TODO: Socket '{socket_name}' not found in introspection results for "
                f"{node.package}::{node.executable}. Please specify socket direction "
                f"(!pub or !sub) and message type. Remapping: {socket_name} -> {remapped_to}"
            ),
            source="todo",
            node_package=node.package,
            node_executable=node.executable,
            introspection_failed=False,
        )

    def _find_socket_in_introspection(
        self, node: NodeMetadata, socket_name: str
    ) -> Optional[SocketInfo]:
        """
        Find socket information in introspection results.

        Handles topic name normalization (with/without leading slash).

        Args:
            node: Node metadata
            socket_name: Socket name to find

        Returns:
            SocketInfo if found, None otherwise
        """
        # Try exact match first
        socket_info = self.introspection_service.get_socket_info(
            node.package, node.executable, socket_name
        )
        if socket_info:
            return socket_info

        # Try with leading slash
        if not socket_name.startswith("/"):
            socket_info = self.introspection_service.get_socket_info(
                node.package, node.executable, f"/{socket_name}"
            )
            if socket_info:
                return socket_info

        # Try without leading slash
        if socket_name.startswith("/"):
            socket_info = self.introspection_service.get_socket_info(
                node.package, node.executable, socket_name[1:]
            )
            if socket_info:
                return socket_info

        return None


def infer_sockets_for_nodes(
    nodes: List[NodeMetadata], introspection_service: IntrospectionService
) -> Dict[str, List[InferredSocket]]:
    """
    Infer sockets for multiple nodes.

    Args:
        nodes: List of node metadata
        introspection_service: Service for introspecting nodes

    Returns:
        Dictionary mapping node identifiers to their inferred sockets
    """
    engine = SocketInferenceEngine(introspection_service)
    result = {}

    for node in nodes:
        # Use node name as identifier (or package::executable if name is None)
        node_id = node.name if node.name else f"{node.package}::{node.executable}"
        result[node_id] = engine.infer_sockets_for_node(node)

    return result
