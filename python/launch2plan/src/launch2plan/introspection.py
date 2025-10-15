"""
RMW introspection integration for socket inference.

This module provides the IntrospectionService class that uses ros2-introspect
to query actual node interfaces without running middleware.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

from ros2_introspect import IntrospectionResult, introspect_node


@dataclass
class SocketInfo:
    """Information about a socket discovered via introspection."""

    name: str  # Topic name
    direction: str  # '!pub' or '!sub'
    message_type: str
    qos: Optional[Dict] = None


@dataclass
class TopicInfo:
    """Information about all sockets for a specific topic in a node."""

    publishers: List[SocketInfo]
    subscriptions: List[SocketInfo]


class IntrospectionService:
    """
    Service for introspecting node interfaces using rmw_introspect_cpp.

    Caches introspection results per package::executable to avoid redundant queries.
    """

    def __init__(self):
        """Initialize the introspection service."""
        self.cache: Dict[str, IntrospectionResult] = {}

    def introspect_node_interfaces(
        self, package: str, executable: str, timeout: float = 3.0
    ) -> Optional[IntrospectionResult]:
        """
        Query node interfaces using rmw_introspect_cpp.

        Args:
            package: ROS 2 package name
            executable: Executable name
            timeout: Introspection timeout in seconds

        Returns:
            IntrospectionResult if successful, None otherwise
        """
        cache_key = f"{package}::{executable}"

        # Check cache
        if cache_key in self.cache:
            return self.cache[cache_key]

        # Run introspection
        result = introspect_node(package, executable, timeout=timeout)

        # Cache successful results
        if result.success:
            self.cache[cache_key] = result
            return result

        return None

    def get_socket_info(
        self, package: str, executable: str, topic_name: str
    ) -> Optional[SocketInfo]:
        """
        Get socket direction and message type for a specific topic.

        Args:
            package: ROS 2 package name
            executable: Executable name
            topic_name: Topic name (without remapping, as defined in the node)

        Returns:
            SocketInfo if found, None otherwise
        """
        result = self.introspect_node_interfaces(package, executable)

        if not result:
            return None

        # Check publishers
        for pub in result.publishers:
            if pub.topic_name == topic_name:
                qos_dict = {
                    "reliability": pub.qos.reliability,
                    "durability": pub.qos.durability,
                    "history": pub.qos.history,
                    "depth": pub.qos.depth,
                }
                return SocketInfo(
                    name=pub.topic_name,
                    direction="!pub",
                    message_type=pub.message_type,
                    qos=qos_dict,
                )

        # Check subscriptions
        for sub in result.subscriptions:
            if sub.topic_name == topic_name:
                qos_dict = {
                    "reliability": sub.qos.reliability,
                    "durability": sub.qos.durability,
                    "history": sub.qos.history,
                    "depth": sub.qos.depth,
                }
                return SocketInfo(
                    name=sub.topic_name,
                    direction="!sub",
                    message_type=sub.message_type,
                    qos=qos_dict,
                )

        return None

    def get_all_topics(self, package: str, executable: str) -> Optional[TopicInfo]:
        """
        Get all publishers and subscriptions for a node.

        Args:
            package: ROS 2 package name
            executable: Executable name

        Returns:
            TopicInfo with all publishers and subscriptions, or None if introspection failed
        """
        result = self.introspect_node_interfaces(package, executable)

        if not result:
            return None

        publishers = []
        subscriptions = []

        # Extract publishers
        for pub in result.publishers:
            qos_dict = {
                "reliability": pub.qos.reliability,
                "durability": pub.qos.durability,
                "history": pub.qos.history,
                "depth": pub.qos.depth,
            }
            publishers.append(
                SocketInfo(
                    name=pub.topic_name,
                    direction="!pub",
                    message_type=pub.message_type,
                    qos=qos_dict,
                )
            )

        # Extract subscriptions
        for sub in result.subscriptions:
            qos_dict = {
                "reliability": sub.qos.reliability,
                "durability": sub.qos.durability,
                "history": sub.qos.history,
                "depth": sub.qos.depth,
            }
            subscriptions.append(
                SocketInfo(
                    name=sub.topic_name,
                    direction="!sub",
                    message_type=sub.message_type,
                    qos=qos_dict,
                )
            )

        return TopicInfo(publishers=publishers, subscriptions=subscriptions)

    def clear_cache(self):
        """Clear the introspection cache."""
        self.cache.clear()
