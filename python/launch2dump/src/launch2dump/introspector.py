"""
RMW introspection wrapper for discovering node interfaces.

This module provides a Python wrapper around rmw_introspect_cpp for
discovering ROS 2 node interfaces (publishers, subscriptions, services, clients)
without running actual middleware communication.
"""

import json
import logging
import os
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class QoSProfile:
    """QoS profile for a ROS 2 interface."""

    reliability: str
    durability: str
    history: str
    depth: int


@dataclass
class PublisherInfo:
    """Publisher interface metadata."""

    topic_name: str
    message_type: str
    qos: QoSProfile
    node_name: str
    node_namespace: str


@dataclass
class SubscriptionInfo:
    """Subscription interface metadata."""

    topic_name: str
    message_type: str
    qos: QoSProfile
    node_name: str
    node_namespace: str


@dataclass
class ServiceInfo:
    """Service interface metadata."""

    service_name: str
    service_type: str
    node_name: str
    node_namespace: str


@dataclass
class ClientInfo:
    """Client interface metadata."""

    service_name: str
    service_type: str
    node_name: str
    node_namespace: str


@dataclass
class IntrospectionResult:
    """Result of introspecting a ROS 2 node."""

    success: bool
    nodes: List[str] = field(default_factory=list)
    publishers: List[PublisherInfo] = field(default_factory=list)
    subscriptions: List[SubscriptionInfo] = field(default_factory=list)
    services: List[ServiceInfo] = field(default_factory=list)
    clients: List[ClientInfo] = field(default_factory=list)
    error: Optional[str] = None
    format_version: Optional[str] = None
    timestamp: Optional[str] = None


def introspect_node(
    package: str,
    executable: str,
    *,
    parameters: Optional[List[Dict[str, Any]]] = None,
    remappings: Optional[List[Tuple[str, str]]] = None,
    namespace: Optional[str] = None,
    node_name: Optional[str] = None,
    arguments: Optional[List[str]] = None,
    timeout: float = 3.0,
    workspace_dir: Optional[str] = None,
) -> IntrospectionResult:
    """
    Introspect a ROS 2 node to discover its interfaces.

    This function spawns the node with rmw_introspect_cpp as the RMW implementation,
    which captures interface metadata without actual middleware communication.

    Args:
        package: ROS 2 package name
        executable: Executable name within the package
        parameters: List of parameter dictionaries to pass to the node
        remappings: List of (from, to) topic remapping tuples
        namespace: Node namespace (e.g., "/robot1")
        node_name: Override node name
        arguments: Additional ROS arguments
        timeout: Maximum time to wait for node initialization (seconds)
        workspace_dir: Path to ROS 2 workspace (defaults to current directory parent)

    Returns:
        IntrospectionResult with discovered interfaces or error information

    Example:
        >>> result = introspect_node("demo_nodes_cpp", "talker")
        >>> if result.success:
        ...     print(f"Found {len(result.publishers)} publishers")
        ...     for pub in result.publishers:
        ...         print(f"  {pub.topic_name}: {pub.message_type}")
    """
    # Create temporary file path for JSON output (don't open it yet)
    import uuid

    output_path = f"/tmp/rmw_introspect_{uuid.uuid4().hex[:8]}.json"

    try:
        # Build command
        cmd = _build_ros2_command(
            package,
            executable,
            parameters=parameters,
            remappings=remappings,
            namespace=namespace,
            node_name=node_name,
            arguments=arguments,
        )

        # Setup environment - we need to export these AFTER sourcing ROS 2
        env = os.environ.copy()

        # Source ROS 2 and workspace, then export our variables
        ros_setup = "/opt/ros/humble/setup.bash"
        bash_cmd_parts = [f"source {ros_setup}"]

        if workspace_dir:
            workspace_path = Path(workspace_dir).resolve()
            setup_script = workspace_path / "install" / "setup.bash"
            if setup_script.exists():
                bash_cmd_parts.append(f"source {setup_script}")

        # Export introspection variables AFTER sourcing
        bash_cmd_parts.append("export RMW_IMPLEMENTATION=rmw_introspect_cpp")
        bash_cmd_parts.append(f"export RMW_INTROSPECT_OUTPUT='{output_path}'")
        bash_cmd_parts.append("export RMW_INTROSPECT_AUTO_EXPORT=1")

        # Add the actual command
        bash_cmd_parts.append(" ".join(cmd))
        bash_cmd = " && ".join(bash_cmd_parts)

        cmd = ["bash", "-c", bash_cmd]

        logger.debug(f"Running introspection: {bash_cmd}")
        logger.debug(f"Output path: {output_path}")

        # Run node with timeout
        try:
            subprocess.run(
                cmd,
                env=env,
                timeout=timeout,
                capture_output=True,
                text=True,
            )
        except subprocess.TimeoutExpired:
            # Timeout is expected - node runs until killed
            logger.debug(f"Node timed out after {timeout}s (expected)")
            # Give the node a moment to write output file in signal handler
            import time

            time.sleep(0.5)
        except FileNotFoundError as e:
            return IntrospectionResult(success=False, error=f"Command not found: {e}")

        # Read JSON output
        if not os.path.exists(output_path):
            return IntrospectionResult(
                success=False,
                error=f"Introspection output file not created: {output_path}",
            )

        with open(output_path, "r") as f:
            data = json.load(f)

        # Parse introspection data
        return _parse_introspection_data(data)

    except json.JSONDecodeError as e:
        return IntrospectionResult(success=False, error=f"Failed to parse JSON output: {e}")
    except Exception as e:
        return IntrospectionResult(success=False, error=f"Introspection failed: {e}")
    finally:
        # Cleanup temporary file
        if os.path.exists(output_path):
            try:
                os.unlink(output_path)
            except OSError:
                pass


def _build_ros2_command(
    package: str,
    executable: str,
    *,
    parameters: Optional[List[Dict[str, Any]]] = None,
    remappings: Optional[List[Tuple[str, str]]] = None,
    namespace: Optional[str] = None,
    node_name: Optional[str] = None,
    arguments: Optional[List[str]] = None,
) -> List[str]:
    """Build ros2 run command with arguments."""
    cmd = ["ros2", "run", package, executable]

    # Add ROS arguments
    ros_args = []

    if node_name:
        ros_args.extend(["-r", f"__node:={node_name}"])

    if namespace:
        ros_args.extend(["-r", f"__ns:={namespace}"])

    if remappings:
        for from_topic, to_topic in remappings:
            ros_args.extend(["-r", f"{from_topic}:={to_topic}"])

    if parameters:
        # Create temporary parameter file
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as param_file:
            import yaml

            yaml.dump({"/**": {"ros__parameters": parameters[0]}}, param_file)
            ros_args.extend(["--params-file", param_file.name])

    if arguments:
        ros_args.extend(arguments)

    if ros_args:
        cmd.append("--ros-args")
        cmd.extend(ros_args)

    return cmd


def _parse_introspection_data(data: Dict[str, Any]) -> IntrospectionResult:
    """Parse JSON introspection data into structured result."""
    result = IntrospectionResult(success=True)

    result.format_version = data.get("format_version")
    result.timestamp = data.get("timestamp")
    result.nodes = data.get("nodes", [])

    # Parse publishers
    for pub_data in data.get("publishers", []):
        qos = QoSProfile(
            reliability=pub_data["qos"]["reliability"],
            durability=pub_data["qos"]["durability"],
            history=pub_data["qos"]["history"],
            depth=pub_data["qos"]["depth"],
        )
        pub = PublisherInfo(
            topic_name=pub_data["topic_name"],
            message_type=pub_data["message_type"],
            qos=qos,
            node_name=pub_data["node_name"],
            node_namespace=pub_data["node_namespace"],
        )
        result.publishers.append(pub)

    # Parse subscriptions
    for sub_data in data.get("subscriptions", []):
        qos = QoSProfile(
            reliability=sub_data["qos"]["reliability"],
            durability=sub_data["qos"]["durability"],
            history=sub_data["qos"]["history"],
            depth=sub_data["qos"]["depth"],
        )
        sub = SubscriptionInfo(
            topic_name=sub_data["topic_name"],
            message_type=sub_data["message_type"],
            qos=qos,
            node_name=sub_data["node_name"],
            node_namespace=sub_data["node_namespace"],
        )
        result.subscriptions.append(sub)

    # Parse services
    for srv_data in data.get("services", []):
        srv = ServiceInfo(
            service_name=srv_data["service_name"],
            service_type=srv_data["service_type"],
            node_name=srv_data["node_name"],
            node_namespace=srv_data["node_namespace"],
        )
        result.services.append(srv)

    # Parse clients
    for client_data in data.get("clients", []):
        client = ClientInfo(
            service_name=client_data["service_name"],
            service_type=client_data["service_type"],
            node_name=client_data["node_name"],
            node_namespace=client_data["node_namespace"],
        )
        result.clients.append(client)

    return result
