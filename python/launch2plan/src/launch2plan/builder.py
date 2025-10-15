"""
Plan builder module for launch2plan.

This module generates ROS-Plan YAML from discovered nodes and inferred sockets.
"""

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ruamel.yaml import YAML

from .arg_inference import infer_argument_type
from .inference import InferredSocket
from .visitor import IncludeMetadata, LaunchArgumentMetadata, NodeMetadata


@dataclass
class PlanLink:
    """Information about a plan link connecting nodes."""

    name: str  # Link name (derived from topic or user-specified)
    message_type: Optional[str]  # Message type (from introspection or TODO)
    sources: List[str]  # List of "node_id/socket_name" publishers
    destinations: List[str]  # List of "node_id/socket_name" subscribers
    comment: Optional[str] = None  # Comment for TODO markers


class PlanBuilder:
    """Builder for generating ROS-Plan YAML from launch file metadata."""

    def __init__(self):
        """Initialize the plan builder."""
        self.yaml = YAML()
        self.yaml.default_flow_style = False
        self.yaml.preserve_quotes = True
        self.yaml.width = 100

    def build_plan(
        self,
        nodes: List[NodeMetadata],
        inferred_sockets: Dict[str, List[InferredSocket]],
        launch_arguments: Optional[List[LaunchArgumentMetadata]] = None,
        includes: Optional[List[IncludeMetadata]] = None,
    ) -> Dict[str, Any]:
        """
        Build a complete plan from nodes and inferred sockets.

        Args:
            nodes: List of node metadata from visitor
            inferred_sockets: Map of node_id -> inferred sockets
            launch_arguments: List of launch argument metadata (optional)
            includes: List of include metadata (optional)

        Returns:
            Plan dictionary ready for YAML serialization
        """
        plan = {}

        # Generate arg section if launch arguments exist
        if launch_arguments:
            plan["arg"] = self._build_arg_section(launch_arguments)

        # Generate include section if includes exist
        if includes:
            plan["include"] = self._build_include_section(includes)

        # Generate node section
        if nodes:
            plan["node"] = self._build_node_section(nodes, inferred_sockets)

        # Infer and generate link section
        links = self._infer_links(nodes, inferred_sockets)
        if links:
            plan["link"] = self._build_link_section(links)

        return plan

    def _build_include_section(self, includes: List[IncludeMetadata]) -> Dict[str, Any]:
        """
        Build include section from launch includes.

        Args:
            includes: List of include metadata

        Returns:
            Dictionary of include definitions
        """
        include_section = {}

        for idx, include in enumerate(includes):
            # Generate include identifier from file name (without extension)
            # Remove ".launch" suffix if present (e.g., "camera.launch.py" -> "camera")
            stem = include.file_path.stem
            if stem.endswith(".launch"):
                stem = stem[:-7]  # Remove ".launch"

            include_name = stem

            # Handle duplicate names
            if include_name in include_section:
                include_name = f"{include_name}_{idx}"

            # Build include definition
            include_def = {"file": f"{stem}.plan.yaml"}

            # Add arguments if any
            if include.arguments:
                arg_dict = {}
                for arg_name, arg_value in include.arguments.items():
                    # Infer type and convert value
                    type_tag, converted_value = infer_argument_type(arg_value)

                    if type_tag == "!bool":
                        arg_dict[arg_name] = {type_tag: converted_value == "true"}
                    elif type_tag == "!i64":
                        arg_dict[arg_name] = {type_tag: int(converted_value)}
                    elif type_tag == "!f64":
                        arg_dict[arg_name] = {type_tag: float(converted_value)}
                    else:
                        # String or substitution
                        arg_dict[arg_name] = arg_value

                include_def["arg"] = arg_dict

            # Add when clause if conditional
            if include.condition_expr:
                include_def["when"] = include.condition_expr

            include_section[include_name] = include_def

        return include_section

    def _build_arg_section(self, launch_arguments: List[LaunchArgumentMetadata]) -> Dict[str, Any]:
        """
        Build arg section from launch arguments.

        Args:
            launch_arguments: List of launch argument metadata

        Returns:
            Dictionary of argument definitions with type tags
        """
        arg_section = {}

        for arg in launch_arguments:
            # Infer type from default value
            type_tag, converted_value = infer_argument_type(arg.default_value)

            # Create argument entry
            if type_tag == "!todo":
                # No default value - mark as TODO
                arg_section[arg.name] = {type_tag: None}
            elif type_tag == "!bool":
                # Boolean value
                bool_value = converted_value == "true"
                arg_section[arg.name] = {type_tag: bool_value}
            elif type_tag == "!i64":
                # Integer value
                arg_section[arg.name] = {type_tag: int(converted_value)}
            elif type_tag == "!f64":
                # Float value
                arg_section[arg.name] = {type_tag: float(converted_value)}
            else:
                # String value
                arg_section[arg.name] = {type_tag: converted_value}

        return arg_section

    def _build_node_section(
        self,
        nodes: List[NodeMetadata],
        inferred_sockets: Dict[str, List[InferredSocket]],
    ) -> Dict[str, Any]:
        """
        Build the node section of the plan.

        Args:
            nodes: List of node metadata
            inferred_sockets: Map of node_id -> inferred sockets

        Returns:
            Dictionary of node definitions
        """
        node_section = {}

        for node in nodes:
            # Get node identifier (name or package::executable)
            node_id = node.name if node.name else f"{node.package}::{node.executable}"

            # Build node definition
            node_def = {
                "pkg": node.package,
                "exec": node.executable,
            }

            # Add namespace if specified
            if node.namespace:
                node_def["ns"] = node.namespace

            # Add parameters if specified
            if node.parameters:
                node_def["param"] = self._build_param_section(node.parameters)

            # Add sockets if any remappings exist
            if node_id in inferred_sockets and inferred_sockets[node_id]:
                node_def["socket"] = self._build_socket_section(inferred_sockets[node_id])

            # Add when clause if conditional
            if node.condition_expr:
                node_def["when"] = node.condition_expr

            node_section[node_id] = node_def

        return node_section

    def _build_param_section(self, parameters: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Build parameter section from node parameters.

        Args:
            parameters: List of parameter dictionaries

        Returns:
            Merged parameter dictionary with LaunchConfiguration substitutions converted
        """
        merged_params = {}
        for param_dict in parameters:
            # Skip __param_file markers (we inline these already)
            if "__param_file" in param_dict:
                continue
            # Convert LaunchConfiguration substitutions
            converted_params = self._convert_launch_configurations(param_dict)
            merged_params.update(converted_params)
        return merged_params

    def _convert_launch_configurations(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Convert LaunchConfiguration objects to $(arg_name) syntax.

        Recursively processes parameter dictionaries to find and convert
        LaunchConfiguration references.

        Args:
            params: Parameter dictionary potentially containing LaunchConfiguration objects

        Returns:
            Dictionary with LaunchConfiguration objects replaced by $(arg_name) strings
        """
        from launch.substitutions import LaunchConfiguration

        converted = {}
        for key, value in params.items():
            if isinstance(value, LaunchConfiguration):
                # Convert to $(arg_name) syntax
                arg_name = (
                    value.variable_name[0].text
                    if hasattr(value.variable_name[0], "text")
                    else str(value.variable_name[0])
                )
                converted[key] = f"$({arg_name})"
            elif isinstance(value, dict):
                # Recursively convert nested dictionaries
                converted[key] = self._convert_launch_configurations(value)
            elif isinstance(value, list):
                # Convert lists
                converted[key] = [
                    self._convert_launch_configurations(item)
                    if isinstance(item, dict)
                    else f"$({item.variable_name[0].text if hasattr(item.variable_name[0], 'text') else str(item.variable_name[0])})"
                    if isinstance(item, LaunchConfiguration)
                    else item
                    for item in value
                ]
            else:
                # Keep other values as-is
                converted[key] = value
        return converted

    def _build_socket_section(self, sockets: List[InferredSocket]) -> Dict[str, Any]:
        """
        Build socket section from inferred sockets.

        Args:
            sockets: List of inferred sockets

        Returns:
            Dictionary of socket definitions
        """
        socket_section = {}

        for socket in sockets:
            if socket.direction and socket.direction != "TODO":
                # Successfully inferred socket
                socket_section[socket.socket_name] = socket.direction
            else:
                # TODO marker needed
                socket_section[socket.socket_name] = "!todo"

        return socket_section

    def _infer_links(
        self,
        nodes: List[NodeMetadata],
        inferred_sockets: Dict[str, List[InferredSocket]],
    ) -> List[PlanLink]:
        """
        Infer links by matching remappings across nodes.

        Links are created when multiple nodes remap sockets to the same topic.

        Args:
            nodes: List of node metadata
            inferred_sockets: Map of node_id -> inferred sockets

        Returns:
            List of inferred links
        """
        # Group sockets by remapped topic name
        topic_map: Dict[str, Dict[str, List[Tuple[str, InferredSocket]]]] = defaultdict(
            lambda: {"publishers": [], "subscribers": []}
        )

        for node in nodes:
            node_id = node.name if node.name else f"{node.package}::{node.executable}"

            if node_id not in inferred_sockets:
                continue

            for socket in inferred_sockets[node_id]:
                if not socket.remapped_to:
                    continue

                # Normalize topic name (ensure leading slash)
                topic = socket.remapped_to
                if not topic.startswith("/"):
                    topic = f"/{topic}"

                # Add to appropriate list based on direction
                if socket.direction == "!pub":
                    topic_map[topic]["publishers"].append((node_id, socket))
                elif socket.direction == "!sub":
                    topic_map[topic]["subscribers"].append((node_id, socket))

        # Generate links for topics with both publishers and subscribers
        links = []
        for topic, connections in topic_map.items():
            publishers = connections["publishers"]
            subscribers = connections["subscribers"]

            # Only create link if there are both publishers and subscribers
            if publishers and subscribers:
                # Derive link name from topic (remove leading slash, replace / with _)
                link_name = topic.lstrip("/").replace("/", "_")
                if not link_name:
                    link_name = "link"

                # Get message type from first publisher (should be consistent)
                message_type = None
                comment = None

                # Try to get message type from publishers
                for _, socket in publishers:
                    if socket.message_type and socket.message_type != "TODO":
                        message_type = socket.message_type
                        break

                # If no message type from publishers, try subscribers
                if not message_type:
                    for _, socket in subscribers:
                        if socket.message_type and socket.message_type != "TODO":
                            message_type = socket.message_type
                            break

                # If still no message type, mark as TODO
                if not message_type:
                    message_type = "TODO"
                    comment = (
                        f"TODO: Specify message type for topic {topic}. "
                        "Unable to determine from introspection."
                    )

                # Build source and destination lists
                sources = [f"{node_id}/{socket.socket_name}" for node_id, socket in publishers]
                destinations = [
                    f"{node_id}/{socket.socket_name}" for node_id, socket in subscribers
                ]

                links.append(
                    PlanLink(
                        name=link_name,
                        message_type=message_type,
                        sources=sources,
                        destinations=destinations,
                        comment=comment,
                    )
                )

        return links

    def _build_link_section(self, links: List[PlanLink]) -> Dict[str, Any]:
        """
        Build link section from inferred links.

        Args:
            links: List of inferred links

        Returns:
            Dictionary of link definitions
        """
        link_section = {}

        for link in links:
            link_def = {
                "!pubsub": {
                    "type": link.message_type,
                    "src": link.sources,
                    "dst": link.destinations,
                }
            }

            link_section[link.name] = link_def

        return link_section

    def write_plan(self, plan: Dict[str, Any], output_path: Path) -> None:
        """
        Write plan to YAML file.

        Args:
            plan: Plan dictionary
            output_path: Path to output file
        """
        with open(output_path, "w") as f:
            self.yaml.dump(plan, f)

    def plan_to_yaml_string(self, plan: Dict[str, Any]) -> str:
        """
        Convert plan to YAML string.

        Args:
            plan: Plan dictionary

        Returns:
            YAML string representation
        """
        import io

        stream = io.StringIO()
        self.yaml.dump(plan, stream)
        return stream.getvalue()
