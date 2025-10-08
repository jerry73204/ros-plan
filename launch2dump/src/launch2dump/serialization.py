"""
Serialization utilities for converting LaunchResult to JSON/YAML formats.
"""

import json
from typing import Any, Dict

try:
    import yaml
except ImportError:
    yaml = None

from .result import ComposableNodeInfo, ContainerInfo, LaunchResult, NodeInfo


def clean_dict(obj):
    """
    Recursively convert tuples and other non-serializable types to serializable forms.
    """
    if isinstance(obj, dict):
        return {str(k): clean_dict(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [clean_dict(item) for item in obj]
    elif hasattr(obj, "__dict__"):
        # Convert objects with __dict__ to their string representation
        return str(obj)
    elif isinstance(obj, (str, int, float, bool, type(None))):
        return obj
    else:
        # For any other type, convert to string
        return str(obj)


def serialize_launch_result(result: LaunchResult, format: str = "yaml") -> str:
    """
    Serialize a LaunchResult to JSON or YAML string.

    :param result: The LaunchResult to serialize
    :param format: Output format ('json' or 'yaml')
    :return: Serialized string
    """
    data = launch_result_to_dict(result)
    # Clean data to ensure all keys are strings
    data = clean_dict(data)

    if format == "json":
        return json.dumps(data, indent=2)
    elif format == "yaml":
        if yaml is None:
            raise ImportError("PyYAML is not installed. Install it with: pip install pyyaml")
        return yaml.dump(data, default_flow_style=False, sort_keys=False)
    else:
        raise ValueError(f"Unsupported format: {format}. Use 'json' or 'yaml'")


def launch_result_to_dict(result: LaunchResult) -> Dict[str, Any]:
    """
    Convert a LaunchResult to a dictionary.

    :param result: The LaunchResult to convert
    :return: Dictionary representation
    """
    return {
        "nodes": [node_info_to_dict(node) for node in result.nodes],
        "containers": [container_info_to_dict(container) for container in result.containers],
        "parameter_dependencies": sorted(list(result.parameter_dependencies)),
        "errors": result.errors,
    }


def node_info_to_dict(node: NodeInfo) -> Dict[str, Any]:
    """
    Convert a NodeInfo to a dictionary.

    :param node: The NodeInfo to convert
    :return: Dictionary representation
    """
    data = {
        "package": node.package,
        "executable": node.executable,
    }

    if node.name is not None:
        data["name"] = node.name

    if node.namespace is not None:
        data["namespace"] = node.namespace

    if node.parameters:
        data["parameters"] = node.parameters

    if node.remappings:
        data["remappings"] = [
            {"from": str(from_name), "to": str(to_name)} for from_name, to_name in node.remappings
        ]

    if node.arguments:
        data["arguments"] = node.arguments

    if node.env_vars:
        data["env_vars"] = node.env_vars

    return data


def container_info_to_dict(container: ContainerInfo) -> Dict[str, Any]:
    """
    Convert a ContainerInfo to a dictionary.

    :param container: The ContainerInfo to convert
    :return: Dictionary representation
    """
    data = {
        "name": container.name,
        "namespace": container.namespace,
        "composable_nodes": [composable_node_to_dict(node) for node in container.composable_nodes],
    }

    return data


def composable_node_to_dict(node: ComposableNodeInfo) -> Dict[str, Any]:
    """
    Convert a ComposableNodeInfo to a dictionary.

    :param node: The ComposableNodeInfo to convert
    :return: Dictionary representation
    """
    data = {
        "plugin": node.plugin,
    }

    if node.name is not None:
        data["name"] = node.name

    if node.namespace is not None:
        data["namespace"] = node.namespace

    if node.parameters:
        data["parameters"] = node.parameters

    if node.remappings:
        data["remappings"] = [
            {"from": str(from_name), "to": str(to_name)} for from_name, to_name in node.remappings
        ]

    if node.extra_arguments:
        data["extra_arguments"] = node.extra_arguments

    return data
