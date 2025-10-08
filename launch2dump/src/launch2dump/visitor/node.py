"""
Node visitor for extracting node metadata
"""

from typing import Any, Dict, List, Optional, Tuple

from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions.node import Node
from launch_ros.descriptions import Parameter

from ..result import NodeInfo
from .session import CollectionSession
from .substitution_tracker import track_substitutions_in_value


def visit_node(
    node: Node, context: LaunchContext, session: CollectionSession
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit a Node action and extract metadata.

    Uses Python name mangling to access private members of the Node class.

    :param node: The Node action
    :param context: The launch context
    :param session: The collection session
    :return: None (no process spawned)
    """
    # Track parameter dependencies BEFORE performing substitutions
    # This ensures we capture LaunchConfiguration uses before they're resolved
    track_substitutions_in_value(node._Node__package, session)
    track_substitutions_in_value(node._Node__node_executable, session)
    if node._Node__node_name is not None:
        track_substitutions_in_value(node._Node__node_name, session)
    if node._Node__parameters is not None:
        for param in node._Node__parameters:
            track_substitutions_in_value(param, session)
    if node._Node__remappings is not None:
        track_substitutions_in_value(node._Node__remappings, session)
    if node._Node__arguments is not None:
        track_substitutions_in_value(node._Node__arguments, session)
    if node._Node__ros_arguments is not None:
        track_substitutions_in_value(node._Node__ros_arguments, session)

    # Now perform substitutions to resolve all placeholders
    node._perform_substitutions(context)

    def substitute(subst):
        """Helper to perform substitutions."""
        result = perform_substitutions(context, normalize_to_list_of_substitutions(subst))
        return result

    # Extract basic node information using name mangling
    package = substitute(node._Node__package)
    executable = substitute(node._Node__node_executable)

    # Extract node name (optional)
    name = None
    if node._Node__node_name is not None:
        name = node._Node__expanded_node_name

    # Extract namespace (optional)
    namespace = None
    if node._Node__expanded_node_namespace != "":
        namespace = node._Node__expanded_node_namespace

    # Extract parameters
    parameters = extract_parameters(node, context)

    # Extract remappings
    remappings = extract_remappings(node)

    # Extract arguments
    arguments = extract_arguments(node, substitute)

    # Extract environment variables
    env_vars = extract_env_vars(node, substitute)

    # Create NodeInfo and add to session
    node_info = NodeInfo(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=parameters,
        remappings=remappings,
        arguments=arguments,
        env_vars=env_vars,
    )

    session.add_node(node_info)

    # Don't spawn process - return None
    return None


def extract_parameters(node: Node, context: LaunchContext) -> List[Dict[str, Any]]:
    """Extract parameters from node."""
    import os

    import yaml as pyyaml

    parameters = []
    node_params = node._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                # Parameter file
                param_file_path = str(entry)

                # Check if it's a temporary file and inline it
                if param_file_path.startswith("/tmp/launch_params_"):
                    # Read and inline temporary parameter file
                    try:
                        if os.path.exists(param_file_path):
                            with open(param_file_path, "r") as f:
                                file_params = pyyaml.safe_load(f)
                                if file_params:
                                    parameters.append(file_params)
                                else:
                                    # Empty file, keep reference
                                    parameters.append({"__param_file": param_file_path})
                        else:
                            # File doesn't exist, keep reference
                            parameters.append({"__param_file": param_file_path})
                    except Exception:
                        # If reading fails, keep reference
                        parameters.append({"__param_file": param_file_path})
                else:
                    # Keep reference to permanent files
                    parameters.append({"__param_file": param_file_path})
            else:
                # Individual parameter
                if isinstance(entry, Parameter):
                    param_dict = {}
                    name = entry.name
                    value = entry.value

                    # Handle nested parameter names (e.g., "namespace.param")
                    if isinstance(name, str) and "." in name:
                        # Split nested parameter name
                        parts = name.split(".")
                        current = param_dict
                        for part in parts[:-1]:
                            if part not in current:
                                current[part] = {}
                            current = current[part]
                        current[parts[-1]] = value
                    else:
                        param_dict[str(name)] = value

                    parameters.append(param_dict)

    return parameters


def extract_remappings(node: Node) -> List[Tuple[str, str]]:
    """Extract remappings from node."""
    if node.expanded_remapping_rules is None:
        return []
    return list(node.expanded_remapping_rules)


def extract_arguments(node: Node, substitute) -> List[str]:
    """Extract command-line arguments from node."""
    arguments = []

    if node._Node__arguments is not None:
        for arg in node._Node__arguments:
            arguments.append(substitute(arg))

    if node._Node__ros_arguments is not None:
        for arg in node._Node__ros_arguments:
            arguments.append(substitute(arg))

    return arguments


def extract_env_vars(node: Node, substitute) -> Dict[str, str]:
    """Extract environment variables from node."""
    env_vars = {}

    # Try to get env from different possible attributes
    env_list = None
    if hasattr(node, "env") and node.env is not None:
        env_list = node.env
    elif hasattr(node, "_Node__env") and node._Node__env is not None:
        env_list = node._Node__env

    if env_list is not None:
        for env_var in env_list:
            # env_var is typically a tuple of (name, value)
            if isinstance(env_var, tuple) and len(env_var) == 2:
                name = substitute(env_var[0])
                value = substitute(env_var[1])
                env_vars[name] = value

    return env_vars
