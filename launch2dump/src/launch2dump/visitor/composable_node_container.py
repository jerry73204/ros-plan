"""
Composable node container visitor
"""

from typing import List, Optional

from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ..result import ComposableNodeInfo, ContainerInfo
from .session import CollectionSession
from .substitution_tracker import track_substitutions_in_value


def visit_composable_node_container(
    container: ComposableNodeContainer,
    context: LaunchContext,
    session: CollectionSession,
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit a ComposableNodeContainer action and extract metadata.

    :param container: The ComposableNodeContainer action
    :param context: The launch context
    :param session: The collection session
    :return: None (no process spawned)
    """
    # Perform substitutions
    container._perform_substitutions(context)

    def substitute(subst):
        """Helper to perform substitutions."""
        result = perform_substitutions(context, normalize_to_list_of_substitutions(subst))
        track_substitutions_in_value(subst, session)
        return result

    # Extract container information using name mangling
    # ComposableNodeContainer extends Node, so use _Node__ prefix
    package = substitute(container._Node__package)
    executable = substitute(container._Node__node_executable)
    name = container._Node__node_name
    if name is not None:
        name = container._Node__expanded_node_name

    namespace = None
    if container._Node__expanded_node_namespace != "":
        namespace = container._Node__expanded_node_namespace

    # Extract composable nodes
    composable_nodes = []
    if container._ComposableNodeContainer__composable_node_descriptions is not None:
        for comp_node_desc in container._ComposableNodeContainer__composable_node_descriptions:
            composable_nodes.append(extract_composable_node(comp_node_desc, context, session))

    # Create ContainerInfo and add to session
    container_info = ContainerInfo(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        composable_nodes=composable_nodes,
    )

    session.add_container(container_info)

    # Don't spawn process
    return None


def extract_composable_node_parameters(
    comp_node: ComposableNode, context: LaunchContext, substitute
):
    """
    Extract and resolve parameters from a ComposableNode.

    ComposableNode parameters are stored as tuples of substitutions that need to be resolved.
    Both parameter keys and values can contain substitutions.

    :param comp_node: The ComposableNode description
    :param context: The launch context
    :param substitute: Function to perform substitutions
    :return: List of parameter dictionaries
    """
    import os
    import sys
    from typing import Any, Dict, List

    import yaml as pyyaml
    from launch_ros.descriptions import ParameterFile

    parameters: List[Any] = []

    if comp_node.parameters is None:
        return parameters

    for param in comp_node.parameters:
        if isinstance(param, str):
            # Parameter file path
            resolved_path = substitute(param)

            # Inline temporary parameter files
            if resolved_path.startswith("/tmp/launch_params_"):
                try:
                    if os.path.exists(resolved_path):
                        with open(resolved_path, "r") as f:
                            file_params = pyyaml.safe_load(f)
                            if file_params:
                                parameters.append(file_params)
                    else:
                        parameters.append({"__param_file": resolved_path})
                except Exception:
                    parameters.append({"__param_file": resolved_path})
            else:
                parameters.append({"__param_file": resolved_path})
        elif isinstance(param, ParameterFile):
            # ParameterFile object - resolve the param_file path
            try:
                resolved_path = substitute(param.param_file)

                # Inline temporary parameter files
                if resolved_path.startswith("/tmp/launch_params_"):
                    try:
                        if os.path.exists(resolved_path):
                            with open(resolved_path, "r") as f:
                                file_params = pyyaml.safe_load(f)
                                if file_params:
                                    parameters.append(file_params)
                        else:
                            parameters.append({"__param_file": resolved_path})
                    except Exception:
                        parameters.append({"__param_file": resolved_path})
                else:
                    parameters.append({"__param_file": resolved_path})
            except Exception as e:
                # If resolution fails, convert to string (will be caught by serialization warning)
                print(
                    f"Warning: Failed to resolve ParameterFile: {param}, error: {e}",
                    file=sys.stderr,
                )
                parameters.append(str(param))
        elif isinstance(param, dict):
            # Parameter dictionary - resolve all keys and values
            resolved_dict: Dict[str, Any] = {}
            for key, value in param.items():
                # Resolve key (may be a tuple of substitutions)
                resolved_key = substitute(key)

                # Resolve value (may be a tuple of substitutions or ParameterValue)
                resolved_value = resolve_parameter_value(value, context, substitute)

                resolved_dict[resolved_key] = resolved_value

            parameters.append(resolved_dict)
        else:
            # Unknown parameter type - try to resolve it
            try:
                resolved = substitute(param)
                parameters.append(resolved)
            except Exception as e:
                # If substitution fails, keep original (will be stringified by clean_dict)
                print(f"Warning: Failed to resolve parameter {param}: {e}", file=sys.stderr)
                parameters.append(param)

    return parameters


def resolve_parameter_value(value, context: LaunchContext, substitute):
    """
    Resolve a parameter value which may contain substitutions.

    :param value: The parameter value (may be str, tuple, ParameterValue, etc.)
    :param context: The launch context
    :param substitute: Function to perform substitutions
    :return: Resolved value
    """
    from launch_ros.parameter_descriptions import ParameterValue

    if isinstance(value, ParameterValue):
        # ParameterValue wraps substitutions - need to extract the actual value
        # The value is stored in value.value
        try:
            # Recursively resolve the wrapped value
            inner_value = value.value
            return resolve_parameter_value(inner_value, context, substitute)
        except Exception:
            # If substitution fails, try to get string representation
            return str(value)
    elif isinstance(value, (list, tuple)):
        # Check if it's a tuple of substitutions (common case)
        if len(value) > 0 and hasattr(value[0], "perform"):
            # This is a tuple/list of Substitution objects
            return substitute(value)
        else:
            # Regular list/tuple - resolve each element recursively
            return [resolve_parameter_value(item, context, substitute) for item in value]
    elif isinstance(value, dict):
        # Nested dictionary
        resolved_dict = {}
        for k, v in value.items():
            resolved_key = substitute(k) if isinstance(k, (list, tuple)) else str(k)
            resolved_value = resolve_parameter_value(v, context, substitute)
            resolved_dict[resolved_key] = resolved_value
        return resolved_dict
    elif hasattr(value, "perform"):
        # Single Substitution object
        return substitute(value)
    else:
        # Primitive value (str, int, float, bool, etc.)
        return value


def extract_composable_node(
    comp_node: ComposableNode, context: LaunchContext, session: CollectionSession
) -> ComposableNodeInfo:
    """
    Extract metadata from a ComposableNode description.

    :param comp_node: The ComposableNode description
    :param context: The launch context
    :param session: The collection session
    :return: ComposableNodeInfo
    """

    def substitute(subst):
        """Helper to perform substitutions."""
        result = perform_substitutions(context, normalize_to_list_of_substitutions(subst))
        track_substitutions_in_value(subst, session)
        return result

    # Track substitutions BEFORE resolution
    track_substitutions_in_value(comp_node.node_plugin, session)
    if comp_node.node_name:
        track_substitutions_in_value(comp_node.node_name, session)
    if comp_node.node_namespace:
        track_substitutions_in_value(comp_node.node_namespace, session)
    if comp_node.parameters is not None:
        for param in comp_node.parameters:
            track_substitutions_in_value(param, session)

    plugin = substitute(comp_node.node_plugin)
    name = substitute(comp_node.node_name) if comp_node.node_name else None
    namespace = substitute(comp_node.node_namespace) if comp_node.node_namespace else None

    # Extract and resolve parameters
    parameters = extract_composable_node_parameters(comp_node, context, substitute)

    # Extract remappings
    remappings = []
    if comp_node.remappings is not None:
        for remap in comp_node.remappings:
            if isinstance(remap, tuple) and len(remap) == 2:
                from_topic = substitute(remap[0])
                to_topic = substitute(remap[1])
                remappings.append((from_topic, to_topic))

    # Extract extra arguments
    extra_arguments = []
    if comp_node.extra_arguments is not None:
        for arg in comp_node.extra_arguments:
            if isinstance(arg, dict):
                for key, value in arg.items():
                    resolved_key = substitute(key)
                    resolved_value = substitute(value)
                    extra_arguments.append(f"{resolved_key}:={resolved_value}")

    return ComposableNodeInfo(
        plugin=plugin,
        name=name,
        namespace=namespace,
        parameters=parameters,
        remappings=remappings,
        extra_arguments=extra_arguments,
    )
