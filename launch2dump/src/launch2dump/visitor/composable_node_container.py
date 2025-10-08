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

    plugin = substitute(comp_node.node_plugin)
    name = substitute(comp_node.node_name) if comp_node.node_name else None
    namespace = substitute(comp_node.node_namespace) if comp_node.node_namespace else None

    # Extract parameters (simplified)
    parameters = []
    if comp_node.parameters is not None:
        for param in comp_node.parameters:
            if isinstance(param, dict):
                parameters.append(param)

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
                    extra_arguments.append(f"{key}:={substitute(value)}")

    return ComposableNodeInfo(
        plugin=plugin,
        name=name,
        namespace=namespace,
        parameters=parameters,
        remappings=remappings,
        extra_arguments=extra_arguments,
    )
