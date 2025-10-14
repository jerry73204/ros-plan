"""
Launch visitor for branch exploration.

Unlike launch2dump which evaluates conditions and follows only True branches,
this visitor explores ALL branches to generate complete plans with `when` clauses.
"""

from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from launch.action import Action
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a, normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions.node import Node


@dataclass
class NodeMetadata:
    """Metadata extracted from a launch node."""

    package: str
    executable: str
    name: Optional[str] = None
    namespace: Optional[str] = None
    parameters: List[Dict[str, Any]] = field(default_factory=list)
    remappings: List[Tuple[str, str]] = field(default_factory=list)
    arguments: List[str] = field(default_factory=list)
    condition_expr: Optional[str] = None  # When clause (Lua expression)


@dataclass
class IncludeMetadata:
    """Metadata extracted from a launch include."""

    file_path: Path
    arguments: Dict[str, Any] = field(default_factory=dict)
    condition_expr: Optional[str] = None


@dataclass
class BranchExplorerSession:
    """
    Session for exploring all branches in a launch file.

    Unlike launch2dump's CollectionSession which only follows True branches,
    this session explores ALL conditional branches.
    """

    nodes: List[NodeMetadata] = field(default_factory=list)
    includes: List[IncludeMetadata] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    # Condition tracking
    condition_stack: List[str] = field(default_factory=list)

    # Cycle detection for includes
    include_stack: List[Path] = field(default_factory=list)
    visited_files: Set[Path] = field(default_factory=set)

    def add_node(self, node: NodeMetadata) -> None:
        """Add a discovered node."""
        self.nodes.append(node)

    def add_include(self, include: IncludeMetadata) -> None:
        """Add a discovered include."""
        self.includes.append(include)

    def add_error(self, error: str) -> None:
        """Add an error message."""
        self.errors.append(error)

    @contextmanager
    def condition_context(self, condition_expr: Optional[str]):
        """Context manager for tracking condition stack."""
        if condition_expr:
            self.condition_stack.append(condition_expr)
        try:
            yield
        finally:
            if condition_expr:
                self.condition_stack.pop()

    def get_current_condition(self) -> Optional[str]:
        """
        Get the current condition expression (compound if nested).

        Returns None if no conditions are active.
        Returns a Lua expression combining all active conditions with 'and'.
        """
        if not self.condition_stack:
            return None
        if len(self.condition_stack) == 1:
            return self.condition_stack[0]
        # Combine with 'and'
        return " and ".join(f"({expr})" for expr in self.condition_stack)

    def check_cycle(self, file_path: Path) -> bool:
        """Check if including this file would create a cycle."""
        return file_path in self.include_stack


def extract_condition_expression(condition) -> Optional[str]:
    """
    Extract condition expression and convert to Lua.

    Examples:
    - IfCondition(LaunchConfiguration('use_sim_time')) -> "$(use_sim_time)"
    - UnlessCondition(...) -> "$(not use_sim_time)"
    """
    from launch.conditions import IfCondition, UnlessCondition
    from launch.substitutions import LaunchConfiguration

    if condition is None:
        return None

    # Handle IfCondition
    if isinstance(condition, IfCondition):
        # Extract the predicate expression
        predicate = condition.predicate_expr
        if isinstance(predicate, list) and len(predicate) == 1:
            subst = predicate[0]
            if isinstance(subst, LaunchConfiguration):
                return f"$({subst.variable_name[0].text})"
        # Fallback: try to get string representation
        return "true"  # Placeholder

    # Handle UnlessCondition
    elif isinstance(condition, UnlessCondition):
        predicate = condition.predicate_expr
        if isinstance(predicate, list) and len(predicate) == 1:
            subst = predicate[0]
            if isinstance(subst, LaunchConfiguration):
                return f"$(not {subst.variable_name[0].text})"
        return "false"  # Placeholder

    # Unknown condition type
    return "true"


def visit_action(
    action: Action, context: LaunchContext, session: BranchExplorerSession
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit an action and explore ALL branches (not just True branches).

    This is the key difference from launch2dump: we don't evaluate conditions,
    we track them and explore all possible paths.
    """
    condition = action.condition

    # Extract condition expression (don't evaluate it)
    condition_expr = extract_condition_expression(condition) if condition else None

    # Visit the action with condition context
    with session.condition_context(condition_expr):
        return visit_action_by_class(action, context, session)


def visit_action_by_class(
    action: Action, context: LaunchContext, session: BranchExplorerSession
) -> Optional[List[LaunchDescriptionEntity]]:
    """Dispatch action to appropriate visitor based on type."""
    if is_a(action, Node):
        return visit_node(action, context, session)
    elif is_a(action, IncludeLaunchDescription):
        return visit_include_launch_description(action, context, session)
    else:
        # For other actions (GroupAction, SetEnvironmentVariable, etc.),
        # execute normally to traverse their children
        try:
            return action.execute(context)
        except Exception as e:
            session.add_error(f"Error executing {type(action).__name__}: {e}")
            return None


def visit_node(
    node: Node, context: LaunchContext, session: BranchExplorerSession
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit a Node action and extract metadata without spawning process.

    This is adapted from launch2dump's visit_node but simplified for our needs.
    """
    # Perform substitutions to resolve placeholders
    node._perform_substitutions(context)

    def substitute(subst):
        """Helper to perform substitutions."""
        return perform_substitutions(context, normalize_to_list_of_substitutions(subst))

    # Extract basic information using name mangling
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

    # Get current condition
    condition_expr = session.get_current_condition()

    # Create metadata
    metadata = NodeMetadata(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=parameters,
        remappings=remappings,
        arguments=arguments,
        condition_expr=condition_expr,
    )

    session.add_node(metadata)

    # Don't spawn process
    return None


def extract_parameters(node: Node, context: LaunchContext) -> List[Dict[str, Any]]:
    """Extract parameters from node (simplified version)."""
    import os

    import yaml as pyyaml
    from launch_ros.descriptions import Parameter

    parameters = []
    node_params = node._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                # Parameter file
                param_file_path = str(entry)

                # Try to inline temporary files
                if param_file_path.startswith("/tmp/launch_params_"):
                    try:
                        if os.path.exists(param_file_path):
                            with open(param_file_path, "r") as f:
                                file_params = pyyaml.safe_load(f)
                                if file_params:
                                    parameters.append(file_params)
                                else:
                                    parameters.append({"__param_file": param_file_path})
                        else:
                            parameters.append({"__param_file": param_file_path})
                    except Exception:
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

                    # Handle nested parameter names
                    if isinstance(name, str) and "." in name:
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


def visit_include_launch_description(
    include: IncludeLaunchDescription,
    context: LaunchContext,
    session: BranchExplorerSession,
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Visit an include and extract metadata.

    Note: We don't recursively process includes yet (Phase 12.7).
    For now, we just record them.
    """
    # TODO: Phase 12.7 will implement recursive conversion
    # For now, just record the include

    try:
        # Try to get the file path
        launch_description_source = include.launch_description_source
        if hasattr(launch_description_source, "location"):
            file_path = Path(launch_description_source.location)
        else:
            # Can't determine file path
            session.add_error("Include with unknown file path")
            return None

        # Get launch arguments
        launch_arguments = {}
        if include.launch_arguments is not None:
            for name, value in include.launch_arguments:
                # Perform substitution
                name_str = perform_substitutions(context, normalize_to_list_of_substitutions(name))
                value_str = perform_substitutions(
                    context, normalize_to_list_of_substitutions(value)
                )
                launch_arguments[name_str] = value_str

        # Get current condition
        condition_expr = session.get_current_condition()

        # Create metadata
        metadata = IncludeMetadata(
            file_path=file_path, arguments=launch_arguments, condition_expr=condition_expr
        )

        session.add_include(metadata)

    except Exception as e:
        session.add_error(f"Error processing include: {e}")

    # Don't execute the include (we'll handle it in Phase 12.7)
    return None
