"""
Data structures for launch file loading results
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple


@dataclass
class NodeInfo:
    """Metadata for a regular ROS 2 node."""

    package: str
    executable: str
    name: Optional[str] = None
    namespace: Optional[str] = None
    parameters: List[Dict[str, Any]] = field(default_factory=list)
    remappings: List[Tuple[str, str]] = field(default_factory=list)
    arguments: List[str] = field(default_factory=list)
    env_vars: Dict[str, str] = field(default_factory=dict)


@dataclass
class ComposableNodeInfo:
    """Metadata for a composable node."""

    plugin: str
    name: str
    namespace: Optional[str] = None
    parameters: List[Dict[str, Any]] = field(default_factory=list)
    remappings: List[Tuple[str, str]] = field(default_factory=list)
    extra_arguments: List[str] = field(default_factory=list)


@dataclass
class ContainerInfo:
    """Metadata for a composable node container."""

    package: str
    executable: str
    name: str
    namespace: Optional[str] = None
    composable_nodes: List[ComposableNodeInfo] = field(default_factory=list)


@dataclass
class LaunchResult:
    """Result of loading a launch file."""

    nodes: List[NodeInfo] = field(default_factory=list)
    containers: List[ContainerInfo] = field(default_factory=list)
    lifecycle_nodes: List[NodeInfo] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    parameter_dependencies: Set[str] = field(default_factory=set)
