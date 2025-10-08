"""
Session for collecting node information during launch inspection
"""

from dataclasses import dataclass, field
from typing import List, Set

from ..result import ContainerInfo, NodeInfo


@dataclass
class CollectionSession:
    """Session that collects nodes, containers, and dependencies during inspection."""

    nodes: List[NodeInfo] = field(default_factory=list)
    containers: List[ContainerInfo] = field(default_factory=list)
    lifecycle_nodes: List[NodeInfo] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    parameter_dependencies: Set[str] = field(default_factory=set)

    def add_node(self, node: NodeInfo) -> None:
        """Add a node to the session."""
        self.nodes.append(node)

    def add_container(self, container: ContainerInfo) -> None:
        """Add a container to the session."""
        self.containers.append(container)

    def add_lifecycle_node(self, node: NodeInfo) -> None:
        """Add a lifecycle node to the session."""
        self.lifecycle_nodes.append(node)

    def add_error(self, error: str) -> None:
        """Add an error message to the session."""
        self.errors.append(error)

    def add_parameter_dependency(self, param_name: str) -> None:
        """Add a parameter dependency."""
        self.parameter_dependencies.add(param_name)
