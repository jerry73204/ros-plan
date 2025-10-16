"""Metadata tracking for launch-to-plan conversion.

This module provides data structures and operations for tracking conversion
state, TODOs, and statistics across the conversion process.
"""

import hashlib
import json
import logging
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ruamel.yaml import YAML

# Version of the metadata format
METADATA_VERSION = "0.1.0"


# ============================================================================
# Enums
# ============================================================================


class TodoStatus(Enum):
    """Status of a TODO item."""

    PENDING = "pending"  # Not yet resolved
    COMPLETED = "completed"  # User manually completed


class TodoReason(Enum):
    """Reason why a TODO was created."""

    INTROSPECTION_FAILED = "introspection_failed"  # ros2-introspect failed
    SOCKET_NOT_FOUND = "socket_not_found"  # Socket not in introspection results
    DYNAMIC_CODE = "dynamic_code"  # OpaqueFunction or runtime code
    MISSING_PACKAGE = "missing_package"  # Package not available


# ============================================================================
# Data Structures
# ============================================================================


@dataclass
class TodoContext:
    """Context information to help user complete TODO."""

    # What we know
    node_package: Optional[str] = None
    node_executable: Optional[str] = None
    remapping: Optional[Tuple[str, str]] = None  # (from, to)

    # Why it's unknown
    reason: Optional[str] = None  # TodoReason enum value
    error_message: Optional[str] = None  # Detailed error if available

    # Helpful hint
    hint: Optional[str] = None  # Human-readable suggestion for completion


@dataclass
class TodoItem:
    """A single TODO marker in the generated plan."""

    # Location in plan
    location: str  # JSONPath-style: "node.camera.socket.image"
    field: str  # "direction" | "message_type" | "arg_type"

    # Current state
    current_value: str  # "!todo" or "TODO"
    status: str  # TodoStatus enum value

    # Context for user
    context: TodoContext


@dataclass
class NodeSource:
    """Track where a node came from in the launch file."""

    launch_file: str  # Path to launch file
    line_number: Optional[int] = None  # Line number in launch file (if available)
    include_path: Optional[List[str]] = None  # Chain of includes leading to this node
    condition: Optional[str] = None  # Condition expression (if any)


@dataclass
class ConversionStats:
    """Statistics about the conversion."""

    # Counts
    total_nodes: int = 0
    total_includes: int = 0
    total_links: int = 0
    total_arguments: int = 0

    # TODO tracking
    total_todos: int = 0
    pending_todos: int = 0
    completed_todos: int = 0

    # Introspection results
    nodes_introspected: int = 0  # Nodes successfully introspected
    nodes_failed_introspection: int = 0  # Nodes where introspection failed
    sockets_from_introspection: int = 0  # Sockets resolved via introspection
    sockets_requiring_user_input: int = 0  # Sockets marked as TODO

    # Completion rate
    completion_rate: float = 0.0  # TODOs completed / total TODOs

    # Performance
    introspection_time_ms: int = 0
    conversion_time_ms: int = 0


@dataclass
class ConversionMetadata:
    """Complete metadata for a plan conversion."""

    # Source information
    source_file: str  # Original launch file path
    source_hash: str  # SHA256 hash for change detection
    generated_at: str  # ISO 8601 timestamp
    converter_version: str  # launch2plan version

    # TODO tracking
    todos: List[TodoItem]

    # Conversion statistics
    stats: ConversionStats

    # Node-to-source mapping (for debugging)
    node_sources: Dict[str, NodeSource]  # node_id -> source info


@dataclass
class DiscoveredTodo:
    """A TODO marker found in the current plan."""

    location: str
    field: str
    current_value: str


@dataclass
class CompletedTodo:
    """Record of a TODO that was completed by user."""

    location: str
    field: str
    old_value: str  # Original TODO marker
    new_value: str  # User-provided value


# ============================================================================
# Metadata Manager (F68: Persistence)
# ============================================================================


class MetadataManager:
    """Manage metadata persistence."""

    def save_metadata(self, metadata: ConversionMetadata, plan_path: Path):
        """Save metadata as JSON alongside plan file."""
        meta_path = plan_path.with_suffix(".plan.meta.json")

        # Convert dataclasses to dict
        data = asdict(metadata)

        # Serialize to JSON with pretty printing
        json_str = json.dumps(data, indent=2, sort_keys=True, ensure_ascii=False)

        meta_path.write_text(json_str)
        logging.info(f"Saved metadata to {meta_path}")

    def load_metadata(self, plan_path: Path) -> Optional[ConversionMetadata]:
        """Load metadata from JSON file."""
        meta_path = plan_path.with_suffix(".plan.meta.json")

        if not meta_path.exists():
            logging.debug(f"Metadata file not found: {meta_path}")
            return None

        try:
            data = json.loads(meta_path.read_text())

            # Validate schema version if present
            if "converter_version" in data:
                self._validate_version(data["converter_version"])

            # Reconstruct dataclasses
            metadata = self._dict_to_metadata(data)
            logging.info(f"Loaded metadata from {meta_path}")
            return metadata

        except (json.JSONDecodeError, TypeError, KeyError) as e:
            logging.error(f"Failed to load metadata: {e}")
            return None

    def _validate_version(self, version: str):
        """Validate metadata format version."""
        if version != METADATA_VERSION:
            logging.warning(
                f"Metadata version mismatch: expected {METADATA_VERSION}, got {version}"
            )

    def _dict_to_metadata(self, data: Dict) -> ConversionMetadata:
        """Convert dict back to ConversionMetadata dataclass."""
        # Convert todos
        todos = [
            TodoItem(
                location=t["location"],
                field=t["field"],
                current_value=t["current_value"],
                status=t["status"],
                context=TodoContext(**t["context"]),
            )
            for t in data.get("todos", [])
        ]

        # Convert stats
        stats = ConversionStats(**data.get("stats", {}))

        # Convert node_sources
        node_sources = {
            node_id: NodeSource(**source_data)
            for node_id, source_data in data.get("node_sources", {}).items()
        }

        return ConversionMetadata(
            source_file=data["source_file"],
            source_hash=data["source_hash"],
            generated_at=data["generated_at"],
            converter_version=data["converter_version"],
            todos=todos,
            stats=stats,
            node_sources=node_sources,
        )


# ============================================================================
# Plan Parser (F69: YAML Parsing & TODO Discovery)
# ============================================================================


class PlanParser:
    """Parse plan YAML files and discover TODOs."""

    def parse_plan_yaml(self, plan_path: Path) -> Dict:
        """Load and parse plan YAML file."""
        yaml = YAML()
        yaml.preserve_quotes = True
        with plan_path.open() as f:
            return yaml.load(f)

    def find_todos_in_plan(self, plan_data: Dict) -> List[DiscoveredTodo]:
        """
        Scan plan YAML structure for TODO markers.

        Returns list of TODOs found in the current plan state.
        """
        todos = []

        # Scan node sockets
        if "node" in plan_data:
            for node_id, node_def in plan_data["node"].items():
                if "socket" in node_def and isinstance(node_def["socket"], dict):
                    for socket_name, socket_value in node_def["socket"].items():
                        # Check for !todo tag
                        if socket_value == "!todo" or (
                            isinstance(socket_value, str) and socket_value == "!todo"
                        ):
                            todos.append(
                                DiscoveredTodo(
                                    location=f"node.{node_id}.socket.{socket_name}",
                                    field="direction",
                                    current_value="!todo",
                                )
                            )

        # Scan link message types
        if "link" in plan_data:
            for link_id, link_def in plan_data["link"].items():
                if isinstance(link_def, dict) and link_def.get("type") == "TODO":
                    todos.append(
                        DiscoveredTodo(
                            location=f"link.{link_id}.type",
                            field="message_type",
                            current_value="TODO",
                        )
                    )

        # Scan argument types
        if "arg" in plan_data:
            for arg_name, arg_def in plan_data["arg"].items():
                if isinstance(arg_def, dict):
                    if arg_def.get("type") == "!todo":
                        todos.append(
                            DiscoveredTodo(
                                location=f"arg.{arg_name}.type",
                                field="arg_type",
                                current_value="!todo",
                            )
                        )

        return todos

    def get_value_at_path(self, plan_data: Dict, location: str) -> Any:
        """
        Navigate plan structure using JSONPath-style location.

        Example: "node.camera.socket.image" -> plan_data['node']['camera']['socket']['image']
        """
        parts = location.split(".")
        current = plan_data

        for part in parts:
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]

        return current


# ============================================================================
# TODO Status Updater (F70: Status Update)
# ============================================================================


class TodoStatusUpdater:
    """Detect and update TODO completion status."""

    def __init__(self):
        self.parser = PlanParser()

    def detect_completed_todos(
        self, metadata: ConversionMetadata, plan_data: Dict
    ) -> List[CompletedTodo]:
        """
        Compare metadata TODOs against current plan to find user edits.

        Algorithm:
        1. For each TODO in metadata with status=PENDING
        2. Get current value at TODO location in plan
        3. If current value differs from metadata value AND is not a TODO marker
        4. Mark as COMPLETED and record the new value
        """
        completed = []

        for todo in metadata.todos:
            if todo.status != TodoStatus.PENDING.value:
                continue  # Already completed

            # Get current value from plan
            current_value = self.parser.get_value_at_path(plan_data, todo.location)

            # Check if user filled in the TODO
            if (
                current_value is not None
                and current_value != todo.current_value
                and current_value not in ["!todo", "TODO"]
            ):
                completed.append(
                    CompletedTodo(
                        location=todo.location,
                        field=todo.field,
                        old_value=todo.current_value,
                        new_value=str(current_value),
                    )
                )

                # Update status in metadata
                todo.status = TodoStatus.COMPLETED.value

        return completed

    def update_metadata_from_plan(
        self, metadata: ConversionMetadata, plan_path: Path
    ) -> ConversionMetadata:
        """
        Update metadata by comparing against current plan state.

        This is called when user runs `launch2plan status` to see progress.
        """
        plan_data = self.parser.parse_plan_yaml(plan_path)

        # Detect completed TODOs
        completed = self.detect_completed_todos(metadata, plan_data)

        if completed:
            logging.info(f"Detected {len(completed)} completed TODOs")
            for c in completed:
                logging.debug(f"  {c.location}: {c.old_value} -> {c.new_value}")

        # Recalculate statistics
        from launch2plan.statistics import calculate_stats

        metadata.stats = calculate_stats(metadata, plan_data)

        return metadata

    def check_metadata_staleness(
        self, metadata: ConversionMetadata, source_path: Path
    ) -> Optional[str]:
        """
        Check if source file has changed since conversion.

        Returns warning message if stale, None if up-to-date.
        """
        # Calculate current source hash
        current_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()

        if current_hash != metadata.source_hash:
            return (
                f"Warning: Source file has changed since conversion.\n"
                f"  Expected: {metadata.source_hash[:8]}...\n"
                f"  Current:  {current_hash[:8]}...\n"
                f"Consider re-running conversion to ensure plan is up-to-date."
            )

        return None
