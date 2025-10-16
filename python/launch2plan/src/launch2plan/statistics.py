"""Statistics calculation for launch-to-plan conversion (F71)."""

from typing import Dict

from launch2plan.metadata import ConversionMetadata, ConversionStats, TodoStatus


def calculate_stats(metadata: ConversionMetadata, plan_data: Dict) -> ConversionStats:
    """
    Calculate conversion statistics from metadata and plan data.

    Args:
        metadata: Current conversion metadata
        plan_data: Parsed plan YAML data

    Returns:
        Updated ConversionStats
    """
    stats = ConversionStats()

    # Count structural elements
    if "node" in plan_data:
        stats.total_nodes = len(plan_data["node"])

    if "include" in plan_data:
        stats.total_includes = len(plan_data["include"])

    if "link" in plan_data:
        stats.total_links = len(plan_data["link"])

    if "arg" in plan_data:
        stats.total_arguments = len(plan_data["arg"])

    # Count TODO status
    stats.total_todos = len(metadata.todos)
    stats.pending_todos = sum(
        1 for todo in metadata.todos if todo.status == TodoStatus.PENDING.value
    )
    stats.completed_todos = sum(
        1 for todo in metadata.todos if todo.status == TodoStatus.COMPLETED.value
    )

    # Calculate completion rate
    if stats.total_todos > 0:
        stats.completion_rate = stats.completed_todos / stats.total_todos
    else:
        stats.completion_rate = 1.0  # No TODOs means 100% complete

    # Note: introspection stats and performance metrics are set during conversion
    # and carried over from the original metadata
    if hasattr(metadata.stats, "nodes_introspected"):
        stats.nodes_introspected = metadata.stats.nodes_introspected
        stats.nodes_failed_introspection = metadata.stats.nodes_failed_introspection
        stats.sockets_from_introspection = metadata.stats.sockets_from_introspection
        stats.sockets_requiring_user_input = metadata.stats.sockets_requiring_user_input
        stats.introspection_time_ms = metadata.stats.introspection_time_ms
        stats.conversion_time_ms = metadata.stats.conversion_time_ms

    return stats
