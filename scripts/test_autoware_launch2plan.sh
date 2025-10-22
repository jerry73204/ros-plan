#!/usr/bin/env bash
#
# Test launch2plan with Autoware Planning Simulator
#
# Prerequisites:
# - Autoware symlink configured: scripts/autoware -> /path/to/autoware/workspace
# - Autoware sample maps downloaded to ~/autoware_map/sample-map-planning
# - ROS 2 workspace built and sourced (for rmw_introspect_cpp)

set -e

# Determine script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
AUTOWARE_WS="$SCRIPT_DIR/autoware"

# Check if Autoware symlink exists
if [ ! -L "$AUTOWARE_WS" ]; then
    echo "Error: Autoware symlink not configured"
    echo "Please create symlink: ln -s /path/to/autoware/workspace scripts/autoware"
    exit 1
fi

# Source Autoware
source "$AUTOWARE_WS/install/setup.bash"

# Source ROS 2 workspace (for rmw_introspect_cpp)
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
    source "$REPO_ROOT/install/setup.bash"
else
    echo "Warning: ROS 2 workspace not built. Run 'make build' first."
    echo "launch2plan will work but introspection will be limited."
fi

# Configuration
MAP_PATH="${HOME}/autoware_map/sample-map-planning"
OUTPUT_DIR="$SCRIPT_DIR/autoware_output"
OUTPUT_FILE="$OUTPUT_DIR/planning_simulator.plan.yaml"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Test with Autoware planning simulator
LAUNCH_FILE="$AUTOWARE_WS/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml"

if [ ! -f "$LAUNCH_FILE" ]; then
    echo "Error: Launch file not found: $LAUNCH_FILE"
    echo "Please check your Autoware installation"
    exit 1
fi

(cd "$REPO_ROOT/python/launch2plan" && uv run launch2plan convert \
    "$LAUNCH_FILE" \
    -o "$OUTPUT_FILE" \
    -- \
    map_path:="$MAP_PATH" \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit)

echo ""
echo "=== Conversion Complete ==="
echo "Plan file: $OUTPUT_FILE"
echo "Metadata:  ${OUTPUT_FILE%.yaml}.meta.json"
echo ""

# Display summary if metadata exists
META_FILE="${OUTPUT_FILE%.plan.yaml}.plan.meta.json"
if [ -f "$META_FILE" ]; then
    echo "=== Metadata Summary ==="
    python3 -c "
import json
import sys

with open('$META_FILE') as f:
    meta = json.load(f)

stats = meta.get('stats', {})
print(f\"Nodes:     {stats.get('total_nodes', 0)}\")
print(f\"Links:     {stats.get('total_links', 0)}\")
print(f\"Arguments: {stats.get('total_arguments', 0)}\")
print(f\"Includes:  {stats.get('total_includes', 0)}\")
print(f\"TODOs:     {stats.get('total_todos', 0)} ({stats.get('pending_todos', 0)} pending, {stats.get('completed_todos', 0)} completed)\")
print(f\"Completion: {stats.get('completion_rate', 0)*100:.1f}%\")
"
fi
