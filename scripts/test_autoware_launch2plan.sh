#!/usr/bin/env bash
#
# Test launch2plan with Autoware Planning Simulator
#
# Prerequisites:
# - Autoware symlink configured: scripts/autoware -> /path/to/autoware/workspace
# - Autoware sample maps downloaded to ~/autoware_map/sample-map-planning
# - ROS 2 workspace built and sourced (for rmw_introspect_cpp)

set -e

# Cleanup function to kill orphaned ROS nodes from introspection
cleanup_orphaned_nodes() {
    echo "Cleaning up orphaned ROS nodes from previous runs..."
    local count=$(pkill -9 -f "autoware.*--ros-args" 2>/dev/null | wc -l || echo 0)
    if [ $count -gt 0 ]; then
        echo "Killed $count orphaned nodes"
    fi
}

# Trap to ensure cleanup on exit
trap cleanup_orphaned_nodes EXIT

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
OUTPUT_DIR="$SCRIPT_DIR/autoware/output"

# Clean up any existing orphaned nodes before starting
cleanup_orphaned_nodes

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Test with Autoware planning simulator
LAUNCH_FILE="$AUTOWARE_WS/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml"

if [ ! -f "$LAUNCH_FILE" ]; then
    echo "Error: Launch file not found: $LAUNCH_FILE"
    echo "Please check your Autoware installation"
    exit 1
fi

echo "NOTE: This conversion may take several minutes due to introspection of many nodes."
echo ""

(cd "$REPO_ROOT/python/launch2plan" && uv run launch2plan convert \
    "$LAUNCH_FILE" \
    map_path:="$MAP_PATH" \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    --output-dir "$OUTPUT_DIR")

echo ""
echo "=== Conversion Complete ==="
echo "Output directory: $OUTPUT_DIR"
echo ""

# Count generated files
PLAN_COUNT=$(find "$OUTPUT_DIR" -name "*.plan.yaml" -type f | wc -l)
echo "Generated $PLAN_COUNT plan files"
echo ""

# Display generated files
echo "=== Generated Plan Files ==="
find "$OUTPUT_DIR" -name "*.plan.yaml" -type f | sort
