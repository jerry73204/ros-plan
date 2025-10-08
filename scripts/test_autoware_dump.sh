#!/usr/bin/env bash
#
# Test launch2dump with Autoware Planning Simulator
#
# Prerequisites:
# - Autoware installed and sourced
# - Autoware sample maps downloaded to ~/autoware_map/sample-map-planning

set -e

# Determine script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Check environment
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "Error: ROS 2 environment not sourced" >&2
    exit 1
fi

# Configuration
MAP_PATH="${HOME}/autoware_map/sample-map-planning"
OUTPUT_FILE="$SCRIPT_DIR/autoware_planning_simulator.json"

# Test with Autoware planning simulator
(cd "$REPO_ROOT/launch2dump" && uv run launch2dump \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$MAP_PATH" \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    -f json \
    -o "$OUTPUT_FILE")

echo "Output: $OUTPUT_FILE"
