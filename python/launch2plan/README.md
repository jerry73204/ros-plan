# launch2plan

Convert ROS 2 launch files to ROS-Plan format.

## Status: Active Development

This tool is in active development. Core infrastructure for visiting launch files, introspecting nodes, and generating plan YAML is operational.

### Design Principles

1. **No Heuristics**: Only uses launch file structure and RMW introspection
2. **Explicit TODOs**: Unknown information becomes clear TODO markers
3. **Multi-Pass Workflow**: Learn from user corrections across conversions
4. **Always Validate**: Generated plans must compile with ros2plan

## Usage (Planned)

```bash
# Convert launch file to plan
source /opt/ros/humble/setup.bash
source /path/to/workspace/install/setup.bash
launch2plan convert robot.launch.py

# Validate generated plan
launch2plan validate robot.plan.yaml

# Check TODO completion status
launch2plan status robot.plan.yaml
```

## Dependencies

### Python Dependencies
- `ros2-introspect`: RMW introspection for node interface discovery (from `external/rmw_introspect/`)
- `ruamel-yaml`: YAML generation with proper formatting
- `pyyaml`: YAML parsing
- `lark`: Expression parsing

### ROS 2 Dependencies (Runtime)
- `rmw_introspect_cpp`: Custom RMW implementation for lightweight node introspection
  - Located in `external/rmw_introspect/rmw_introspect_cpp/`
  - Built via `make build` (symlinked in `ros2/`)
  - Must be sourced before running launch2plan

## Setup

```bash
# 1. Build rmw_introspect_cpp (required for introspection)
make build

# 2. Source the workspace
source install/setup.bash

# 3. Install Python dependencies
cd python/launch2plan
uv sync
```

## Development

```bash
# Run tests (requires workspace sourced)
source install/setup.bash
cd python/launch2plan
uv run pytest
```

## References

See `book/src/launch2plan.md` for complete design documentation.
