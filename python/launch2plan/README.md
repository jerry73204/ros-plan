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

- `ros2-introspect`: RMW introspection for node interface discovery
- `launch2dump`: Launch file visitor infrastructure (reused)
- `ros2plan`: Plan compiler for validation

## Development

```bash
cd python/launch2plan
uv sync
uv run pytest
```

## References

See `book/src/launch2plan.md` for complete design documentation.
