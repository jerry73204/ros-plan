# Launch2Plan: Launch-to-Plan Conversion Tool

This document describes the architecture, features, and usage of the `launch2plan` tool for converting ROS 2 launch files to ROS-Plan format.

## Overview

**launch2plan** is a conversion tool that transforms ROS 2 launch files (Python, XML, YAML) into ROS-Plan's declarative plan format. Unlike launch2dump which extracts metadata from a single execution path, launch2plan explores **all conditional branches** to generate a complete plan with `when` clauses.

### Design Principle: No Heuristics or Guessing

**launch2plan uses only two sources of information:**

1. **Launch file structure** - Explicit information (nodes, arguments, remappings, conditions)
2. **RMW introspection** - Actual node interfaces via `ros2-introspect`

**No heuristics or guessing.** If information cannot be determined from these two sources, the tool generates explicit TODO markers with helpful comments for manual completion. This ensures:

- ✓ **Correctness**: No incorrect guesses that need debugging later
- ✓ **Transparency**: Clear about what is known vs. unknown
- ✓ **Maintainability**: User corrections are precise and reusable

### Key Differences from launch2dump

| Aspect           | launch2dump                                   | launch2plan                                     |
|------------------|-----------------------------------------------|-------------------------------------------------|
| **Purpose**      | Extract metadata for a specific configuration | Convert to declarative plan format              |
| **Conditionals** | Evaluates and follows only True branches      | Explores ALL branches, generates `when` clauses |
| **Output**       | JSON/YAML metadata listing nodes              | Complete ROS-Plan YAML with links and sockets   |
| **Execution**    | Single execution path                         | Multiple hypothetical paths                     |
| **Use Case**     | Inspection, debugging, integration            | Migration, modernization                        |

## Architecture

### High-Level Components

```
┌─────────────────────┐
│  ROS 2 Launch File  │
│  (.launch.py, etc.) │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Launch Visitor     │
│  (Branch Explorer)  │
│                     │
│  ✓ Visit all nodes  │
│  ✓ Track conditions │
│  ✓ Detect cycles    │
│  ✓ Follow includes  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Socket Inference    │
│                     │
│  • RMW introspection│
│  • TODO generation  │
│  • Type inference   │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Plan Builder      │
│                     │
│  • Generate nodes   │
│  • Create links     │
│  • Handle includes  │
│  • Map arguments    │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  ROS-Plan YAML      │
│  + Metadata JSON    │
└─────────────────────┘
```

### Project Structure

```
launch2plan/
├── src/
│   └── launch2plan/
│       ├── __init__.py
│       ├── __main__.py              # CLI entry point
│       ├── visitor.py               # Branch exploration visitor
│       ├── introspection.py         # RMW introspection integration
│       ├── inference.py             # Socket/link inference
│       ├── builder.py               # Plan YAML construction
│       ├── metadata.py              # Conversion tracking
│       ├── arg_inference.py         # Argument type inference
│       └── statistics.py            # Stats calculation
├── tests/
│   ├── fixtures/                    # Test launch files
│   ├── test_visitor.py
│   ├── test_introspection.py
│   ├── test_inference.py
│   ├── test_builder.py
│   ├── test_metadata.py
│   ├── test_conditionals.py
│   └── test_includes.py
├── pyproject.toml
└── README.md
```

## Key Features

### Branch Exploration

Launch files use conditional execution, but we need to capture all possible configurations. launch2plan explores ALL branches without evaluation:

```python
# Standard approach (launch2dump)
def visit_action(action, context, session):
    condition = action.condition
    if condition is None or condition.evaluate(context):  # Only True branch
        return visit_action_by_class(action, context, session)
    return None

# Branch-exploring approach (launch2plan)
def visit_action(action, context, session):
    condition = action.condition

    # Track the condition expression
    condition_expr = extract_condition_expression(condition) if condition else None

    # Visit the action with condition context
    with session.condition_context(condition_expr):
        return visit_action_by_class(action, context, session)
```

**When Clause Generation**: Convert Python/LaunchConfiguration expressions to Lua:
- `IfCondition(LaunchConfiguration('use_sim_time'))` → `when: "$(use_sim_time)"`
- `UnlessCondition(...)` → `when: "$(not use_sim_time)"`

### RMW Introspection

Query actual node interfaces without running middleware:

```python
from ros2_introspect import introspect_node

class IntrospectionService:
    def __init__(self):
        self.cache = {}  # Cache introspection results

    def introspect_node_interfaces(self, package: str, executable: str):
        """Query node interfaces using rmw_introspect_cpp."""
        cache_key = f"{package}::{executable}"

        if cache_key in self.cache:
            return self.cache[cache_key]

        result = introspect_node(package, executable, timeout=3.0)

        if result.success:
            self.cache[cache_key] = result
            return result

        return None
```

**Introspection provides:**
- Socket directions (!pub vs !sub)
- Message types
- QoS settings

**When introspection fails**, generate TODO markers with helpful context.

### Socket and Link Inference

```python
# Introspection succeeds → complete socket info
node:
  camera:
    pkg: image_tools
    exec: cam2image
    socket:
      image: !pub  # Resolved via introspection

# Introspection fails → TODO marker
node:
  custom_node:
    pkg: custom_pkg
    exec: custom_exe
    socket:
      # TODO: Specify direction (!pub or !sub) and message type
      # Unable to introspect custom_pkg::custom_exe
      # Remapping: data -> /topic/name
      data: !todo

link:
  camera_image: !pubsub
    type: "sensor_msgs/msg/Image"  # From introspection
    src: ["camera/image"]
    dst: ["detector/image"]
```

### Argument Type Inference

Infer argument types from default values:

```python
# Launch: DeclareLaunchArgument("camera_fps", default_value="30")
# Plan:
arg:
  camera_fps:
    type: "i64"
    default: !i64 30
```

**Type inference rules:**
- `"true"/"false"` → `!bool`
- `"123"` → `!i64`
- `"3.14"` → `!f64`
- `"/path/to/file"` → `!str`
- No default → `!todo`

### Include Handling

Generate plan includes matching launch includes:

```python
# Launch include:
IncludeLaunchDescription(
    PythonLaunchDescriptionSource('camera.launch.py'),
    launch_arguments={'device': '/dev/video0', 'fps': '30'}.items()
)

# Plan include:
include:
  camera:
    file: "camera.plan.yaml"
    arg:
      device: "/dev/video0"
      fps: !i64 30
```

**Current Limitation**: Phases 1-8 generate a single monolithic plan file with all included content inlined. This does NOT preserve the modular structure of launch files. Phase 13 (Modular Plan Generation) will address this by generating separate plan files for each launch file with include references.

### Multi-Plan File Generation

**Objective**: Generate exactly one plan file for each launch file visited during conversion, maintaining **graph equivalence** between the launch file tree and the plan file tree.

**Design Principles**:
- **One plan per launch file** - Not per argument combination
- **Deduplication** - Same launch file → same plan file (regardless of how many times included)
- **Uniform treatment** - Root launch file handled identically to included launch files
- **Argument substitution** - Use `$(arg_name)` expressions instead of hardcoded values
- **Modular structure** - Preserve the hierarchical organization of launch files

#### Output File Naming Strategy

All launch files (including the root) follow the same naming rules based on their location:

**1. Package-based Launch Files**

Launch files located in a ROS 2 package's share directory:

```
Launch: /opt/ros/humble/share/nav2_bringup/launch/navigation.launch.py
Plan:   $outdir/nav2_bringup/launch/navigation.plan.yaml

Pattern: $outdir/{package_name}/{relative_path_within_share}.plan.yaml
```

**Detection**: Use `ament_index_python.packages` to determine if the absolute path belongs to a package share directory.

**2. Path-based Launch Files**

Launch files NOT in a ROS 2 package share directory:

```
Launch: /home/user/custom_launches/robot.launch.py
Plan:   $outdir/_path/home/user/custom_launches/robot.plan.yaml

Pattern: $outdir/_path{absolute_path_without_extension}.plan.yaml
```

The `_path/` prefix distinguishes path-based files from package-based files.

**3. User-specified Output File**

When using `-o output.plan.yaml`:
- Specified file is used ONLY for the root launch file
- All included files still follow package-based or path-based rules above

#### Include References in Generated Plans

Plan files use relative paths from `$outdir` and handle arguments via substitutions:

```yaml
# Package-based include
include:
  navigation:
    file: "nav2_bringup/launch/navigation.plan.yaml"
    arg:
      use_sim_time: $(use_sim_time)    # Substitution, not hardcoded
      map_file: $(map_path)

# Path-based include
include:
  custom_sensors:
    file: "_path/home/user/custom_launches/sensors.plan.yaml"
    arg:
      device: $(camera_device)
      fps: $(camera_fps)
```

**Same Plan File, Different Arguments**:

A single plan file can be included multiple times with different arguments. The plan file accepts arguments and uses them via substitutions:

```yaml
# sensors.plan.yaml - ONE plan file that accepts arguments
arg:
  device:
    type: "str"
  fps:
    type: "i64"
    default: !i64 30

node:
  camera:
    pkg: camera_driver
    exec: camera_node
    param:
      device_path: $(device)    # Uses argument
      frame_rate: $(fps)         # Uses argument

# Parent plan can include it multiple times with different values
include:
  front_camera:
    file: "sensors.plan.yaml"
    arg:
      device: "/dev/video0"
      fps: !i64 30

  rear_camera:
    file: "sensors.plan.yaml"    # Same file!
    arg:
      device: "/dev/video1"      # Different arguments
      fps: !i64 15
```

**Key Features**:
- File paths are relative to `$outdir` for portability
- Arguments use substitutions (`$(arg_name)`) to support different values
- Same plan file can be included multiple times with different argument values
- No file duplication - one launch file → one plan file, always

#### Implementation Requirements

1. **Package Detection**
   - Use `ament_index_python.packages.get_package_share_directory()` to map paths to packages
   - Extract package name and relative path within share directory

2. **Deduplication Tracking**
   - Track visited launch files by absolute path
   - Map absolute path → output plan file path
   - Skip regeneration if already visited

3. **Output Path Generation**
   - Detect package membership from absolute path
   - Generate appropriate output path (package-based vs path-based)
   - Create necessary subdirectories

4. **Include Reference Generation**
   - Convert absolute paths to relative paths from `$outdir`
   - Preserve argument mappings with substitutions
   - Handle nested includes correctly

5. **Root File Handling**
   - Apply same package/path detection as included files
   - Override with `-o` option if specified
   - Update metadata to track all generated files

#### Graph Equivalence

The plan file tree structure exactly mirrors the launch file tree with a strict **1-to-1 mapping**:

```
Launch File Tree:                Plan File Tree:
robot.launch.py                  robot.plan.yaml
├─ nav2_bringup/                 ├─ nav2_bringup/
│  └─ navigation.launch.py       │  └─ navigation.plan.yaml
├─ sensors/                      ├─ sensors/
│  ├─ camera.launch.py           │  ├─ camera.plan.yaml
│  └─ lidar.launch.py            │  └─ lidar.plan.yaml
└─ custom/                       └─ _path/home/user/custom/
   └─ special.launch.py             └─ special.plan.yaml
```

**Core Principle**: One launch file → one plan file (regardless of how many times included or with what arguments).

**Example - Multiple Inclusions**:
```
Launch Structure:
  robot.launch.py includes sensor.launch.py twice:
    - Once with device="/dev/video0"
    - Once with device="/dev/video1"

Generated Plans:
  robot.plan.yaml          (includes sensor.plan.yaml twice with different args)
  sensor.plan.yaml         (single file, accepts $(device) argument)
```

**Benefits**:
- ✓ Preserves modular structure of original launch system
- ✓ Allows independent editing of plan files
- ✓ Supports reuse of plan files with different argument values
- ✓ Maintains clear 1-to-1 mapping for easier migration
- ✓ No duplication - each launch file generates exactly one plan file
- ✓ Arguments handled at include-time via substitutions, not via file variations

### Metadata Tracking

Track conversion state for transparency:

```json
{
  "source_file": "/path/to/robot.launch.py",
  "source_hash": "sha256:1234abcd...",
  "generated_at": "2025-10-16T14:30:00Z",
  "converter_version": "0.1.0",

  "todos": [
    {
      "location": "node.camera.socket.image_raw",
      "field": "direction",
      "current_value": "!todo",
      "status": "pending",
      "context": {
        "node_package": "sensor_pkg",
        "node_executable": "camera_node",
        "remapping": ["image_raw", "/camera/image"],
        "reason": "introspection_failed",
        "hint": "Specify !pub if camera publishes images"
      }
    }
  ],

  "stats": {
    "total_nodes": 2,
    "total_includes": 1,
    "total_links": 1,
    "total_arguments": 3,
    "total_todos": 2,
    "pending_todos": 2,
    "completed_todos": 0,
    "nodes_introspected": 1,
    "nodes_failed_introspection": 1,
    "completion_rate": 0.0
  }
}
```

## Usage

### Installation

```bash
# Install dependencies
cd python/launch2plan
uv sync

# Ensure ROS 2 and workspace are sourced
source /opt/ros/humble/setup.bash
source install/setup.bash  # For rmw_introspect_cpp
```

### Basic Conversion

```bash
# Simple conversion
cd python/launch2plan
uv run launch2plan convert robot.launch.py

# With launch arguments
uv run launch2plan convert robot.launch.py \
    map_path:=/path/to/map \
    vehicle_model:=sample_vehicle

# With output file
uv run launch2plan convert robot.launch.py -o output.plan.yaml
```

**Output:**
- `robot.plan.yaml` - Generated plan file
- `robot.plan.meta.json` - Metadata with TODO tracking

### Checking Status

```bash
# Show conversion status
uv run launch2plan status robot.plan.yaml
```

**Output:**
```
Conversion Status: robot.plan.yaml
Source: /path/to/robot.launch.py
Generated: 2025-10-25T10:30:00Z

Structure:
  Nodes:     60
  Includes:  68
  Links:     45
  Arguments: 12

TODO Status:
  Total:     148
  Pending:   148
  Completed: 0
  Progress:  0.0%

Introspection:
  Nodes introspected:         45
  Nodes failed introspection: 15
  Sockets from introspection: 120
  Sockets requiring input:    28
```

### Completing TODOs

1. **Review generated plan** - Check for `!todo` markers and `TODO` message types
2. **Fill in missing information** - Edit plan YAML to replace TODOs
3. **Re-run status** - Verify TODOs are marked completed
4. **Validate plan** - Compile with `ros2plan compile`

Example TODO completion:

```yaml
# Before:
node:
  custom_node:
    socket:
      data: !todo  # TODO: Specify !pub or !sub

# After:
node:
  custom_node:
    socket:
      data: !pub  # User specified direction
```

### Testing Your Conversion

```bash
# Validate the generated plan compiles
ros2plan compile robot.plan.yaml

# If successful, test run
ros2plan run robot.plan.yaml
```

## Implementation Status

**Current Phase**: Phase 8 complete (Metadata Tracking)

**Completed Features** (85 tests passing):
- ✅ Phase 1: Foundation & Basic Visitor (3 tests)
- ✅ Phase 2: RMW Introspection Integration (5 tests)
- ✅ Phase 3: Socket Inference & TODO Generation (6 tests)
- ✅ Phase 4: Plan Builder & Link Generation (10 tests)
- ✅ Phase 5: Argument & Parameter Conversion (26 tests)
- ✅ Phase 6: Conditional Branch Exploration (18 tests)
- ✅ Phase 7: Include Handling & Plan Hierarchy (8 tests)
- ✅ Phase 8: Metadata Tracking (13 tests)

**Next Steps**: See roadmap.md for future phases (9-13)

## Current Limitations

1. **Single-File Output** (Phases 1-8): Current implementation generates one monolithic plan file with all included content inlined. Multi-plan file generation (see "Multi-Plan File Generation" section above) is designed but not yet implemented. Phase 13 will implement this feature.
2. **Dynamic Behavior**: Cannot analyze `OpaqueFunction` or runtime-evaluated code (generates TODO markers)
3. **Introspection Failures**: Nodes that cannot be introspected require manual TODO completion
4. **Complex Conditions**: Some Python expressions may not convert cleanly to Lua (marked as TODO)
5. **Custom Packages**: Nodes from packages not available in the environment require manual specification

## Testing

```bash
# Run all tests
cd python/launch2plan
uv run pytest

# Run specific test module
uv run pytest tests/test_visitor.py

# Run with coverage
uv run pytest --cov=launch2plan

# Test with Autoware (requires Autoware workspace)
./scripts/test_autoware_launch2plan.sh
```

## Troubleshooting

### Introspection Failures

**Problem**: Many nodes show introspection failures

**Solutions:**
1. Ensure ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
2. Build and source workspace: `make build && source install/setup.bash`
3. Check package availability: `ros2 pkg list | grep <package>`
4. Some nodes may genuinely not be introspectable (use TODO completion)

### Conversion Takes Too Long

**Problem**: Script times out or takes several minutes

**Cause**: Introspecting many nodes (60+) can take 3-5 minutes

**Solutions:**
1. This is expected for large launch files like Autoware
2. Consider converting smaller launch file subsets first
3. Introspection results are cached for subsequent conversions

### Generated Plan Won't Compile

**Problem**: `ros2plan compile` fails on generated plan

**Solutions:**
1. Check for remaining TODOs: `uv run launch2plan status <plan-file>`
2. Complete all pending TODOs before compiling
3. Verify message types are correct
4. Check that `!pub`/`!sub` directions match actual node behavior

## References

- **ros2-introspect**: RMW introspection tool (`python/ros2-introspect`)
- **rmw_introspect_cpp**: Custom RMW implementation (`external/rmw_introspect`)
- **launch2dump**: Node metadata extraction tool
- **ROS-Plan Format**: Plan YAML specification (`book/src/plan-format.md`)
- **Roadmap**: Implementation progress tracking (`book/src/roadmap.md`)
