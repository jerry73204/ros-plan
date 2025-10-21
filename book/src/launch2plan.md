# Launch2Plan: Launch-to-Plan Conversion Tool

This document describes the design and strategy for converting ROS 2 launch files to ROS-Plan format using the `launch2plan` tool.

## Overview

**launch2plan** is a conversion tool that transforms ROS 2 launch files (Python, XML, YAML) into ROS-Plan's declarative plan format. Unlike launch2dump which extracts metadata from a single execution path, launch2plan explores **all conditional branches** to generate a complete plan with `when` clauses.

The tool uses a **multi-pass workflow** where it generates skeleton plans with TODO markers, the user fills in missing information, and subsequent passes learn from user annotations to complete remaining TODOs automatically.

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

## Multi-Pass Workflow

### Core Principles

1. **No depth limits** - Handle arbitrarily complex nested conditionals and includes
2. **Multi-pass workflow** - Iterative refinement with user input
3. **Preserve structure** - Generate plan includes matching launch includes
4. **Always validate** - Compile after generation to catch errors early
5. **TODO-driven** - Leave clear markers for manual completion with helpful hints

### Workflow Stages

```
┌──────────────────────────────────────────────────────────────────┐
│ Conversion: Launch to Plan                                        │
│ • Explore all branches (no depth limit, cycle detection)         │
│ • Use rmw_introspect to discover node interfaces                 │
│ • Generate plan with sockets/links from introspection            │
│ • Mark unknowns as TODO with helpful context                     │
│ • Save metadata for tracking                                     │
└────────────────┬─────────────────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│ User Completion                                                   │
│ • Review generated plan and TODO markers                         │
│ • Fill in socket directions (!pub or !sub) for failed introspect│
│ • Specify message types where introspection unavailable          │
│ • Verify when clauses and conditions                             │
│ • Adjust node parameters and remappings as needed                │
└────────────────┬─────────────────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│ Validation                                                        │
│ • Compile plan with ros2plan                                     │
│ • Report compilation errors                                      │
│ • Iterate if needed                                              │
└──────────────────────────────────────────────────────────────────┘
```

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
│  • Heuristic hints  │
│  • Pattern learning │
│  • Confidence score │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Plan Builder      │
│                     │
│  • Generate nodes   │
│  • Create TODOs     │
│  • Preserve includes│
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
│       ├── inference.py             # Socket/link inference + pattern learning
│       ├── builder.py               # Plan YAML construction
│       ├── metadata.py              # Multi-pass state tracking
│       └── validator.py             # Plan compilation validation
├── tests/
│   ├── fixtures/                    # Test launch files
│   ├── test_visitor.py
│   ├── test_introspection.py
│   ├── test_inference.py
│   ├── test_builder.py
│   └── test_cli.py
├── pyproject.toml
└── README.md
```

## Feature Breakdown

### F61: Branch-Exploring Launch Visitor

**Challenge**: ROS 2 launch files use conditional execution, but we need to capture all possible configurations.

**Approach**: Modified visitor that explores ALL branches instead of evaluating conditions:

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

```python
# Input: IfCondition(LaunchConfiguration('use_sim_time'))
# Output: when: "$(use_sim_time)"

# Input: UnlessCondition(...)
# Output: when: "$(not use_sim_time)"
```

**Cycle Detection**: Track include stack to prevent infinite recursion:

```python
class VisitorSession:
    def __init__(self):
        self.include_stack = []
        self.visited_files = set()

    def check_cycle(self, file_path):
        return file_path in self.include_stack
```

### F62: RMW Introspection for Accurate Inference

**Challenge**: Launch files don't specify socket directions (!pub vs !sub) or message types.

**Approach**: Use `ros2-introspect` to query actual node interfaces without running middleware. No heuristics or guessing - if information cannot be determined from launch files or introspection, generate TODO markers for manual completion.

```python
from ros2_introspect import introspect_node

class IntrospectionService:
    def __init__(self):
        self.cache = {}  # Cache introspection results

    def introspect_node_interfaces(self, package: str, executable: str) -> IntrospectionResult:
        """Query node interfaces using rmw_introspect_cpp."""
        cache_key = f"{package}::{executable}"

        if cache_key in self.cache:
            return self.cache[cache_key]

        # Run introspection (requires sourced ROS 2 environment)
        result = introspect_node(package, executable, timeout=3.0)

        if result.success:
            self.cache[cache_key] = result
            return result

        return None

    def get_socket_info(self, package: str, executable: str, topic_name: str):
        """Get socket direction and message type for a topic."""
        result = self.introspect_node_interfaces(package, executable)

        if not result:
            return None

        # Check publishers
        for pub in result.publishers:
            if pub.topic_name == topic_name:
                return {
                    'direction': '!pub',
                    'message_type': pub.message_type,
                    'qos': pub.qos,
                    'confidence': 1.0  # Direct introspection
                }

        # Check subscriptions
        for sub in result.subscriptions:
            if sub.topic_name == topic_name:
                return {
                    'direction': '!sub',
                    'message_type': sub.message_type,
                    'qos': sub.qos,
                    'confidence': 1.0
                }

        return None
```

**Integration with Conversion**:

```python
def convert_node(launch_node, introspection_service):
    """Convert launch node to plan node with introspection."""
    package = launch_node.node_package
    executable = launch_node.node_executable

    # Try introspection first
    introspection_result = introspection_service.introspect_node_interfaces(
        package, executable
    )

    sockets = {}
    for remap_from, remap_to in launch_node.remappings:
        # Try to get socket info from introspection
        if introspection_result:
            socket_info = introspection_service.get_socket_info(
                package, executable, remap_from
            )

            if socket_info:
                sockets[remap_from] = {
                    'direction': socket_info['direction'],
                    'type': socket_info['message_type'],
                    'qos': socket_info['qos'],
                    'source': 'introspection'
                }
                continue

        # If introspection failed or didn't find this socket, mark as TODO
        sockets[remap_from] = {
            'direction': '!todo',
            'type': 'TODO',
            'source': 'unknown',
            'comment': f'Unable to introspect {package}::{executable}. Please specify direction and type manually.'
        }

    return PlanNode(package=package, executable=executable, sockets=sockets)
```

**Pass 1: TODO Stub Generation**

When introspection succeeds, sockets are fully resolved:

```yaml
node:
  camera:
    pkg: image_tools
    exec: cam2image
    socket:
      # Resolved via introspection
      image: !pub
```

When introspection fails, generate TODO markers:

```yaml
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
  custom_link: !pubsub
    # TODO: Specify message type
    # Unable to determine from introspection
    type: "TODO"
    src: ["custom_node/data"]
```

**Pass 2: Pattern Learning from User Corrections**

Learn from user-completed TODOs to auto-fill similar cases:

```python
@dataclass
class LearnedPattern:
    pattern_type: str      # "socket_direction", "message_type"
    package: str           # Package name
    executable: str        # Executable name
    socket_name: str       # Socket/topic name
    value: Any            # !pub, !sub, msg type
    examples: List[str]    # Where it was applied

class PatternLearner:
    def learn_from_changes(self, old_plan, new_plan):
        """Detect user completions of TODO markers."""
        changes = diff_plans(old_plan, new_plan)

        for change in changes:
            # User changed !todo to actual direction
            if change.old_value == "!todo" and change.new_value in ["!pub", "!sub"]:
                self.add_pattern(LearnedPattern(
                    pattern_type="socket_direction",
                    package=change.node.package,
                    executable=change.node.executable,
                    socket_name=change.socket_name,
                    value=change.new_value,
                    examples=[change.location]
                ))

            # User filled in message type
            if change.old_value == "TODO" and change.field == "message_type":
                self.add_pattern(LearnedPattern(
                    pattern_type="message_type",
                    package=change.nodes[0].package,  # From src node
                    executable=change.nodes[0].executable,
                    socket_name=change.nodes[0].socket,
                    value=change.new_value,
                    examples=[change.location]
                ))

    def apply_patterns(self, pending_todos):
        """Apply learned patterns to TODOs with exact matches."""
        for todo in pending_todos:
            # Only apply if exact match: same package, executable, and socket name
            pattern = self.find_exact_match(
                todo.package, todo.executable, todo.socket_name
            )
            if pattern:
                todo.inferred_value = pattern.value
                todo.comment = f"Auto-filled from previous user completion"
```

### F63: Plan Builder with Includes

**Challenge**: Preserve launch file structure in generated plans.

**Approach**: Generate plan includes matching launch includes:

```python
# Input: Launch file with include
IncludeLaunchDescription(
    PythonLaunchDescriptionSource('/path/to/camera.launch.py'),
    launch_arguments={'device': '/dev/video0'}.items()
)

# Output: Plan with include
include:
  camera:
    file: "camera.plan.yaml"
    arg:
      device: "/dev/video0"
```

**Argument Forwarding**: Map launch arguments to plan arguments:

```yaml
# Launch DeclareLaunchArgument
DeclareLaunchArgument("use_sim_time", default_value="false")

# Plan arg section
arg:
  use_sim_time:
    type: "bool"
    default: !bool false
```

### F64: CLI Tool with Multi-Pass Support

**Commands**:

1. **convert**: Pass 1 conversion with TODO generation
   ```bash
   launch2plan convert robot.launch.py
   # Generates: robot.plan.yaml + robot.plan.meta.json
   ```

2. **refine**: Pass 2+ with pattern learning
   ```bash
   launch2plan refine robot.plan.yaml
   # Updates plan using learned patterns
   ```

3. **validate**: Compile plan to check correctness
   ```bash
   launch2plan validate robot.plan.yaml
   # Runs: ros2plan compile robot.plan.yaml
   ```

4. **status**: Show TODO completion progress
   ```bash
   launch2plan status robot.plan.yaml
   # Shows: X% complete, N TODOs remaining
   ```

5. **diff**: Compare two plan versions
   ```bash
   launch2plan diff robot.plan.v1.yaml robot.plan.v2.yaml
   ```

**Metadata Tracking**:

```json
{
  "source_file": "robot.launch.py",
  "pass_number": 2,
  "todos": [
    {
      "location": "node.camera.socket.image",
      "field": "direction",
      "hint": "Heuristic suggests !pub (confidence: 0.7)",
      "status": "completed",
      "user_value": "!pub"
    }
  ],
  "learned_patterns": [
    {
      "type": "socket_direction",
      "matcher": "image",
      "value": "!pub",
      "confidence": 0.9
    }
  ]
}
```

### F65: Testing and Validation

**Test Strategy**:

1. **Unit Tests**
   - Branch exploration logic
   - Socket inference heuristics
   - Pattern learning algorithms
   - Plan building

2. **Integration Tests**
   - Full conversion workflow
   - Multi-pass refinement
   - Include handling

3. **Validation Tests**
   - Generated plans must compile
   - Semantic equivalence checks
   - Round-trip conversion

**Always Validate**: Every generated plan is compiled to catch errors early:

```bash
# After conversion
launch2plan convert robot.launch.py
ros2plan compile robot.plan.yaml  # Must succeed
```

## Conversion Components

### 1. Argument Mapping

```python
# Input: ROS 2 Launch
DeclareLaunchArgument(
    "camera_fps",
    default_value="30",
    description="Camera frame rate"
)

# Output: ROS-Plan
arg:
  camera_fps:
    type: "i64"
    default: !i64 30
    help: "Camera frame rate"
```

### 2. Node Conversion

```python
# Input: Launch Node
Node(
    package='demo_nodes_cpp',
    executable='talker',
    name='talker',
    namespace='/demo',
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    remappings=[('chatter', '/demo/chatter')]
)

# Output: Plan Node
node:
  talker:
    pkg: demo_nodes_cpp
    exec: talker
    ns: /demo
    param:
      use_sim_time: "$(use_sim_time)"
    socket:
      chatter: !pub  # Inferred or TODO
```

### 3. Socket Inference

```python
# From remappings and introspection, resolve sockets
remappings = [
    ('image_raw', '/camera/image_raw'),
    ('cmd_vel', '/robot/cmd_vel'),
]

# Introspect to get actual directions
introspection_result = introspect_node("sensor_pkg", "camera_node")

# If introspection succeeds
socket:
  image_raw: !pub  # From introspection

# If introspection fails
socket:
  # TODO: Specify !pub or !sub
  # Unable to introspect sensor_pkg::camera_node
  # Remapping: image_raw -> /camera/image_raw
  image_raw: !todo
```

### 4. Link Inference

```python
# Discover topics from multiple nodes
node1: remaps 'chatter' -> '/demo/chatter'  # talker: !pub
node2: remaps 'chatter' -> '/demo/chatter'  # listener: !sub

# Introspect both nodes to get message types
talker_info = introspect_node("demo_nodes_cpp", "talker")
listener_info = introspect_node("demo_nodes_cpp", "listener")

# If introspection succeeds, generate complete link
link:
  demo_chatter: !pubsub
    type: "std_msgs/msg/String"  # From introspection
    src: ["talker/chatter"]
    dst: ["listener/chatter"]

# If introspection fails for message type
link:
  demo_chatter: !pubsub
    # TODO: Specify message type
    # Unable to determine from introspection
    type: "TODO"
    src: ["talker/chatter"]
    dst: ["listener/chatter"]
```

### 5. Condition Tracking

```python
# Input: Conditional node
GroupAction(
    actions=[
        Node(package='pkg', executable='node', name='optional_node')
    ],
    condition=IfCondition(LaunchConfiguration('enable_feature'))
)

# Output: Node with when clause
node:
  optional_node:
    pkg: pkg
    exec: node
    when: "$(enable_feature)"
```

### 6. Include Conversion

```python
# Input: Launch include
IncludeLaunchDescription(
    PythonLaunchDescriptionSource('camera.launch.py'),
    launch_arguments={'device': '/dev/video0', 'fps': '30'}.items(),
    condition=IfCondition(LaunchConfiguration('use_camera'))
)

# Output: Plan include
include:
  camera:
    file: "camera.plan.yaml"
    arg:
      device: "/dev/video0"
      fps: !i64 30
    when: "$(use_camera)"
```

## Example: Complete Multi-Pass Workflow

### Input Launch File

```python
# robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='sensor_pkg',
            executable='camera_node',
            name='camera',
            parameters=[{'fps': 30}],
            remappings=[('image_raw', '/camera/image')]
        ),

        Node(
            package='processing_pkg',
            executable='detector',
            name='detector',
            remappings=[('image', '/camera/image'), ('detections', '/objects')]
        )
    ])
```

### Pass 1: Initial Conversion with Introspection

```bash
$ launch2plan convert robot.launch.py
Running introspection...
✓ Introspected sensor_pkg::camera_node (1 publisher, 0 subscribers)
✓ Introspected processing_pkg::detector (1 subscriber, 1 publisher)
✓ Plan saved to: robot.plan.yaml
✓ Metadata saved to: robot.plan.meta.json
✓ Discovered 2 nodes
✓ All sockets resolved via introspection (confidence: 1.0)
✓ 0 TODOs remaining
```

Generated `robot.plan.yaml`:

```yaml
arg:
  use_sim_time:
    type: "bool"
    default: !bool false

node:
  camera:
    pkg: sensor_pkg
    exec: camera_node
    param:
      fps: !i64 30
    socket:
      # Resolved via introspection (confidence: 1.0)
      image_raw: !pub

  detector:
    pkg: processing_pkg
    exec: detector
    socket:
      # Resolved via introspection (confidence: 1.0)
      image: !sub

      # Resolved via introspection (confidence: 1.0)
      detections: !pub

link:
  camera_image: !pubsub
    # Resolved via introspection (confidence: 1.0)
    type: "sensor_msgs/msg/Image"
    src: ["camera/image_raw"]
    dst: ["detector/image"]

  detections: !pubsub
    # Resolved via introspection (confidence: 1.0)
    type: "std_msgs/msg/String"
    src: ["detector/detections"]
```

### User Review (Optional)

Since introspection resolved all sockets, no user intervention needed! But user can still review and adjust:

```yaml
# User might want to adjust link names or structure
link:
  # Rename for clarity
  camera_to_detector: !pubsub  # ← Renamed from camera_image
    type: "sensor_msgs/msg/Image"
    src: ["camera/image_raw"]
    dst: ["detector/image"]
```

### Validation

```bash
$ launch2plan validate robot.plan.yaml
✓ Plan is valid!

Compilation output:
Generated executable plan with 2 nodes
```

## Conversion Challenges and Solutions

### Challenge 1: Socket Direction Ambiguity

**Problem**: Can't determine if `image` topic is !pub or !sub from remapping alone.

**Solution**:
- **Runtime Introspection**: Use `ros2-introspect` to query actual node interfaces
- **TODO markers**: When introspection fails or is not available
- **Pattern learning**: Apply user completions to identical cases across multiple files

### Challenge 2: Message Type Unknown

**Problem**: Launch files don't specify message types.

**Solution**:
- **Runtime Introspection**: Use `ros2-introspect` to query actual node interfaces
- **TODO markers**: When introspection is not available or fails
- **Pattern learning**: Remember user-specified types for reuse across similar nodes

### Challenge 3: Complex Conditionals

**Problem**: Nested conditions and dynamic evaluation.

**Solution**:
- Explore all branches without evaluation
- Generate compound `when` clauses using Lua
- Track condition paths through traversal

### Challenge 4: Dynamic Node Creation

**Problem**: OpaqueFunction and dynamic Python code.

**Solution**:
- Mark as limitation in generated plan
- Add manual TODO for user review
- Document what couldn't be converted

### Challenge 5: Include Depth

**Problem**: Deep nesting of launch includes.

**Solution**:
- No artificial depth limits
- Cycle detection prevents infinite loops
- Preserve include hierarchy in plan

## Implementation Plan

### Implementation Status

**✓ Phase 1: Foundation & Basic Visitor** (Completed)
- Created `launch2plan` package structure
- Implemented branch-exploring visitor that visits ALL branches
- Created CLI with `convert` command
- Extract node metadata with condition tracking
- **Tests**: 3/3 passing

**✓ Phase 2: RMW Introspection Integration** (Completed)
- Created `IntrospectionService` with caching
- Socket direction and message type resolution from introspection
- Graceful handling of introspection failures
- **Tests**: 5/5 passing

**✓ Phase 3: Socket Inference & TODO Generation** (Completed)
- Created `inference.py` module with `SocketInferenceEngine`
- Integrated introspection with node conversion
- Implemented topic name normalization (with/without leading slash)
- Generate TODO markers when introspection fails
- Helpful comments for TODO markers (remapping info, suggestions)
- **Tests**: 6/6 passing

**✓ Phase 4: Plan Builder & Link Generation** (Completed)
- Created `builder.py` module with `PlanBuilder` class
- Generate node sections with sockets using ruamel.yaml
- Infer links by matching remappings (group by resolved topic name)
- Generate link sections with message types from introspection
- Write complete plan YAML with proper formatting
- Support TODO markers in links when message type unknown
- **Tests**: 10/10 passing

**✓ Phase 5: Argument & Parameter Conversion** (Completed)
- Extended `visitor.py` to capture `DeclareLaunchArgument` actions
- Created `arg_inference.py` module for type inference from default values
- Extended `builder.py` to generate `arg` section with type tags (!bool, !i64, !f64, !str, !todo)
- Implemented LaunchConfiguration substitution to $(arg_name) syntax
- Convert node parameters with LaunchConfiguration references
- Support nested parameter dictionaries with recursive conversion
- **Tests**: 26/26 passing (17 arg_inference + 9 builder tests)

**✓ Phase 6: Conditional Branch Exploration** (Completed)
- Enhanced `extract_condition_expression()` to handle IfCondition and UnlessCondition
- Support for LaunchConfiguration substitutions in conditions
- Support for TextSubstitution (literal "true"/"false", numeric "1"/"0")
- Proper handling of UnlessCondition as subclass of IfCondition (check order matters)
- Nested condition tracking with compound "and" expressions
- Condition stack management via context manager
- Convert conditions to Lua expressions for `when` clauses
- **Tests**: 18/18 passing (all conditional tests)

**✓ Phase 7: Include Handling & Plan Hierarchy** (Completed)
- Enhanced `visit_include_launch_description()` to recursively process includes
- Implemented include path resolution with name-mangled `_LaunchDescriptionSource__location` attribute
- Added cycle detection using include_stack to prevent infinite recursion
- Extended `builder.py` with `_build_include_section()` method
- Generate include sections with argument forwarding and type inference
- Support conditional includes with `when` clauses
- Remove ".launch" suffix from file stem for include names
- Handle duplicate include names with numeric suffixes
- **Tests**: 8/8 passing (all include tests)

**✓ Phase 8: Metadata Tracking** (Completed)
- Created `metadata.py` with data structures (TodoItem, ConversionMetadata, ConversionStats, TodoContext, TodoReason, TodoStatus, NodeSource)
- Modified `builder.py` to collect TODOs during plan generation with rich context (node package, executable, remapping, reason, hints)
- Implemented `MetadataManager` for saving/loading metadata to JSON (`.plan.meta.json` files)
- Created `PlanParser` for plan YAML parsing and TODO discovery with JSONPath addressing
- Implemented `TodoStatusUpdater` for detecting user-completed TODOs by comparing metadata against current plan
- Added `statistics.py` for conversion statistics calculation (nodes, links, TODOs, completion rate, introspection metrics)
- Integrated metadata generation into `handle_convert()` CLI command with SHA256 staleness detection
- Added `status` subcommand to display TODO completion progress and statistics
- **Tests**: 13/13 passing (all metadata tests)

**Current Status**: 85 tests passing (440 total across entire project)

---

### Phase 1: Foundation & Basic Visitor ✓ COMPLETED

**Goal**: Set up project structure and basic launch file visiting

**Implementation**:
- ✓ Created `launch2plan` package structure in `python/launch2plan`
- ✓ Set up pyproject.toml with dependencies (ros2-introspect)
- ✓ Created CLI with `convert` command
- ✓ Implemented branch-exploring visitor (explores ALL branches, not just true)
- ✓ Extract node metadata (package, executable, name, namespace, remappings)
- ✓ Track condition expressions for `when` clause generation
- ✓ Detect includes (basic - no recursion yet)

**Tests** (3/3 passing):
- ✓ `test_visitor.py::test_visit_simple_node` - Single node discovery
- ✓ `test_visitor.py::test_visit_multiple_nodes` - Multiple nodes with remappings
- ✓ `test_cli.py::test_convert_command` - Basic CLI invocation

**Deliverable**: ✓ Can visit a simple launch file and list discovered nodes

---

### Phase 2: RMW Introspection Integration ✓ COMPLETED

**Goal**: Integrate ros2-introspect for accurate socket inference

**Implementation**:
- ✓ Created `introspection.py` module
- ✓ Implemented `IntrospectionService` class with caching
- ✓ Added `get_socket_info()` method for topic resolution
- ✓ Added `get_all_topics()` method for full node interface query
- ✓ Graceful handling of introspection failures (returns None)
- ✓ Cache introspection results per package::executable

**Tests** (5/5 passing):
- ✓ `test_introspection.py::test_introspect_demo_nodes` - Demo nodes (talker/listener)
- ✓ `test_introspection.py::test_introspection_cache` - Caching behavior
- ✓ `test_introspection.py::test_introspection_fallback` - Handle failures
- ✓ `test_introspection.py::test_socket_direction_resolution` - Pub/sub detection
- ✓ `test_introspection.py::test_message_type_resolution` - Type extraction

**Deliverable**: ✓ Can introspect nodes and determine socket directions + message types

---

### Phase 3: Socket Inference & TODO Generation ✓ COMPLETED

**Goal**: Integrate introspection with node conversion, generate TODO markers for unknowns

**Implementation**:
- ✓ Created `inference.py` module with `SocketInferenceEngine` class
- ✓ Implemented `infer_sockets_for_node()` with introspection integration
- ✓ Added topic name normalization (handles with/without leading slash)
- ✓ Generate TODO markers when introspection fails completely
- ✓ Generate TODO markers when socket not found in introspection results
- ✓ Add helpful comments to TODO markers (package::executable, remapping info, suggestions)
- ✓ Support batch inference with `infer_sockets_for_nodes()` helper

**Tests** (6/6 passing):
- ✓ `test_inference.py::test_resolve_from_introspection` - Successful resolution
- ✓ `test_inference.py::test_introspection_not_available` - Generate TODO when introspection fails
- ✓ `test_inference.py::test_socket_not_found_in_introspection` - Generate TODO when socket not found
- ✓ `test_inference.py::test_todo_comment_generation` - Helpful TODO comments
- ✓ `test_inference.py::test_topic_name_matching` - Topic name normalization
- ✓ `test_inference.py::test_infer_sockets_for_multiple_nodes` - Batch inference

**Deliverable**: ✓ Node conversion with introspection-based inference or explicit TODOs

---

### Phase 4: Plan Builder & Link Generation ✓ COMPLETED

**Goal**: Generate complete ROS-Plan YAML with sockets and links

**Implementation**:
- ✓ Created `builder.py` module with `PlanBuilder` class
- ✓ Implemented `build_plan()` to generate complete plan from nodes and sockets
- ✓ Generate node sections with pkg, exec, namespace, parameters, sockets
- ✓ Used ruamel.yaml for YAML formatting and preservation
- ✓ Infer links by grouping remappings by resolved topic name
- ✓ Generate link sections with message types from introspection
- ✓ Support TODO markers in links when message type unknown
- ✓ Handle multiple publishers and subscribers per link
- ✓ Support conditional nodes with `when` clauses
- ✓ Write complete plan YAML with proper formatting

**Tests** (10/10 passing):
- ✓ `test_builder.py::test_build_node_section` - Node YAML generation
- ✓ `test_builder.py::test_build_socket_section` - Socket with directions and TODO markers
- ✓ `test_builder.py::test_infer_links_from_remappings` - Link discovery from remappings
- ✓ `test_builder.py::test_generate_link_section` - Link YAML with types
- ✓ `test_builder.py::test_full_plan_generation` - Complete plan output
- ✓ `test_builder.py::test_plan_with_todo_sockets` - TODO socket handling
- ✓ `test_builder.py::test_link_with_todo_message_type` - TODO message type in links
- ✓ `test_builder.py::test_multiple_publishers_and_subscribers` - Multi-endpoint links
- ✓ `test_builder.py::test_conditional_node` - Node with when clause
- ✓ `test_builder.py::test_plan_to_yaml_string` - YAML string conversion

**Deliverable**: ✓ Generate valid, compilable plan YAML files with sockets and links

---

### Phase 5: Argument & Parameter Conversion ✓ COMPLETED

**Goal**: Convert launch arguments and parameters to plan format

**Implementation**:
- ✓ Extended `visitor.py` to capture `DeclareLaunchArgument` actions
- ✓ Added `LaunchArgumentMetadata` dataclass with name, default_value, description
- ✓ Created `arg_inference.py` module for type inference from default values
- ✓ Infer types: bool (true/false), i64 (integers), f64 (floats), str (strings), todo (no default)
- ✓ Extended `builder.py` with `_build_arg_section()` method
- ✓ Generate arg section with proper type tags (!bool, !i64, !f64, !str, !todo)
- ✓ Implemented `_convert_launch_configurations()` to handle LaunchConfiguration substitutions
- ✓ Recursive conversion for nested parameter dictionaries
- ✓ Support LaunchConfiguration in lists and nested structures
- ✓ Convert LaunchConfiguration references to $(arg_name) syntax

**Tests** (26/26 passing):
- ✓ `test_arg_inference.py::test_infer_bool_true` - Boolean true inference
- ✓ `test_arg_inference.py::test_infer_bool_false` - Boolean false inference
- ✓ `test_arg_inference.py::test_infer_bool_case_insensitive` - Case-insensitive boolean
- ✓ `test_arg_inference.py::test_infer_integer_positive` - Positive integer
- ✓ `test_arg_inference.py::test_infer_integer_negative` - Negative integer
- ✓ `test_arg_inference.py::test_infer_integer_zero` - Zero value
- ✓ `test_arg_inference.py::test_infer_float_decimal` - Decimal float
- ✓ `test_arg_inference.py::test_infer_float_scientific` - Scientific notation
- ✓ `test_arg_inference.py::test_infer_float_negative` - Negative float
- ✓ `test_arg_inference.py::test_infer_string_path` - Path string
- ✓ `test_arg_inference.py::test_infer_string_namespace` - Namespace string
- ✓ `test_arg_inference.py::test_infer_string_general` - General string
- ✓ `test_arg_inference.py::test_infer_none_default` - TODO marker for no default
- ✓ `test_arg_inference.py::test_infer_empty_string` - Empty string
- ✓ `test_arg_inference.py::test_infer_whitespace_handling` - Whitespace stripping
- ✓ `test_arg_inference.py::test_infer_mixed_alphanumeric` - Mixed alphanumeric
- ✓ `test_arg_inference.py::test_infer_numeric_string_with_units` - Numeric with units
- ✓ `test_builder.py::test_build_arg_section_with_bool` - Boolean arg generation
- ✓ `test_builder.py::test_build_arg_section_with_int` - Integer arg generation
- ✓ `test_builder.py::test_build_arg_section_with_float` - Float arg generation
- ✓ `test_builder.py::test_build_arg_section_with_string` - String arg generation
- ✓ `test_builder.py::test_build_arg_section_with_no_default` - TODO arg generation
- ✓ `test_builder.py::test_build_arg_section_with_multiple_types` - Multiple types
- ✓ `test_builder.py::test_launch_configuration_substitution_in_params` - LaunchConfiguration in params
- ✓ `test_builder.py::test_nested_launch_configuration_in_params` - Nested LaunchConfiguration
- ✓ `test_builder.py::test_complete_plan_with_args_and_params` - Complete plan with args and params

**Deliverable**: ✓ Complete argument and parameter handling with type inference and substitution

---

### Phase 6: Conditional Branch Exploration ✓ COMPLETED

**Goal**: Handle conditional nodes and generate `when` clauses

**Implementation**:
- ✓ Enhanced `extract_condition_expression()` function with robust condition handling
- ✓ Check UnlessCondition before IfCondition (subclass relationship)
- ✓ Extract predicate from name-mangled attribute `_IfCondition__predicate_expression`
- ✓ Handle LaunchConfiguration substitutions → `$(var_name)`
- ✓ Handle UnlessCondition with negation → `$(not var_name)`
- ✓ Support TextSubstitution for literal values ("true", "false", "1", "0")
- ✓ Case-insensitive boolean text handling
- ✓ Multiple substitutions with Lua concatenation (..)
- ✓ Nested condition tracking with condition_stack
- ✓ Compound expressions with "and" operator for nested conditions
- ✓ Helper function `_extract_variable_name()` for variable name extraction
- ✓ Condition context manager for proper stack management
- ✓ Node with conditional when clauses (already implemented in Phase 4)

**Tests** (18/18 passing):
- ✓ `test_conditionals.py::test_extract_ifcondition_with_launch_configuration` - IfCondition with LaunchConfiguration
- ✓ `test_conditionals.py::test_extract_unlesscondition_with_launch_configuration` - UnlessCondition with LaunchConfiguration
- ✓ `test_conditionals.py::test_extract_ifcondition_with_text_true` - IfCondition with literal true
- ✓ `test_conditionals.py::test_extract_ifcondition_with_text_false` - IfCondition with literal false
- ✓ `test_conditionals.py::test_extract_unlesscondition_with_text_true` - UnlessCondition with literal true
- ✓ `test_conditionals.py::test_extract_unlesscondition_with_text_false` - UnlessCondition with literal false
- ✓ `test_conditionals.py::test_extract_none_condition` - None condition returns None
- ✓ `test_conditionals.py::test_condition_stack_single_level` - Single condition
- ✓ `test_conditionals.py::test_condition_stack_nested_conditions` - Two nested conditions
- ✓ `test_conditionals.py::test_condition_stack_three_levels` - Three nested conditions
- ✓ `test_conditionals.py::test_condition_context_none` - None doesn't affect stack
- ✓ `test_conditionals.py::test_condition_stack_mixed_if_unless` - Mixed If/Unless conditions
- ✓ `test_conditionals.py::test_ifcondition_with_numeric_text` - Numeric text "1"/"0"
- ✓ `test_conditionals.py::test_unlesscondition_with_numeric_text` - Numeric negation
- ✓ `test_conditionals.py::test_ifcondition_with_arbitrary_text` - Custom text values
- ✓ `test_conditionals.py::test_unlesscondition_with_arbitrary_text` - Custom text negation
- ✓ `test_conditionals.py::test_extract_empty_condition` - Empty predicate lists
- ✓ `test_conditionals.py::test_case_insensitive_boolean_text` - Case-insensitive booleans

**Deliverable**: ✓ Complete support for conditional nodes with `when` clauses

---

### Phase 7: Include Handling & Plan Hierarchy ✓ COMPLETED

**Goal**: Preserve launch file structure with plan includes

**Implementation**:
- ✓ Enhanced `visit_include_launch_description()` to recursively process includes
- ✓ Implemented proper include path resolution using `_LaunchDescriptionSource__location` attribute
- ✓ Added cycle detection using include_stack (prevents infinite recursion)
- ✓ Extended `builder.py` with `_build_include_section()` method
- ✓ Generate include sections with file reference and argument forwarding
- ✓ Infer argument types for included files (!bool, !i64, !f64, !str)
- ✓ Support conditional includes with `when` clauses
- ✓ Handle duplicate include names with numeric suffixes
- ✓ Remove ".launch" suffix from file stems for cleaner include names

**Tests** (8/8 passing):
- ✓ `test_includes.py::test_include_detection` - Detect includes with path resolution
- ✓ `test_includes.py::test_include_with_arguments` - Argument capture
- ✓ `test_includes.py::test_include_with_launch_configuration_arguments` - LaunchConfiguration substitution
- ✓ `test_includes.py::test_cycle_detection_simple` - Simple cycle prevention
- ✓ `test_includes.py::test_cycle_detection_nested` - Nested cycle prevention
- ✓ `test_includes.py::test_include_section_generation` - Include YAML generation
- ✓ `test_includes.py::test_include_with_duplicate_names` - Duplicate name handling
- ✓ `test_includes.py::test_include_argument_type_inference` - Argument type inference

**Deliverable**: ✓ Full support for launch file includes with plan includes, argument forwarding, and cycle detection

---

### Phase 8: Metadata Tracking ✓ COMPLETED

**Goal**: Track conversion state for transparency and debugging

**Micro-Steps**:

1. ✓ **F66: Metadata Data Structures** (2 hours)
   - Define dataclasses: `ConversionMetadata`, `TodoItem`, `TodoContext`, `NodeSource`, `ConversionStats`
   - Define enums: `TodoStatus`, `TodoReason`
   - Create type annotations and validation

2. ✓ **F67: TODO Collection During Conversion** (3 hours)
   - Modify `builder.py` to collect TODOs during plan generation
   - Track location, field, value, and context for each TODO
   - Record node sources (file, line, include path, condition)
   - Generate metadata alongside plan YAML

3. ✓ **F68: Metadata Persistence** (2 hours)
   - Implement `save_metadata()` - serialize to JSON with pretty printing
   - Implement `load_metadata()` - deserialize from JSON
   - Handle missing metadata files gracefully
   - Validate metadata schema on load

4. ✓ **F69: Plan YAML Parsing & TODO Discovery** (4 hours)
   - Implement `parse_plan_yaml()` - load plan YAML structure
   - Implement `find_todos_in_plan()` - scan for `!todo` and `"TODO"` values
   - Build JSONPath for each discovered TODO
   - Compare plan TODOs against metadata TODOs

5. ✓ **F70: TODO Status Update** (3 hours)
   - Implement `detect_completed_todos()` - find TODOs resolved by user
   - Compare metadata TODO list with current plan state
   - Update TODO status to COMPLETED when user fills in value
   - Handle TODOs that were removed entirely

6. ✓ **F71: Conversion Statistics** (2 hours)
   - Implement `calculate_stats()` - compute all statistics
   - Count introspection success/failure rates
   - Calculate completion rate
   - Track performance metrics (timing)

7. ✓ **F72: CLI Integration** (2 hours)
   - Add `--metadata` flag to `convert` command
   - Add `status` subcommand to show TODO completion status
   - Display statistics in human-readable format
   - Warn if metadata is stale (source file changed)

**Tests** (13/13 passing):
- ✓ `test_metadata.py::test_dataclass_serialization` - JSON round-trip
- ✓ `test_metadata.py::test_save_load_metadata` - Persistence
- ✓ `test_metadata.py::test_load_missing_metadata` - Missing metadata handling
- ✓ `test_metadata.py::test_plan_yaml_parsing` - YAML structure parsing
- ✓ `test_metadata.py::test_find_todos_in_plan` - TODO discovery
- ✓ `test_metadata.py::test_get_value_at_path` - JSONPath navigation
- ✓ `test_metadata.py::test_detect_completed_todos` - User edit detection
- ✓ `test_metadata.py::test_detect_completed_todos_ignores_todo_markers` - TODO marker filtering
- ✓ `test_metadata.py::test_check_metadata_staleness` - Source hash checking
- ✓ `test_metadata.py::test_calculate_stats` - Statistics computation
- ✓ `test_metadata.py::test_calculate_stats_empty_plan` - Empty plan statistics
- ✓ `test_metadata.py::test_builder_collects_socket_todos` - Socket TODO collection
- ✓ `test_metadata.py::test_builder_collects_link_todos` - Link TODO collection

**Deliverable**: ✓ Transparent conversion tracking with explicit TODO markers and user edit detection

---

## Phase 8: Metadata Structure Design

### Design Principle: Explicit Tracking, No Guessing

The metadata system tracks the conversion process for transparency and debugging:
1. **Progress tracking** - What TODOs remain, what's been completed by user
2. **Source traceability** - Map plan elements back to launch file origins
3. **Reproducibility** - Understand how a plan was generated
4. **Statistics** - Clear view of conversion completeness

**No heuristics, no guessing, no confidence scoring.** The tool uses only:
- **RMW introspection** - Actual node interfaces (direction, message type, QoS)
- **Launch file structure** - Explicit information (nodes, args, remappings, conditions)
- **User input** - Manual completion of TODOs where information is missing

### Metadata File Structure

Metadata is stored as JSON alongside the generated plan file:
- Plan file: `robot.plan.yaml`
- Metadata file: `robot.plan.meta.json`

This separation keeps the plan file clean and human-readable while preserving conversion context.

### Core Data Model

```python
@dataclass
class ConversionMetadata:
    """Complete metadata for a plan conversion."""

    # Source information
    source_file: str              # Original launch file path
    source_hash: str              # SHA256 hash for change detection
    generated_at: str             # ISO 8601 timestamp
    converter_version: str        # launch2plan version

    # TODO tracking
    todos: List[TodoItem]

    # Conversion statistics
    stats: ConversionStats

    # Node-to-source mapping (for debugging)
    node_sources: Dict[str, NodeSource]  # node_id -> source info
```

#### TodoItem: Track Incomplete Conversions

```python
@dataclass
class TodoItem:
    """A single TODO marker in the generated plan."""

    # Location in plan
    location: str                 # JSONPath-style: "node.camera.socket.image"
    field: str                    # "direction" | "message_type" | "arg_type"

    # Current state
    current_value: str            # "!todo" or "TODO"
    status: TodoStatus            # pending | completed

    # Context for user
    context: TodoContext

@dataclass
class TodoContext:
    """Context information to help user complete TODO."""

    # What we know
    node_package: Optional[str]
    node_executable: Optional[str]
    remapping: Optional[Tuple[str, str]]  # (from, to)

    # Why it's unknown
    reason: TodoReason            # introspection_failed | socket_not_found
    error_message: Optional[str]  # Detailed error if available

    # Helpful hint
    hint: Optional[str]           # Human-readable suggestion for completion

@enum
class TodoStatus(Enum):
    PENDING = "pending"           # Not yet resolved
    COMPLETED = "completed"       # User manually completed

@enum
class TodoReason(Enum):
    INTROSPECTION_FAILED = "introspection_failed"     # ros2-introspect failed
    SOCKET_NOT_FOUND = "socket_not_found"             # Socket not in introspection results
    DYNAMIC_CODE = "dynamic_code"                     # OpaqueFunction or runtime code
    MISSING_PACKAGE = "missing_package"               # Package not available

@dataclass
class NodeSource:
    """Track where a node came from in the launch file."""

    launch_file: str              # Path to launch file
    line_number: Optional[int]    # Line number in launch file (if available)
    include_path: List[str]       # Chain of includes leading to this node
    condition: Optional[str]      # Condition expression (if any)
```

**Design Decision: Location as JSONPath**

Using JSONPath-style strings like `"node.camera.socket.image"` provides:
- ✓ **Unambiguous addressing** - Precisely identifies where in the plan file
- ✓ **Human-readable** - Easy to understand when reviewing metadata
- ✓ **Programmatic access** - Can be parsed to navigate the plan structure
- ✓ **Stable across edits** - Survives minor formatting changes

Alternative considered: Line numbers (rejected - fragile to edits)

#### ConversionStats: Progress Tracking

```python
@dataclass
class ConversionStats:
    """Statistics about the conversion."""

    # Counts
    total_nodes: int
    total_includes: int
    total_links: int
    total_arguments: int

    # TODO tracking
    total_todos: int
    pending_todos: int
    completed_todos: int

    # Introspection results
    nodes_introspected: int              # Nodes successfully introspected
    nodes_failed_introspection: int      # Nodes where introspection failed
    sockets_from_introspection: int      # Sockets resolved via introspection
    sockets_requiring_user_input: int    # Sockets marked as TODO

    # Completion rate
    completion_rate: float               # TODOs completed / total TODOs

    # Performance
    introspection_time_ms: int
    conversion_time_ms: int
```

### Metadata Operations

#### Save/Load (F68: Metadata Persistence)

```python
class MetadataManager:
    """Manage metadata persistence."""

    def save_metadata(self, metadata: ConversionMetadata, plan_path: Path):
        """Save metadata as JSON alongside plan file."""
        meta_path = plan_path.with_suffix('.meta.json')

        # Serialize to JSON with pretty printing
        json_str = json.dumps(
            asdict(metadata),
            indent=2,
            sort_keys=True,
            ensure_ascii=False
        )

        meta_path.write_text(json_str)

    def load_metadata(self, plan_path: Path) -> Optional[ConversionMetadata]:
        """Load metadata from JSON file."""
        meta_path = plan_path.with_suffix('.meta.json')

        if not meta_path.exists():
            return None

        try:
            data = json.loads(meta_path.read_text())
            # Validate schema version if present
            if 'converter_version' in data:
                self._validate_version(data['converter_version'])
            return ConversionMetadata(**data)
        except (json.JSONDecodeError, TypeError, KeyError) as e:
            logging.error(f"Failed to load metadata: {e}")
            return None
```

#### Plan YAML Parsing (F69: Plan YAML Parsing & TODO Discovery)

```python
class PlanParser:
    """Parse plan YAML files and discover TODOs."""

    def parse_plan_yaml(self, plan_path: Path) -> Dict:
        """Load and parse plan YAML file."""
        from ruamel.yaml import YAML

        yaml = YAML()
        with plan_path.open() as f:
            return yaml.load(f)

    def find_todos_in_plan(self, plan_data: Dict) -> List[DiscoveredTodo]:
        """
        Scan plan YAML structure for TODO markers.

        Returns list of TODOs found in the current plan state.
        """
        todos = []

        # Scan node sockets
        if 'node' in plan_data:
            for node_id, node_def in plan_data['node'].items():
                if 'socket' in node_def:
                    for socket_name, socket_value in node_def['socket'].items():
                        # Check for !todo tag
                        if isinstance(socket_value, str) and socket_value == "!todo":
                            todos.append(DiscoveredTodo(
                                location=f"node.{node_id}.socket.{socket_name}",
                                field="direction",
                                current_value="!todo"
                            ))

        # Scan link message types
        if 'link' in plan_data:
            for link_id, link_def in plan_data['link'].items():
                if isinstance(link_def, dict) and link_def.get('type') == 'TODO':
                    todos.append(DiscoveredTodo(
                        location=f"link.{link_id}.type",
                        field="message_type",
                        current_value="TODO"
                    ))

        # Scan argument types
        if 'arg' in plan_data:
            for arg_name, arg_def in plan_data['arg'].items():
                if isinstance(arg_def, dict):
                    if arg_def.get('type') == '!todo':
                        todos.append(DiscoveredTodo(
                            location=f"arg.{arg_name}.type",
                            field="arg_type",
                            current_value="!todo"
                        ))

        return todos

    def get_value_at_path(self, plan_data: Dict, location: str) -> Any:
        """
        Navigate plan structure using JSONPath-style location.

        Example: "node.camera.socket.image" -> plan_data['node']['camera']['socket']['image']
        """
        parts = location.split('.')
        current = plan_data

        for part in parts:
            if not isinstance(current, dict) or part not in current:
                return None
            current = current[part]

        return current

@dataclass
class DiscoveredTodo:
    """A TODO marker found in the current plan."""
    location: str
    field: str
    current_value: str
```

#### TODO Status Update (F70: TODO Status Update)

```python
class TodoStatusUpdater:
    """Detect and update TODO completion status."""

    def detect_completed_todos(
        self,
        metadata: ConversionMetadata,
        plan_data: Dict
    ) -> List[CompletedTodo]:
        """
        Compare metadata TODOs against current plan to find user edits.

        Algorithm:
        1. For each TODO in metadata with status=PENDING
        2. Get current value at TODO location in plan
        3. If current value differs from metadata value AND is not a TODO marker
        4. Mark as COMPLETED and record the new value
        """
        parser = PlanParser()
        completed = []

        for todo in metadata.todos:
            if todo.status != TodoStatus.PENDING:
                continue  # Already completed or removed

            # Get current value from plan
            current_value = parser.get_value_at_path(plan_data, todo.location)

            # Check if user filled in the TODO
            if current_value is not None and \
               current_value != todo.current_value and \
               current_value not in ["!todo", "TODO"]:

                completed.append(CompletedTodo(
                    location=todo.location,
                    field=todo.field,
                    old_value=todo.current_value,
                    new_value=current_value
                ))

                # Update status in metadata
                todo.status = TodoStatus.COMPLETED

        return completed

    def update_metadata_from_plan(
        self,
        metadata: ConversionMetadata,
        plan_path: Path
    ) -> ConversionMetadata:
        """
        Update metadata by comparing against current plan state.

        This is called when user runs `launch2plan status` to see progress.
        """
        parser = PlanParser()
        plan_data = parser.parse_plan_yaml(plan_path)

        # Detect completed TODOs
        completed = self.detect_completed_todos(metadata, plan_data)

        if completed:
            logging.info(f"Detected {len(completed)} completed TODOs")
            for c in completed:
                logging.debug(f"  {c.location}: {c.old_value} -> {c.new_value}")

        # Recalculate statistics
        metadata.stats = self._calculate_stats(metadata, plan_data)

        return metadata

    def check_metadata_staleness(
        self,
        metadata: ConversionMetadata,
        source_path: Path
    ) -> Optional[str]:
        """
        Check if source file has changed since conversion.

        Returns warning message if stale, None if up-to-date.
        """
        import hashlib

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

@dataclass
class CompletedTodo:
    """Record of a TODO that was completed by user."""
    location: str
    field: str
    old_value: str      # Original TODO marker
    new_value: str      # User-provided value
```

**Design Decision: Explicit TODO Discovery**

The system discovers TODOs by scanning the plan YAML structure:

1. **Load plan YAML** - Parse with ruamel.yaml to preserve structure
2. **Scan known locations** - Check node sockets, link types, arg types
3. **Build JSONPath** - Create stable location identifiers
4. **Compare with metadata** - Match discovered TODOs against saved list
5. **Detect completions** - Find TODOs that user has filled in

**Key Principles:**
- ✓ **Structural scan** - Navigate YAML tree, don't use regex on text
- ✓ **Stable addressing** - JSONPath survives formatting changes
- ✓ **Explicit comparison** - Exact value matching, no fuzzy logic
- ✓ **No inference** - If metadata missing, regenerate from scratch

### Example Metadata File

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
        "error_message": "Failed to spawn node: package 'sensor_pkg' not found",
        "hint": "Specify !pub if camera publishes images, !sub if it subscribes"
      }
    },
    {
      "location": "link.camera_image.type",
      "field": "message_type",
      "current_value": "TODO",
      "status": "pending",
      "context": {
        "reason": "socket_not_found",
        "hint": "Common image types: sensor_msgs/msg/Image, sensor_msgs/msg/CompressedImage"
      }
    }
  ],

  "node_sources": {
    "camera": {
      "launch_file": "/path/to/robot.launch.py",
      "line_number": 42,
      "include_path": [],
      "condition": null
    },
    "detector": {
      "launch_file": "/path/to/vision.launch.py",
      "line_number": 15,
      "include_path": ["robot.launch.py"],
      "condition": "$(use_vision)"
    }
  },

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
    "sockets_from_introspection": 3,
    "sockets_requiring_user_input": 2,
    "completion_rate": 0.0,
    "introspection_time_ms": 1500,
    "conversion_time_ms": 3200
  }
}
```

### Key Design Decisions Summary

1. **Separate metadata file** - Keep plan YAML clean and human-readable
2. **JSONPath addressing** - Unambiguous, stable location identifiers for TODOs
3. **Explicit TODO tracking** - Every unknown gets tracked with helpful context
4. **No heuristics or guessing** - Only introspection and user input, no pattern learning
5. **Rich context** - Help users understand why TODOs exist and how to complete them
6. **Node source tracking** - Map each node back to its launch file origin
7. **Simple status tracking** - Pending or completed, nothing inferred
8. **Statistics for progress** - Clear view of what was introspected vs. requiring user input
9. **Transparency** - Metadata makes conversion process fully visible and debuggable
10. **User control** - Tool never guesses, user makes all decisions about unknowns

---

### Phase 9: Validation & Compilation (Week 5)

**Goal**: Validate generated plans and ensure they compile

**Tasks**:
1. Create `validator.py` module
2. Implement plan compilation check (call `ros2plan compile`)
3. Parse and report compilation errors
4. Add `validate` CLI command
5. Implement `status` CLI command (show TODO completion rate)

**Tests**:
- `test_validator.py::test_validate_simple_plan` - Basic validation
- `test_validator.py::test_compilation_errors` - Error reporting
- `test_validator.py::test_status_command` - Status display
- `test_cli.py::test_validate_command` - CLI validate
- `test_cli.py::test_status_command` - CLI status

**Deliverable**: Always-validate workflow ensures correctness

---

### Phase 10: End-to-End Testing & Examples (Week 5)

**Goal**: Comprehensive testing with real-world launch files

**Tasks**:
1. Create test fixtures (simple, complex, with includes)
2. Test complete conversion workflow
3. Validate all generated plans compile successfully
4. Create example conversions for documentation
5. Test edge cases (missing packages, invalid syntax, etc.)

**Tests**:
- `test_e2e.py::test_convert_simple_launch` - Simple launch file
- `test_e2e.py::test_convert_complex_launch` - Complex with conditions
- `test_e2e.py::test_convert_with_includes` - Multi-file launch
- `test_e2e.py::test_introspection_demo_nodes` - Demo nodes (talker/listener)
- `test_e2e.py::test_compilation_success` - All plans compile

**Deliverable**: Production-ready tool with comprehensive test coverage

---

### Summary: 10 Phases, 5 Weeks

**Week 1**: Foundation + Introspection (Phases 1-2)
**Week 2**: Inference + Plan Building (Phases 3-4)
**Week 3**: Arguments + Conditionals (Phases 5-6)
**Week 4**: Includes + Pattern Learning (Phases 7-8)
**Week 5**: Validation + E2E Testing (Phases 9-10)

**Total Tests**: ~50+ test cases across all phases

## Limitations and Future Work

### Current Limitations

1. **Dynamic Behavior**: Cannot analyze `OpaqueFunction` or runtime-evaluated code (will generate TODO markers)
2. **Introspection Failures**: Nodes that cannot be introspected require manual TODO completion
3. **Complex Conditions**: Some Python expressions may not convert cleanly to Lua (will be marked as TODO)
4. **Custom Packages**: Nodes from packages not available in the environment require manual specification
5. **No Heuristics**: Tool will not guess - unknown information becomes explicit TODO markers for user completion

### Future Enhancements

1. **XML/YAML Launch Support**: Extend beyond Python launch files
2. **Interactive Mode**: Guide user through TODO completion with prompts
3. **Template Library**: Pre-built patterns for common ROS packages (nav2, moveit2)
4. **Diff Visualization**: Web UI for viewing changes between passes
5. **Batch Conversion**: Convert entire workspace of launch files at once
6. **QoS Profile Preservation**: Extract and preserve QoS settings from introspection
7. **Composition Support**: Handle component container nodes
8. **Lifecycle Node Handling**: Proper support for managed nodes

## References

- **ros2-introspect**: RMW introspection tool for discovering node interfaces (`python/ros2-introspect`)
- **rmw_introspect_cpp**: Custom RMW implementation for metadata capture (`ros2/rmw_introspect_cpp`)
- **launch2dump**: Node metadata extraction tool (same codebase)
- **ROS 2 Launch**: Python launch file documentation
- **ROS-Plan Format**: Plan YAML specification (`book/src/plan-format.md`)
- **ros2plan**: Plan compiler and runtime

