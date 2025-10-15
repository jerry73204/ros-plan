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
│ Pass 1: Initial Conversion                                        │
│ • Explore all branches (no depth limit, cycle detection)         │
│ • Generate plan skeleton with nodes and arguments                │
│ • Mark unknown socket directions/types as TODO with hints        │
│ • Save metadata for tracking                                     │
└────────────────┬─────────────────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│ User Annotation                                                   │
│ • Review generated plan and TODO markers                         │
│ • Fill in socket directions (!pub or !sub)                       │
│ • Specify message types                                          │
│ • Verify when clauses                                            │
└────────────────┬─────────────────────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────────────────────┐
│ Pass 2+: Pattern Learning                                        │
│ • Detect user changes since last pass                           │
│ • Learn patterns from completed TODOs                            │
│ • Apply patterns to similar pending TODOs (high confidence)      │
│ • Update metadata with learned patterns                          │
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

**Current Status**: 24 tests passing (385 total across entire project)

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

### Phase 5: Argument & Parameter Conversion

**Goal**: Convert launch arguments and parameters to plan format

**Tasks**:
1. Convert `DeclareLaunchArgument` to plan `arg` section
2. Infer types from default values (string, int, float, bool)
3. Convert node parameters to plan `param` section
4. Handle `LaunchConfiguration` substitutions → `$(arg_name)`
5. Preserve help/description text

**Tests**:
- `test_builder.py::test_convert_launch_arguments` - Arg conversion
- `test_builder.py::test_infer_argument_types` - Type inference
- `test_builder.py::test_convert_node_parameters` - Param conversion
- `test_builder.py::test_launch_configuration_substitution` - $(var) syntax

**Deliverable**: Complete argument and parameter handling

---

### Phase 6: Conditional Branch Exploration (Week 3)

**Goal**: Handle conditional nodes and generate `when` clauses

**Tasks**:
1. Extend visitor to track condition context
2. Extract condition expressions from `IfCondition`/`UnlessCondition`
3. Convert Python conditions to Lua expressions for `when` clauses
4. Handle nested conditions (compound expressions)
5. Generate nodes with appropriate `when` clauses

**Tests**:
- `test_visitor.py::test_conditional_node` - IfCondition handling
- `test_visitor.py::test_unless_condition` - UnlessCondition handling
- `test_visitor.py::test_nested_conditions` - Nested conditionals
- `test_visitor.py::test_when_clause_generation` - Lua conversion
- `test_builder.py::test_build_conditional_node` - Node with `when`

**Deliverable**: Support conditional nodes with `when` clauses

---

### Phase 7: Include Handling & Plan Hierarchy (Week 4)

**Goal**: Preserve launch file structure with plan includes

**Tasks**:
1. Detect `IncludeLaunchDescription` actions
2. Recursively convert included launch files
3. Generate plan includes with argument forwarding
4. Implement cycle detection (prevent infinite recursion)
5. Map launch arguments to plan include arguments

**Tests**:
- `test_visitor.py::test_include_detection` - Detect includes
- `test_visitor.py::test_recursive_conversion` - Multi-level includes
- `test_visitor.py::test_cycle_detection` - Prevent infinite loops
- `test_builder.py::test_generate_include_section` - Include YAML
- `test_builder.py::test_include_argument_forwarding` - Arg mapping

**Deliverable**: Support launch file includes with plan includes

---

### Phase 8: Metadata Tracking & Pattern Learning (Week 4)

**Goal**: Track conversion state and learn from user edits

**Tasks**:
1. Create `metadata.py` module
2. Implement metadata JSON structure (source file, TODOs, patterns)
3. Save/load metadata alongside plan files
4. Detect changes between plan versions (diff)
5. Extract patterns from user completions
6. Apply learned patterns to similar cases

**Tests**:
- `test_metadata.py::test_save_load_metadata` - Persistence
- `test_metadata.py::test_detect_user_changes` - Diff detection
- `test_metadata.py::test_extract_patterns` - Pattern extraction
- `test_metadata.py::test_apply_patterns` - Pattern application
- `test_metadata.py::test_confidence_increase` - Pattern confidence

**Deliverable**: Multi-pass workflow with pattern learning

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

