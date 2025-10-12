# Launch2Plan: Launch-to-Plan Conversion Tool

This document describes the design and strategy for converting ROS 2 launch files to ROS-Plan format using the `launch2plan` tool.

## Overview

**launch2plan** is a conversion tool that transforms ROS 2 launch files (Python, XML, YAML) into ROS-Plan's declarative plan format. Unlike launch2dump which extracts metadata from a single execution path, launch2plan explores **all conditional branches** to generate a complete plan with `when` clauses.

The tool uses a **multi-pass workflow** where it generates skeleton plans with TODO markers, the user fills in missing information, and subsequent passes learn from user annotations to complete remaining TODOs automatically.

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
│       ├── inference.py             # Socket/link inference + pattern learning
│       ├── builder.py               # Plan YAML construction
│       ├── metadata.py              # Multi-pass state tracking
│       └── validator.py             # Plan compilation validation
├── tests/
│   ├── fixtures/                    # Test launch files
│   ├── test_visitor.py
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

### F62: Socket and Link Inference with TODO Stubs

**Challenge**: Launch files don't specify socket directions (!pub vs !sub) or message types.

**Pass 1: TODO Stub Generation**

Generate skeleton with helpful hints:

```yaml
node:
  camera:
    pkg: image_tools
    exec: cam2image
    socket:
      # TODO: Specify direction (!pub or !sub)
      # Remapping: image -> /camera/image_raw
      # Heuristic: Topic name suggests this might be !pub (confidence: 0.6)
      image: !todo
```

**Heuristic Inference**: Use pattern matching for hints:

```python
class SocketInference:
    PUB_PATTERNS = [
        r"publisher", r"_pub$", r"output", r"writer",
        r"^image$", r"^camera", r"sensor"
    ]

    SUB_PATTERNS = [
        r"subscriber", r"_sub$", r"input", r"reader", r"listener"
    ]

    def infer_direction(self, topic_name, node_name):
        # Score based on pattern matches
        pub_score = sum(1 for p in self.PUB_PATTERNS if re.search(p, topic_name))
        sub_score = sum(1 for p in self.SUB_PATTERNS if re.search(p, topic_name))

        if pub_score > sub_score:
            return ("!pub", confidence)
        elif sub_score > pub_score:
            return ("!sub", confidence)
        return (None, 0.0)
```

**Pass 2: Pattern Learning**

Learn from user changes:

```python
@dataclass
class LearnedPattern:
    pattern_type: str      # "socket_direction", "message_type"
    matcher: str           # Topic/node name pattern
    value: Any            # !pub, !sub, msg type
    confidence: float      # 0.0-1.0
    examples: List[str]    # Where it was seen

class PatternLearner:
    def learn_from_changes(self, old_plan, new_plan):
        # Detect what user changed
        changes = diff_plans(old_plan, new_plan)

        for change in changes:
            if change.field == "socket_direction":
                pattern = self.generalize(change.location)
                self.add_pattern(pattern, change.value)

    def apply_patterns(self, pending_todos):
        for todo in pending_todos:
            pattern = self.find_matching_pattern(todo.location)
            if pattern and pattern.confidence >= 0.8:
                todo.inferred_value = pattern.value
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
# From remappings, infer sockets
remappings = [
    ('image_raw', '/camera/image_raw'),  # Likely !pub (sensor output)
    ('cmd_vel', '/robot/cmd_vel'),       # Could be !pub or !sub
]

# Generate with hints
socket:
  # TODO: !pub or !sub?
  # Remapping: image_raw -> /camera/image_raw
  # Hint: Sensor topics are usually !pub
  image_raw: !todo
```

### 4. Link Inference

```python
# Discover topics from multiple nodes
node1: remaps 'chatter' -> '/demo/chatter'
node2: remaps 'chatter' -> '/demo/chatter'

# Generate link
link:
  demo_chatter: !pubsub
    type: "TODO"  # Message type unknown
    # TODO: Specify std_msgs/msg/String or other
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

### Pass 1: Initial Conversion

```bash
$ launch2plan convert robot.launch.py
✓ Plan saved to: robot.plan.yaml
✓ Metadata saved to: robot.plan.meta.json
✓ Discovered 2 nodes
⚠ 3 TODOs need completion:
  - node.camera.socket.image_raw (direction): Heuristic suggests !pub (confidence: 0.7)
  - node.detector.socket.image (direction): Unable to determine
  - link.camera_image.type (type): Heuristic suggests sensor_msgs/msg/Image (confidence: 0.7)
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
      # TODO: Specify direction (!pub or !sub)
      # Remapping: image_raw -> /camera/image
      # Heuristic: Sensor output suggests !pub (confidence: 0.7)
      image_raw: !todo

  detector:
    pkg: processing_pkg
    exec: detector
    socket:
      # TODO: Specify direction (!pub or !sub)
      # Remapping: image -> /camera/image
      # Unable to infer direction
      image: !todo

      # TODO: Specify direction (!pub or !sub)
      # Remapping: detections -> /objects
      # Heuristic: Output suggests !pub (confidence: 0.5)
      detections: !todo

link:
  camera_image: !pubsub
    # TODO: Specify message type
    # Heuristic suggests: sensor_msgs/msg/Image (confidence: 0.7)
    type: "TODO"
```

### User Annotation

User edits `robot.plan.yaml`:

```yaml
node:
  camera:
    socket:
      image_raw: !pub  # ← User fills in

  detector:
    socket:
      image: !sub       # ← User fills in
      detections: !pub  # ← User fills in

link:
  camera_image: !pubsub
    type: "sensor_msgs/msg/Image"  # ← User fills in
```

### Pass 2: Refinement

```bash
$ launch2plan refine robot.plan.yaml
Detecting user changes...
✓ Learned 2 patterns:
  - socket_direction: image_raw -> !pub (confidence: 0.70)
  - message_type: image -> sensor_msgs/msg/Image (confidence: 0.70)
✓ Applied patterns to pending TODOs
  Completion rate: 100.0%
  Completed: 3
  Pending: 0

✓ All TODOs completed! Ready to validate
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
- Heuristic hints based on topic/node names
- TODO markers with suggestions
- Pattern learning across multiple nodes

### Challenge 2: Message Type Unknown

**Problem**: Launch files don't specify message types.

**Solution**:
- Common type database (image → sensor_msgs/msg/Image)
- Leave as TODO with hints
- Require user specification or use runtime introspection

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

### Phase 12.1: Foundation
- Basic visitor infrastructure
- CLI skeleton with `convert` command
- Simple node discovery

### Phase 12.2: Branch Exploration
- Conditional branch exploration
- When clause generation
- Cycle detection

### Phase 12.3: Socket Inference
- Heuristic-based hints
- TODO stub generation
- Metadata tracking

### Phase 12.4: Pattern Learning
- Diff detection between passes
- Pattern extraction from changes
- Auto-completion with confidence thresholds

### Phase 12.5: Plan Builder
- Complete YAML generation
- Include preservation
- Argument forwarding

### Phase 12.6: Testing & Validation
- Comprehensive test suite
- Always-compile validation
- Example conversions

## Limitations and Future Work

### Current Limitations

1. **Dynamic Behavior**: Cannot analyze `OpaqueFunction` or runtime-evaluated code
2. **Message Types**: Requires user specification or introspection
3. **Complex Conditions**: Some Python expressions may not convert cleanly to Lua
4. **Implicit Connections**: Topic connections not explicitly declared in launch files

### Future Enhancements

1. **Runtime Introspection**: Query running nodes for message types
2. **XML/YAML Launch Support**: Extend beyond Python launch files
3. **Interactive Mode**: Guide user through TODO completion
4. **Template Library**: Pre-built patterns for common ROS packages
5. **Diff Visualization**: Better UI for viewing changes between passes

## References

- **launch2dump**: Node metadata extraction tool (same codebase)
- **ROS 2 Launch**: Python launch file documentation
- **ROS-Plan Format**: Plan YAML specification
- **ros2plan**: Plan compiler and runtime

