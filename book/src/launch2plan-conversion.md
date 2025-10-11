# Launch2Plan Conversion Strategy

This document describes the strategy for converting ROS 2 launch files to ROS-Plan format using the `launch2plan` tool.

## Overview

**launch2plan** is a conversion tool that transforms ROS 2 launch files (Python, XML, YAML) into ROS-Plan's declarative plan format. Unlike launch2dump which extracts metadata from a single execution path, launch2plan explores **all conditional branches** to generate a complete plan with `when` clauses.

### Key Differences from launch2dump

| Aspect           | launch2dump                                   | launch2plan                                     |
|------------------|-----------------------------------------------|-------------------------------------------------|
| **Purpose**      | Extract metadata for a specific configuration | Convert to declarative plan format              |
| **Conditionals** | Evaluates and follows only True branches      | Explores ALL branches, generates `when` clauses |
| **Output**       | JSON/YAML metadata listing nodes              | Complete ROS-Plan YAML with links and sockets   |
| **Execution**    | Single execution path                         | Multiple hypothetical paths                     |
| **Use Case**     | Inspection, debugging, integration            | Migration, modernization                        |

## Conversion Architecture

### High-Level Flow

```
┌─────────────────────┐
│  ROS 2 Launch File  │
│  (.launch.py, etc.) │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Launch Inspector   │
│  (Modified Visitor) │
│                     │
│  ✓ Visit all nodes  │
│  ✓ Track conditions │
│  ✓ Infer connections│
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   Plan Builder      │
│                     │
│  • Generate nodes   │
│  • Infer links      │
│  • Create sockets   │
│  • Map arguments    │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  ROS-Plan YAML      │
│  (plan.yaml)        │
└─────────────────────┘
```

### Visitor Modifications

The launch2plan visitor differs from launch2dump in conditional handling:

```python
# launch2dump approach (current)
def visit_action(action, context, session):
    condition = action.condition
    if condition is None or condition.evaluate(context):  # Only True branch
        return visit_action_by_class(action, context, session)
    return None

# launch2plan approach (new)
def visit_action(action, context, session):
    condition = action.condition

    # Track the condition expression
    condition_expr = extract_condition_expression(condition) if condition else None

    # Visit the action with condition context
    with session.condition_context(condition_expr):
        return visit_action_by_class(action, context, session)
```

## Conversion Components

### 1. Argument Mapping

**Challenge:** ROS 2 `DeclareLaunchArgument` → ROS-Plan `arg` section

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

**Strategy:**
- Extract argument name, default value, and description
- Infer type from default value (string → i64/f64/bool/str)
- Map to typed ROS-Plan argument format

**Type Inference Rules:**
```python
def infer_type(value: str) -> tuple[str, Any]:
    if value.lower() in ["true", "false"]:
        return "bool", value.lower() == "true"
    try:
        return "i64", int(value)
    except ValueError:
        try:
            return "f64", float(value)
        except ValueError:
            return "str", value
```

### 2. Node Conversion

**Challenge:** `launch_ros.actions.Node` → ROS-Plan `node` section

```python
# Input: ROS 2 Launch
Node(
    package="camera_driver",
    executable="camera_node",
    name="camera",
    namespace="/sensors",
    parameters=[{"fps": LaunchConfiguration("camera_fps")}],
    remappings=[("image_raw", "/camera/image")],
    condition=IfCondition(LaunchConfiguration("enable_camera"))
)

# Output: ROS-Plan
node:
  camera:
    pkg: camera_driver
    exec: camera_node
    when: $ enable_camera $
    param:
      fps: !i64 $ camera_fps $
    socket:
      image_raw: !pub
        from: /camera/image
```

**Strategy:**
- Extract package, executable, name
- Convert namespace to ROS-Plan namespacing (if applicable)
- Map parameters to typed param section
- **Infer sockets from remappings** (see section below)
- Convert conditions to `when` clauses with Lua expressions

### 3. Socket Inference

**Challenge:** Infer socket declarations from node usage

ROS-Plan requires explicit socket declarations, but launch files don't have them. We must infer sockets from:

1. **Remappings** - Indicate topic connections
2. **Topic patterns** - Publisher/subscriber detection heuristics
3. **User annotations** - Comments or naming conventions

**Inference Strategy:**

```yaml
# Pattern 1: Remapping indicates socket
Node(
    remappings=[("image_out", "/camera/image")]
)
# → Infer socket: image_out: !pub  (assumes publisher from name pattern)

# Pattern 2: Multiple nodes remapping same topic
Node(package="camera", remappings=[("image", "/shared/image")])
Node(package="processor", remappings=[("image", "/shared/image")])
# → Infer: camera/image is !pub, processor/image is !sub

# Pattern 3: Composable nodes specify topics explicitly
ComposableNode(
    remappings=[("~/input", "/data/input")]
)
# → Infer socket: input: !sub (composable nodes typically subscribe to inputs)
```

**Heuristics for Direction (pub/sub):**

```python
def infer_socket_direction(socket_name: str, context: dict) -> str:
    """Infer if socket is publisher or subscriber."""

    # Heuristic 1: Name patterns
    pub_patterns = ["out", "output", "pub", "publisher", "data", "state"]
    sub_patterns = ["in", "input", "sub", "subscriber", "cmd", "command"]

    name_lower = socket_name.lower()
    if any(p in name_lower for p in pub_patterns):
        return "!pub"
    if any(p in name_lower for p in sub_patterns):
        return "!sub"

    # Heuristic 2: Multiple nodes with same topic
    topic = context.get("remapping_target")
    if topic:
        connections = context["topic_connections"].get(topic, [])
        if len(connections) > 1:
            # If this is first node, assume publisher
            # If later nodes, assume subscribers
            if context["is_first_node_for_topic"]:
                return "!pub"
            else:
                return "!sub"

    # Default: Cannot determine, mark for manual review
    return "!pub  # FIXME: Direction unknown"
```

**Limitations:**
- Socket direction (pub/sub) cannot always be determined automatically
- Message types are not available in launch files
- Service vs topic distinction requires heuristics
- **Manual review required** for complex cases

**Mitigation:**
- Generate comments marking uncertain inferences
- Provide validation tool to check against actual node interfaces
- Allow user-provided socket hints file

### 4. Link Inference

**Challenge:** Create `link` declarations from inferred sockets and remappings

```python
# Given nodes with remappings:
Node(package="camera", remappings=[("image", "/sensors/camera/raw")])
Node(package="proc", remappings=[("input", "/sensors/camera/raw")])

# Infer link:
link:
  camera_to_proc: !pubsub
    type: sensor_msgs/msg/Image  # FIXME: Type unknown
    src: ["camera/image"]
    dst: ["proc/input"]
```

**Link Inference Algorithm:**

```python
def infer_links(nodes: list[NodeInfo]) -> list[LinkInfo]:
    """Infer links from node remappings."""

    # Group sockets by target topic
    topic_sockets = defaultdict(lambda: {"pubs": [], "subs": []})

    for node in nodes:
        for socket_name, remapping in node.remappings:
            target_topic = remapping.target
            socket_ref = f"{node.name}/{socket_name}"

            if socket.direction == "!pub":
                topic_sockets[target_topic]["pubs"].append(socket_ref)
            elif socket.direction == "!sub":
                topic_sockets[target_topic]["subs"].append(socket_ref)

    # Create links for each topic
    links = []
    for topic, sockets in topic_sockets.items():
        if sockets["pubs"] and sockets["subs"]:
            link_name = generate_link_name(topic)
            links.append(LinkInfo(
                name=link_name,
                type=None,  # Unknown, needs manual specification
                src=sockets["pubs"],
                dst=sockets["subs"],
            ))

    return links
```

**Link Naming Strategy:**
```python
def generate_link_name(topic: str) -> str:
    """Generate descriptive link name from topic."""
    # /sensors/camera/image → camera_image_link
    parts = topic.strip("/").replace("/", "_")
    return f"{parts}_link"
```

### 5. Condition Tracking

**Challenge:** Convert launch conditions to ROS-Plan `when` clauses

```python
# Input: Launch condition
GroupAction(
    actions=[...],
    condition=IfCondition(LaunchConfiguration("enable_feature"))
)

# Output: Plan when clause
when: $ enable_feature $
```

**Condition Expression Extraction:**

```python
class ConditionTracker:
    """Track and convert launch conditions to Lua expressions."""

    def __init__(self):
        self.condition_stack = []

    def extract_expression(self, condition) -> Optional[str]:
        """Extract Lua expression from launch condition."""
        if condition is None:
            return None

        if isinstance(condition, IfCondition):
            # Extract LaunchConfiguration reference
            predicate = condition._IfCondition__predicate_expression
            if isinstance(predicate, LaunchConfiguration):
                var_name = predicate.variable_name[0].text
                return f"$ {var_name} $"

        elif isinstance(condition, UnlessCondition):
            predicate = condition._UnlessCondition__predicate_expression
            if isinstance(predicate, LaunchConfiguration):
                var_name = predicate.variable_name[0].text
                return f"$ not {var_name} $"

        # Complex conditions
        return self.extract_complex_expression(condition)

    def extract_complex_expression(self, condition) -> str:
        """Handle complex boolean expressions."""
        # AndSubstitution, OrSubstitution, etc.
        # Convert to Lua: "and", "or", "not"
        # This requires deeper substitution tree traversal
        return "$ FIXME: Complex condition $"
```

**Supported Condition Types:**

| Launch Condition                            | ROS-Plan When Clause | Notes                      |
|---------------------------------------------|----------------------|----------------------------|
| `IfCondition(LaunchConfiguration("x"))`     | `when: $ x $`        | Simple boolean             |
| `UnlessCondition(LaunchConfiguration("x"))` | `when: $ not x $`    | Negation                   |
| `IfCondition(EqualsSubstitution(...))`      | `when: $ a == b $`   | Equality                   |
| Complex boolean expressions                 | `when: $ FIXME $`    | Manual conversion required |

### 6. Include Conversion

**Challenge:** Convert `IncludeLaunchDescription` to ROS-Plan `include`

```python
# Input: Launch include
IncludeLaunchDescription(
    PythonLaunchDescriptionSource("subsystem.launch.py"),
    launch_arguments={
        "fps": LaunchConfiguration("camera_fps"),
        "enable": "true"
    }
)

# Output: Plan include
include:
  subsystem: !file
    path: subsystem.yaml  # .launch.py → .yaml
    arg:
      fps: !i64 $ camera_fps $
      enable: !bool true
```

**Strategy:**
- Recursively convert included launch files
- Map launch arguments to plan arguments
- Generate include reference with converted path

### 7. Parameter Handling

**Challenge:** Convert ROS 2 parameters to typed ROS-Plan parameters

```python
# Input: Launch parameters
Node(
    parameters=[
        {"use_sim_time": False, "rate": 10},
        PathJoinSubstitution([...], "config.yaml")
    ]
)

# Output: Plan parameters
param:
  use_sim_time: !bool false
  rate: !i64 10
  __param_file: config.yaml
```

**Parameter Type Inference:**
```python
def infer_param_type(value: Any) -> tuple[str, Any]:
    """Infer ROS-Plan type tag from Python value."""
    if isinstance(value, bool):
        return "!bool", value
    elif isinstance(value, int):
        return "!i64", value
    elif isinstance(value, float):
        return "!f64", value
    elif isinstance(value, str):
        return "!str", value
    elif isinstance(value, list):
        if all(isinstance(v, bool) for v in value):
            return "!bool_list", value
        elif all(isinstance(v, int) for v in value):
            return "!i64_list", value
        # ... etc
    return "!str", str(value)  # Fallback
```

## Implementation Plan

### Phase 1: Core Visitor (Explore All Branches)

**Goal:** Modify launch2dump visitor to explore all conditional branches

**Key Changes:**

```python
class BranchExploringInspector(LaunchInspector):
    """Inspector that explores all conditional branches."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.branch_sessions = {}  # Track sessions per branch

    def explore_branches(self, action, context):
        """Visit action in all possible condition states."""
        condition = action.condition

        if condition is None:
            # No condition, visit normally
            return self.visit_single_branch(action, context, condition_expr=None)

        # Create hypothetical contexts for True and False
        true_context = context.copy()
        false_context = context.copy()

        # Force condition evaluation to True
        with mock_condition(condition, True):
            true_session = self.visit_single_branch(action, true_context,
                                                    condition_expr=extract_expression(condition))

        # Force condition evaluation to False (to explore else branches)
        # Note: In practice, we track which branches are visited with condition=True
        # and which are not visited (would be False branches)

        return true_session
```

**Files to Create:**
- `launch2plan/src/launch2plan/branch_explorer.py` - Branch exploration logic
- `launch2plan/src/launch2plan/condition_tracker.py` - Condition expression extraction

### Phase 2: Plan Builder

**Goal:** Transform collected metadata into ROS-Plan format

**Key Components:**

```python
class PlanBuilder:
    """Build ROS-Plan YAML from launch metadata."""

    def __init__(self, branch_results: BranchExplorationResult):
        self.results = branch_results
        self.socket_inferrer = SocketInferrer()
        self.link_inferrer = LinkInferrer()

    def build_plan(self) -> PlanDocument:
        """Build complete plan document."""
        plan = PlanDocument()

        # 1. Convert arguments
        plan.args = self.convert_arguments()

        # 2. Convert nodes with conditions
        plan.nodes = self.convert_nodes()

        # 3. Infer sockets from remappings
        self.infer_node_sockets(plan.nodes)

        # 4. Infer links from socket connections
        plan.links = self.infer_links(plan.nodes)

        # 5. Convert includes
        plan.includes = self.convert_includes()

        return plan
```

**Files to Create:**
- `launch2plan/src/launch2plan/plan_builder.py` - Main conversion logic
- `launch2plan/src/launch2plan/socket_inferrer.py` - Socket inference heuristics
- `launch2plan/src/launch2plan/link_inferrer.py` - Link generation from sockets
- `launch2plan/src/launch2plan/type_inference.py` - Type inference utilities

### Phase 3: YAML Generation

**Goal:** Serialize plan to ROS-Plan YAML format

```python
class PlanSerializer:
    """Serialize plan to ROS-Plan YAML."""

    def serialize(self, plan: PlanDocument) -> str:
        """Generate ROS-Plan YAML with proper formatting."""
        doc = ruamel.yaml.YAML()
        doc.default_flow_style = False

        output = {}

        # Order: arg, var, node, link, socket, include
        if plan.args:
            output["arg"] = self.serialize_args(plan.args)
        if plan.nodes:
            output["node"] = self.serialize_nodes(plan.nodes)
        if plan.links:
            output["link"] = self.serialize_links(plan.links)

        return doc.dump(output)

    def serialize_nodes(self, nodes: list[NodeInfo]) -> dict:
        """Serialize nodes with proper YAML tags."""
        result = {}
        for node in nodes:
            node_dict = {
                "pkg": node.package,
                "exec": node.executable,
            }
            if node.condition:
                node_dict["when"] = node.condition
            if node.parameters:
                node_dict["param"] = self.serialize_params(node.parameters)
            if node.sockets:
                node_dict["socket"] = self.serialize_sockets(node.sockets)

            result[node.name] = node_dict
        return result
```

**Files to Create:**
- `launch2plan/src/launch2plan/yaml_serializer.py` - YAML output generation

### Phase 4: CLI Tool

**Goal:** Command-line interface for conversion

```bash
# Convert single launch file
launch2plan camera.launch.py -o camera.yaml

# Convert with arguments
launch2plan system.launch.py fps:=30 debug:=true -o system.yaml

# Convert entire package
launch2plan --package my_robot_bringup --output plans/
```

**CLI Implementation:**

```python
def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS 2 launch files to ROS-Plan format"
    )
    parser.add_argument("launch_file", help="Path to launch file")
    parser.add_argument("-o", "--output", help="Output plan file")
    parser.add_argument("--validate", action="store_true",
                       help="Validate generated plan")
    parser.add_argument("--review", action="store_true",
                       help="Mark uncertain conversions for review")

    args = parser.parse_args()

    # Explore all branches
    explorer = BranchExploringInspector()
    results = explorer.explore_launch_file(args.launch_file)

    # Build plan
    builder = PlanBuilder(results)
    plan = builder.build_plan()

    # Generate YAML
    serializer = PlanSerializer(review_mode=args.review)
    yaml_output = serializer.serialize(plan)

    # Write output
    if args.output:
        Path(args.output).write_text(yaml_output)
    else:
        print(yaml_output)
```

**Files to Create:**
- `launch2plan/src/launch2plan/__main__.py` - CLI entry point

## Conversion Challenges and Solutions

### Challenge 1: Socket Direction Ambiguity

**Problem:** Cannot determine if socket is publisher or subscriber from launch file alone

**Solutions:**

1. **Heuristic-based inference** (see Socket Inference section)
2. **User-provided hints file:**
   ```yaml
   # socket_hints.yaml
   nodes:
     camera_driver:
       sockets:
         image: !pub
         trigger: !sub
   ```
3. **Interactive mode:** Prompt user for ambiguous cases
4. **Post-conversion validation:** Cross-reference with actual node interfaces using `ros2 node info`

### Challenge 2: Message Type Unknown

**Problem:** Launch files don't specify message types

**Solutions:**

1. **Leave type blank with FIXME comment:**
   ```yaml
   link:
     connection: !pubsub
       type: FIXME  # Type unknown, specify manually
       src: [...]
   ```

2. **Query running system:**
   ```python
   # If nodes are available, query type
   topic_type = subprocess.run(
       ["ros2", "topic", "type", topic_name],
       capture_output=True
   ).stdout.decode().strip()
   ```

3. **Type database:** Build database of common node interfaces

### Challenge 3: Complex Conditionals

**Problem:** Nested, complex boolean expressions

**Solutions:**

1. **Simplify when possible:**
   ```python
   # IfCondition(AndSubstitution([a, b]))
   # → when: $ a and b $
   ```

2. **Mark complex cases:**
   ```yaml
   when: $ FIXME: Complex condition - see original launch file line 42 $
   ```

3. **Provide conversion hints:**
   ```python
   # In generated file:
   # Original condition: IfCondition(EqualsSubstitution(LaunchConfiguration('mode'), 'sim'))
   when: $ mode == "sim" $
   ```

### Challenge 4: Dynamic Node Creation

**Problem:** Launch files with loops creating multiple nodes

```python
# Dynamic node creation
nodes = []
for i in range(num_cameras):
    nodes.append(Node(
        package="camera",
        name=f"camera_{i}",
        ...
    ))
```

**Solutions:**

1. **Expand loops at conversion time:**
   ```yaml
   node:
     camera_0: ...
     camera_1: ...
     camera_2: ...
   ```

2. **Use plan arguments with Lua expressions:**
   ```yaml
   # Convert to parameterized plan
   arg:
     camera_count:
       type: "i64"
       default: !i64 3

   # Manual step: User must create conditional nodes
   node:
     camera_0:
       when: $ camera_count >= 1 $
     camera_1:
       when: $ camera_count >= 2 $
     camera_2:
       when: $ camera_count >= 3 $
   ```

3. **Emit warning:** "Dynamic node creation detected - manual conversion required"

### Challenge 5: GroupAction Nesting

**Problem:** Nested groups with different namespaces/conditions

```python
GroupAction(
    actions=[
        PushRosNamespace("subsystem"),
        GroupAction(
            actions=[...],
            condition=IfCondition(...)
        ),
        PopRosNamespace()
    ]
)
```

**Solutions:**

1. **Track namespace stack:**
   ```python
   class NamespaceTracker:
       def __init__(self):
           self.stack = []

       def current_namespace(self) -> str:
           return "/".join(self.stack)
   ```

2. **Flatten groups when possible:**
   - Merge conditions using `and`
   - Apply namespace to all contained nodes

3. **Use plan groups:**
   ```yaml
   group:
     subsystem:
       when: $ enable_subsystem $
       node:
         ...
   ```

## Conversion Quality Levels

Generated plans have different quality levels based on automation:

### Level 1: Automatic (Green)
✅ Fully automatic conversion, no manual intervention needed
- Simple nodes with explicit configuration
- Straightforward conditionals
- Basic parameter passing

### Level 2: Assisted (Yellow)
⚠️ Automatic conversion with manual review required
- Inferred socket directions marked with comments
- Unknown message types marked FIXME
- Simple condition expressions

### Level 3: Manual (Red)
❌ Requires significant manual intervention
- Complex boolean logic in conditions
- Dynamic node creation
- Ambiguous socket connections
- Service vs topic ambiguity

**Conversion Reports:**

```yaml
# Generated plan includes metadata comment
# Conversion Report:
# - Level: Assisted (Manual review required)
# - Confidence: 75%
# - Manual review needed:
#   - Line 15: Socket direction for 'data' is inferred (check with node interface)
#   - Line 32: Message type for 'image_link' is unknown
#   - Line 48: Complex condition simplified, verify logic
```

## Testing Strategy

### Unit Tests
- Argument conversion
- Node conversion with parameters
- Condition extraction
- Socket inference heuristics
- Link generation

### Integration Tests
- Convert example launch files
- Validate generated YAML parseable by ros-plan-format
- Round-trip tests (launch → plan → execution equivalence)

### Validation Tests
- Generated plans compile with ros2plan
- Execution matches original launch behavior
- All branches reachable with appropriate arguments

## User Workflow

### Step 1: Convert
```bash
launch2plan my_robot.launch.py -o my_robot.yaml --review
```

### Step 2: Review
```yaml
# my_robot.yaml
# REVIEW REQUIRED: 3 items

node:
  camera:
    socket:
      image: !pub  # REVIEW: Direction inferred from name pattern

link:
  camera_feed: !pubsub
    type: FIXME  # REVIEW: Message type unknown
    src: [camera/image]
    dst: [processor/input]

  control: !pubsub
    when: $ FIXME: Complex condition $  # REVIEW: Simplify manually
```

### Step 3: Fix
Manually address REVIEW/FIXME items:
- Verify socket directions
- Fill in message types
- Simplify complex conditions

### Step 4: Validate
```bash
# Compile plan
ros2plan compile my_robot.yaml

# Compare execution
ros2 launch my_robot.launch.py &
ros2plan run my_robot.yaml &

# Verify same topics/services
ros2 topic list
ros2 service list
```

### Step 5: Test
```bash
# Test all branches
ros2plan compile my_robot.yaml enable_camera:bool=true
ros2plan compile my_robot.yaml enable_camera:bool=false
```

## Future Enhancements

### 1. Type Database
Build database of node interfaces:
```yaml
# node_types.yaml
camera_driver/camera_node:
  sockets:
    image_raw:
      direction: pub
      type: sensor_msgs/msg/Image
    trigger:
      direction: sub
      type: std_msgs/msg/Bool
```

### 2. Interactive Mode
```bash
launch2plan system.launch.py --interactive
# → Prompts user for ambiguous cases
# Socket 'data' on node 'sensor': publisher or subscriber? [pub/sub]:
```

### 3. Batch Conversion
```bash
launch2plan --package my_robot_bringup --recursive
# Converts all launch files in package
```

### 4. Migration Assistant
```bash
launch2plan migrate my_robot_bringup
# - Converts all launch files
# - Updates package.xml
# - Creates migration report
# - Generates validation tests
```

### 5. Round-Trip Validation
```bash
launch2plan verify original.launch.py converted.yaml
# Executes both and compares:
# - Topic list
# - Service list
# - Parameter values
# - QoS settings
```

## Conclusion

Converting ROS 2 launch files to ROS-Plan format is a multi-step process requiring:

1. **Automated conversion** for straightforward cases
2. **Heuristic inference** for missing information (sockets, types)
3. **Manual review** for ambiguous or complex scenarios
4. **Validation** to ensure behavioral equivalence

The launch2plan tool provides a foundation for migration, but **human oversight is essential** for high-quality results. The tool should guide users through the process with clear markings of what needs review.

By exploring all conditional branches and generating appropriate `when` clauses, launch2plan creates **complete, self-contained plans** that leverage ROS-Plan's type safety and explicit topology advantages.
