# Design Rationale

This chapter explains the design philosophy and decisions behind ROS-Plan's core concepts: nodes, links, and sockets.

## Design Philosophy

ROS-Plan is designed around a core principle: **plans are self-contained programs with explicit typed endpoints**.

This philosophy drives several key design decisions:

1. **Explicit over implicit**: Interfaces should be declared, not inferred
2. **Type safety**: Catch connection errors at compile time, not runtime
3. **Modularity**: Plans and subplans compose like functions with clear contracts
4. **Error prevention**: Make incorrect configurations hard to write

### Motivation: Beyond Topic Names

Traditional ROS 2 launch files rely on topic names for connections:

```python
# Traditional ROS 2 launch (error-prone)
Node(package='camera_driver', executable='camera_node',
     remappings=[('image', '/sensors/camera/image')])

Node(package='image_proc', executable='processor',
     remappings=[('input', '/sensors/camera/image')])  # Must match exactly!
```

**Problems with this approach:**
- **String matching**: Typos cause silent disconnections
- **No type checking**: Incompatible types discovered at runtime
- **Scattered information**: Connection topology hidden across many nodes
- **No QoS control**: Quality of Service settings are implicit
- **Refactoring hazards**: Renaming topics requires coordinating changes across files

ROS-Plan solves these problems with **explicit link declarations**:

```yaml
link:
  camera_feed: !pubsub
    type: sensor_msgs/msg/Image      # Type checked at compile time
    qos:
      profile:
        depth: 10
        reliability: reliable
    src: ["camera/image_out"]        # Explicit source
    dst: ["processor/image_in"]      # Explicit destination
```

Benefits:
- ✅ **Type safety**: Message type mismatches caught at compile time
- ✅ **Explicit topology**: All connections visible in one place
- ✅ **QoS control**: Quality of Service explicitly configured
- ✅ **Refactor-safe**: Rename sockets without breaking connections
- ✅ **Validation**: Comprehensive checking before deployment

## Architecture: Plans as Programs

### The Interface Model

ROS-Plan treats plans and nodes uniformly through the **socket interface**:

```
┌─────────────────────────────────────┐
│              Plan                   │
│                                     │
│  ┌──────┐         ┌──────┐         │
│  │Node A│         │Node B│         │
│  │  out●│────────>│●in   │         │
│  └──────┘         └──────┘         │
│     │                   │           │
│     └──────┐    ┌───────┘           │
│            ▼    ▼                   │
│         Plan Sockets                │
│         ●output  ●input             │
└─────────────────────────────────────┘
```

Both plans and nodes:
- Have **sockets** (typed endpoints)
- Can be **connected** through links
- Declare **QoS requirements**
- Are **composable** through the same interface

This enables hierarchical composition:

```yaml
# Parent plan
include:
  camera_subsystem: !file
    path: camera.yaml

  processing_subsystem: !file
    path: processing.yaml

link:
  data_flow: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera_subsystem/output"]     # Plan socket
    dst: ["processing_subsystem/input"]  # Plan socket
```

### Self-Contained Plans

Each plan declares its interface through sockets:

```yaml
# camera.yaml - Self-contained camera subsystem
socket:
  output: !pub
    type: sensor_msgs/msg/Image
    qos: {require: {reliability: reliable}}

node:
  camera_driver:
    pkg: camera_driver
    exec: driver_node
    socket:
      raw_image: !pub

link:
  internal: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera_driver/raw_image"]
    dst: ["output"]  # Maps to plan socket
```

This design:
- Makes the plan's **public API** explicit
- Hides **internal implementation** details
- Enables **black-box composition**
- Supports **interface versioning**

## Socket Design Decisions

### Question 1: Explicit vs Inferred Socket Declaration

**Decision: Sockets must be explicitly declared**

#### Explicit Declaration (Chosen)

```yaml
node:
  camera:
    socket:
      image_out: !pub
        type: sensor_msgs/msg/Image
        qos: {require: {reliability: reliable}}

link:
  image_feed: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera/image_out"]
    dst: ["processor/image_in"]
```

#### Alternative: Inferred from Links (Rejected)

```yaml
node:
  camera:
    # No socket declaration

link:
  image_feed: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera/image_out"]  # Implicitly creates socket
    dst: ["processor/image_in"]
```

#### Rationale

**Explicit sockets are chosen because:**

1. **Self-Documenting Interface**
   ```yaml
   # Node's API is clear from its declaration
   node:
     sensor:
       socket:
         data: !pub          # I provide data
         config: !srv        # I accept configuration
         trigger: !cli       # I call trigger service
   ```

2. **Early Type Checking**
   - Socket types validated when node is defined
   - Mismatches caught before link resolution
   - Better error messages with clear ownership

3. **QoS Requirements at the Boundary**
   ```yaml
   socket:
     critical_data: !pub
       type: std_msgs/msg/Float64
       qos:
         require:
           reliability: reliable
           depth: 100
   ```
   The socket (endpoint) declares what it needs; the link provides it.

4. **Reusability and Multiple Connections**
   ```yaml
   # One socket, multiple links
   socket:
     sensor_data: !pub
       type: sensor_msgs/msg/LaserScan

   link:
     to_slam: !pubsub
       src: ["lidar/sensor_data"]
       dst: ["slam/scan"]

     to_viz: !pubsub
       src: ["lidar/sensor_data"]
       dst: ["rviz/scan"]

     to_logger: !pubsub
       src: ["lidar/sensor_data"]
       dst: ["logger/scan"]
   ```

5. **Encapsulation**
   - Node knows its own interface
   - Interface is independent of how node is used
   - Follows principle of information hiding

6. **Matches "Plans as Programs" Philosophy**
   - Like function signatures in programming languages
   - The contract is at the boundary, not at call sites

### Question 2: Message Type Location

**Decision: Hybrid approach with link as source of truth**

#### Type Declaration Rules

1. **Link type is REQUIRED** (source of truth)
2. **Socket type is OPTIONAL** (for validation)
3. **When both present, they must match**
4. **Socket types are RECOMMENDED for plan-level sockets**

#### Example: Recommended Style (Both Specified)

```yaml
node:
  camera:
    socket:
      image_out: !pub
        type: sensor_msgs/msg/Image
        qos: {require: {reliability: reliable}}

link:
  image_feed: !pubsub
    type: sensor_msgs/msg/Image  # Must match socket type
    qos:
      profile:
        depth: 10
        reliability: reliable
    src: ["camera/image_out"]
    dst: ["processor/image_in"]
```

#### Example: Minimal Style (Link Only)

```yaml
node:
  camera:
    socket:
      image_out: !pub  # Type will be provided by link

link:
  image_feed: !pubsub
    type: sensor_msgs/msg/Image  # Provides type
    src: ["camera/image_out"]
    dst: ["processor/image_in"]
```

#### Rationale

**This hybrid approach balances multiple concerns:**

1. **Link as Source of Truth**
   - The link represents the actual connection/wire
   - Type is a property of the data flowing through the connection
   - Single place to look for the definitive type

2. **Socket Types for Validation**
   - When specified, socket type validates compatibility
   - Acts as a contract checker
   - Provides early error detection

3. **Flexibility for Different Use Cases**

   **Plan-level sockets (public API):**
   ```yaml
   # Types REQUIRED - this is the plan's contract
   socket:
     camera_output: !pub
       type: sensor_msgs/msg/Image
       qos: {require: {reliability: reliable}}
   ```

   **Node-level sockets (internal):**
   ```yaml
   # Types OPTIONAL - implementation detail
   node:
     internal_processor:
       socket:
         temp_data: !pub  # Type from link
   ```

4. **Validation Hierarchy**
   ```
   If socket has type:
     ✓ Validate: link type == socket type
   Else:
     ✓ Accept: link provides type

   Result: Link type is always known and consistent
   ```

#### Type Inference Example

```yaml
socket:
  # Plan socket declares type
  output: !pub
    type: sensor_msgs/msg/Image

node:
  camera:
    socket:
      raw: !pub
        type: sensor_msgs/msg/Image

  processor:
    socket:
      processed: !pub
        # Type inferred from link

link:
  camera_to_processor: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera/raw"]
    dst: ["processor/processed"]  # Gets type from link

  processor_to_output: !pubsub
    type: sensor_msgs/msg/Image
    src: ["processor/processed"]
    dst: ["output"]  # Validates against plan socket
```

Compilation validates:
- `camera/raw` type matches link type ✓
- `processor/processed` receives type from link ✓
- `output` type matches link type ✓

## QoS Design

Quality of Service has a two-level design:

### Socket QoS Requirements

Sockets declare **requirements** (what they need):

```yaml
socket:
  critical_data: !pub
    type: std_msgs/msg/Float64
    qos:
      require:
        reliability: reliable
        min_depth: 10
```

### Link QoS Profile

Links provide **profiles** (what they deliver):

```yaml
link:
  critical_connection: !pubsub
    type: std_msgs/msg/Float64
    qos:
      profile:
        reliability: reliable
        depth: 50
    src: ["sensor/critical_data"]
    dst: ["controller/input"]
```

### Validation

At compile time:
```
For each connected socket:
  ✓ Check: link QoS profile satisfies socket QoS requirements
```

Example validation:
```yaml
# Socket requires reliable, depth >= 10
socket:
  data: !pub
    qos:
      require:
        reliability: reliable
        min_depth: 10

# Link provides reliable, depth = 50 ✓
link:
  connection: !pubsub
    qos:
      profile:
        reliability: reliable
        depth: 50
    src: ["node/data"]
    dst: ["consumer/input"]

# Link provides best-effort ✗ COMPILE ERROR
link:
  bad_connection: !pubsub
    qos:
      profile:
        reliability: best-effort  # Doesn't satisfy requirement!
        depth: 50
    src: ["node/data"]
    dst: ["consumer/input"]
```

### Design Benefits

1. **Separation of Concerns**
   - Sockets know what they need
   - Links know what they provide
   - Compiler validates compatibility

2. **Flexibility**
   - Socket requirements are constraints
   - Link profiles can exceed requirements
   - Allows optimization at link level

3. **Safety**
   - Prevents QoS mismatches
   - Catches performance issues early
   - Makes guarantees explicit

## Link Design

### Explicit Connection Topology

Links make all connections visible and verifiable:

```yaml
link:
  sensor_fusion: !pubsub
    type: sensor_msgs/msg/LaserScan
    qos:
      profile:
        reliability: reliable
        depth: 10
    src:
      - "lidar_front/scan"
      - "lidar_rear/scan"
    dst:
      - "fusion/scan_front"
      - "fusion/scan_rear"
      - "logger/all_scans"
    when: $ enable_fusion $
```

This design enables:

1. **Multiple Publishers**
   ```yaml
   src:
     - "camera_1/image"
     - "camera_2/image"
     - "camera_3/image"
   ```

2. **Multiple Subscribers**
   ```yaml
   dst:
     - "processor/input"
     - "recorder/raw"
     - "debug_viewer/stream"
   ```

3. **Conditional Connections**
   ```yaml
   when: $ environment == "production" $
   ```

4. **Centralized QoS**
   - All participants use the same QoS profile
   - No conflicting settings
   - Easy to adjust for entire connection

### Service Links

Service connections follow the same pattern:

```yaml
link:
  configuration_service: !service
    type: std_srvs/srv/SetParameters
    listen: "config_server/service"
    connect:
      - "node_a/client"
      - "node_b/client"
      - "diagnostics/client"
```

Service semantics:
- One `listen` (server)
- Multiple `connect` (clients)
- No QoS (services have fixed QoS)

## Composition Example

Putting it all together:

```yaml
# Complete example showing design principles

# Plan's public interface (types required)
socket:
  robot_state: !pub
    type: nav_msgs/msg/Odometry
    qos:
      require:
        reliability: reliable

  command_input: !sub
    type: geometry_msgs/msg/Twist
    qos:
      require:
        reliability: reliable

# Arguments for configuration
arg:
  use_simulation:
    type: "bool"
    default: !bool false

  max_speed:
    type: "f64"
    default: !f64 1.0

# Internal nodes
node:
  driver:
    pkg: $ use_simulation and "sim_driver" or "hw_driver" $
    exec: driver_node
    param:
      max_speed: !f64 $ max_speed $
    socket:
      odom: !pub
        type: nav_msgs/msg/Odometry
      cmd: !sub
        type: geometry_msgs/msg/Twist

  safety_monitor:
    pkg: safety
    exec: monitor_node
    when: $ not use_simulation $  # Only in real hardware
    socket:
      odom_in: !sub
      cmd_in: !sub
      cmd_out: !pub
      estop: !srv

# Internal connections
link:
  odom_to_monitor: !pubsub
    type: nav_msgs/msg/Odometry
    qos:
      profile:
        reliability: reliable
        depth: 10
    src: ["driver/odom"]
    dst: ["safety_monitor/odom_in", "robot_state"]

  cmd_through_safety: !pubsub
    type: geometry_msgs/msg/Twist
    when: $ not use_simulation $
    src: ["command_input"]
    dst: ["safety_monitor/cmd_in"]

  safe_cmd_to_driver: !pubsub
    type: geometry_msgs/msg/Twist
    when: $ not use_simulation $
    src: ["safety_monitor/cmd_out"]
    dst: ["driver/cmd"]

  cmd_direct: !pubsub
    type: geometry_msgs/msg/Twist
    when: $ use_simulation $
    src: ["command_input"]
    dst: ["driver/cmd"]
```

This example demonstrates:
- ✅ Explicit typed interfaces (plan sockets)
- ✅ Self-contained design (internal implementation hidden)
- ✅ Type safety (all types validated)
- ✅ QoS requirements and profiles
- ✅ Conditional compilation
- ✅ Parameter-driven configuration
- ✅ Composable subsystems

## Benefits Summary

The ROS-Plan design provides:

### Safety
- **Compile-time type checking**: Catch errors before deployment
- **QoS validation**: Ensure performance guarantees
- **Connection verification**: No silent disconnections
- **Contract enforcement**: Sockets and links must agree

### Clarity
- **Explicit interfaces**: Plan APIs are self-documenting
- **Visible topology**: All connections in one place
- **Clear ownership**: Each element has a defined purpose
- **Traceable data flow**: Follow data through explicit links

### Modularity
- **Composable plans**: Plans include other plans uniformly
- **Encapsulation**: Internal details hidden behind sockets
- **Reusability**: Plans are self-contained, portable units
- **Versioning**: Interfaces can evolve with validation

### Maintainability
- **Refactor-safe**: Change internals without breaking interface
- **Searchable**: Find all uses of a socket or link
- **Analyzable**: Static analysis of connection graph
- **Testable**: Validate configurations before runtime

## Comparison with ROS 2 Launch

| Aspect | ROS 2 Launch | ROS-Plan |
|--------|--------------|----------|
| **Connection Method** | Topic name strings | Explicit typed links |
| **Type Checking** | Runtime | Compile-time |
| **QoS Control** | Implicit/scattered | Explicit in links |
| **Interface Definition** | Implicit | Explicit sockets |
| **Composition** | Launch file includes | Typed plan includes |
| **Validation** | Runtime errors | Compile-time errors |
| **Refactoring** | Error-prone | Safe |
| **Documentation** | External | Self-documenting |

## Design Principles Summary

1. **Explicit over Implicit**
   - Declare interfaces, don't infer them
   - Make connections visible
   - Type everything important

2. **Self-Contained Programs**
   - Plans declare their interfaces
   - Plans hide their implementations
   - Plans compose hierarchically

3. **Type Safety First**
   - Validate at compile time
   - Catch errors early
   - Make invalid states unrepresentable

4. **QoS as First-Class Citizen**
   - Requirements on sockets
   - Profiles on links
   - Validation at compile time

5. **Modular Composition**
   - Plans and nodes share interfaces
   - Black-box composition
   - Hierarchical systems

These principles guide all design decisions in ROS-Plan, creating a system that is safe, clear, and maintainable while supporting complex robotic systems.
