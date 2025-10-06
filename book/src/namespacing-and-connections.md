# Namespacing and Connections

This chapter describes how ROS-Plan handles node namespacing, link connections, topic naming, and plan encapsulation.

## Core Principles

1. **One Link = One ROS Topic**: Each link corresponds to exactly one ROS 2 topic
2. **Source-Based Naming**: Topic names derived from publisher namespace (Autoware convention)
3. **Plan Encapsulation**: Plans are opaque boundaries with explicit interfaces
4. **Configurable Transparency**: Plans can expose internals when appropriate
5. **Self-Contained Plans**: Subplans can be compiled and tested independently

## Namespacing

### Node Namespaces

Nodes receive hierarchical namespaces based on their location in the plan tree:

```
/include_path/node_name
```

**Example:**
```yaml
# root.yaml
include:
  sensing: !file {path: sensing.yaml}

# sensing.yaml (at /sensing)
include:
  camera: !file {path: camera.yaml}

# camera.yaml (at /sensing/camera)
node:
  driver:
    pkg: camera_driver
    exec: driver_node

# ROS node name: /sensing/camera/driver
```

### Socket Namespaces

Sockets live in their containing node or plan namespace:

```yaml
# Node socket
node:
  driver:
    socket:
      output: !pub

# Socket path: /sensing/camera/driver/output
```

```yaml
# Plan socket
socket:
  camera_out: !pub
    src: [driver/output]

# Socket path: /sensing/camera/camera_out
```

## Topic Name Resolution

### Single Source Links (Most Common)

When a link has one source, the topic name is **derived from the source socket**:

```yaml
link:
  camera_feed: !pubsub
    type: sensor_msgs/msg/Image
    src: [sensing/camera/driver/output]
    dst: [perception/detector/input]
```

**Topic Resolution:**
```
1. Source socket: sensing/camera/driver/output
2. Node namespace: /sensing/camera/driver
3. Socket ros_name: output (or override if specified)
4. ROS Topic: /sensing/camera/driver/output
```

**ROS Mapping:**
```
Publisher:  /sensing/camera/driver     publishes to → /sensing/camera/driver/output
Subscriber: /perception/detector       subscribes to → /sensing/camera/driver/output
```

### Multiple Source Links (Requires Explicit Topic)

When a link has multiple sources, the topic name **must be explicitly specified**:

```yaml
link:
  tf_broadcast: !pubsub
    type: tf2_msgs/msg/TFMessage
    topic: /tf  # REQUIRED for multiple sources
    src:
      - sensing/camera/driver/tf
      - sensing/lidar/driver/tf
      - localization/odom_publisher/tf
    dst:
      - planning/path_planner/tf_in
      - visualization/rviz/tf_in
```

**ROS Mapping:**
```
Publisher:  /sensing/camera/driver         publishes to → /tf
Publisher:  /sensing/lidar/driver          publishes to → /tf
Publisher:  /localization/odom_publisher   publishes to → /tf
Subscriber: /planning/path_planner         subscribes to → /tf
Subscriber: /visualization/rviz            subscribes to → /tf
```

### Explicit Topic Override

Links can always override the default topic derivation:

```yaml
link:
  camera_feed: !pubsub
    type: sensor_msgs/msg/Image
    topic: /sensors/front_camera  # Explicit override
    src: [sensing/camera/driver/output]
    dst: [perception/detector/input]

# ROS Topic: /sensors/front_camera (not /sensing/camera/driver/output)
```

### Topic Path Resolution

Topic names can be **absolute** (starting with `/`) or **relative**:

```yaml
# Absolute path
link:
  global_tf: !pubsub
    topic: /tf  # Always /tf regardless of link location
    src: [...]

# Relative path
link:
  local_data: !pubsub
    topic: diagnostics  # Becomes /link_namespace/diagnostics
    src: [...]
```

## Socket Naming Control

### Node Socket `ros_name`

Node sockets can override their name for ROS topic derivation:

```yaml
node:
  driver:
    socket:
      raw_output: !pub
        type: sensor_msgs/msg/Image
        ros_name: image_raw  # Use "image_raw" instead of "raw_output"

# When referenced in a link:
link:
  feed: !pubsub
    src: [driver/raw_output]
    # Topic: /namespace/driver/image_raw (not /namespace/driver/raw_output)
```

**Purpose:** Align internal socket names with ROS conventions without renaming sockets.

**Example - Standard ROS Names:**
```yaml
node:
  camera:
    socket:
      img: !pub
        ros_name: image_raw  # Follow image_raw convention

      cam_info: !pub
        ros_name: camera_info  # Follow camera_info convention
```

### Plan Socket `topic`

Plan sockets with multiple sources can specify the topic name:

```yaml
socket:
  tf_output: !pub
    type: tf2_msgs/msg/TFMessage
    src: [camera/driver/tf, lidar/driver/tf]
    topic: /tf  # Aggregate multiple sources to /tf

# When this plan socket is used:
# Both camera and lidar publish to /tf
```

**Purpose:** Create unified topics from multiple internal publishers.

## Plan Encapsulation

### Opacity by Default

Plans create **encapsulation boundaries**. Parent plans can only reference:

1. **Direct child nodes** (siblings in the same scope)
2. **Direct child plan sockets** (exposed interface)

**Cannot reference:**
- Nodes inside subplans (grandchildren)
- Sockets inside subplans (grandchild sockets)

```yaml
# main.yaml
include:
  sensing: !file {path: sensing.yaml}

node:
  local_node:
    socket:
      output: !pub

link:
  # ✅ Valid: Reference sibling node
  local: !pubsub
    src: [local_node/output]
    dst: [other_node/input]

  # ✅ Valid: Reference child plan socket
  from_subplan: !pubsub
    src: [sensing/camera_out]  # sensing exposes camera_out
    dst: [processor/input]

  # ❌ Invalid: Cannot see inside subplan
  bad: !pubsub
    src: [sensing/camera/driver/output]  # ERROR: camera/driver not visible
    dst: [processor/input]
```

### Socket Forwarding

Subplans expose internal sockets through plan-level socket declarations:

```yaml
# sensing.yaml
node:
  camera_driver:
    socket:
      image: !pub
        type: sensor_msgs/msg/Image

  lidar_driver:
    socket:
      points: !pub
        type: sensor_msgs/msg/PointCloud2

# PUBLIC INTERFACE: Exposed sockets
socket:
  camera_out: !pub
    type: sensor_msgs/msg/Image
    src: [camera_driver/image]  # Forward internal socket

  lidar_out: !pub
    type: sensor_msgs/msg/PointCloud2
    src: [lidar_driver/points]
```

**Parent usage:**
```yaml
# main.yaml
link:
  camera_feed: !pubsub
    src: [sensing/camera_out]  # Uses exposed socket
    dst: [perception/input]
```

### Plan Sockets with Multiple Sources

Plan sockets can aggregate multiple internal sources and specify a topic:

```yaml
# sensing.yaml
socket:
  tf: !pub
    type: tf2_msgs/msg/TFMessage
    src: [camera/driver/tf, lidar/driver/tf, gnss/driver/tf]
    topic: /tf  # All sources publish to /tf

# This creates the /tf topic at the plan boundary
# When compiled standalone: Camera, lidar, gnss publish to /tf
# When included in parent: Still publishes to /tf, parent can add more sources
```

### Self-Contained Plans

Plans should be compilable and testable in isolation:

```bash
# Compile subplan independently
ros2plan compile sensing.yaml -o sensing_standalone.yaml

# Creates /tf topic from camera + lidar + gnss
# No parent needed
```

**Design principle:** Subplans specify their own topic names (especially for well-known topics like `/tf`, `/clock`).

## Transparent Includes

### The Socket Explosion Problem

Strict encapsulation causes **socket explosion** in deep hierarchies:

```
root/
├── sensing/
│   ├── camera/
│   │   ├── front/ (node: image, info, tf)
│   │   └── rear/ (node: image, info, tf)
│   └── lidar/
│       ├── front/ (node: points, intensity, tf)
│       └── rear/ (node: points, intensity, tf)
```

With strict opacity, each level must forward all sockets:
- `camera/front.yaml`: 3 sockets
- `camera.yaml`: 6 sockets (2 cameras × 3)
- `sensing.yaml`: 12+ sockets (cameras + lidars)

This is **unmaintainable** for large systems.

### Transparent Include Solution

Plans can mark includes as `transparent` to expose their internal structure:

```yaml
include:
  camera: !file
    path: camera.yaml
    transparent: true  # Parent can see inside camera

  processor: !file
    path: processor.yaml
    # transparent: false (default) - opaque
```

**With transparency:**
```yaml
# sensing.yaml
include:
  camera: !file
    path: camera.yaml
    transparent: true

# Can reference nested sockets directly
link:
  tf: !pubsub
    topic: /tf
    src:
      - camera/front/driver/tf  # Direct reference through transparent plan
      - camera/rear/driver/tf
    dst: []

# No need to forward sockets!
```

### Transparency Rules

1. **Default:** Plans are opaque (`transparent: false`)
2. **Transitive:** Transparency propagates through the tree
3. **Selective:** Each include decides independently
4. **Interface priority:** Explicit plan sockets take precedence over transparent access

**Example:**
```yaml
# root.yaml
include:
  sensing: !file
    path: sensing.yaml
    transparent: true  # Can see all of sensing's internals

link:
  direct_connection: !pubsub
    type: sensor_msgs/msg/Image
    src: [sensing/camera/front/driver/output]  # Deep reference OK
    dst: [processing/fusion/input]
```

### When to Use Transparency

| Scenario | Transparency | Rationale |
|----------|--------------|-----------|
| **Single project, owned code** | `transparent: true` | No boilerplate, full control |
| **Third-party libraries** | `transparent: false` | Use documented interface |
| **Published packages** | `transparent: false` | Stable API, encapsulation |
| **Development phase** | `transparent: true` | Faster iteration |
| **Production release** | `transparent: false` | Explicit interface |

### Mixed Transparency Example

```yaml
# main.yaml
include:
  # Our sensing stack - we own it
  sensing: !file
    path: sensing.yaml
    transparent: true

  # Third-party perception library
  perception_lib: !file
    path: external/perception.yaml
    transparent: false  # Must use its interface

link:
  # Can wire deep into our code
  camera_feed: !pubsub
    type: sensor_msgs/msg/Image
    src: [sensing/camera/front/driver/image]  # OK: transparent
    dst: [perception_lib/camera_input]  # Must use plan socket
```

## Complete Examples

### Example 1: Simple Connection (1-to-1)

```yaml
# system.yaml
include:
  camera: !file {path: camera.yaml}
  processor: !file {path: processor.yaml}

link:
  image_feed: !pubsub
    type: sensor_msgs/msg/Image
    src: [camera/output]  # camera exposes 'output' socket
    dst: [processor/input]

# ROS Topic: /camera/output (derived from source)
```

### Example 2: Broadcast (1-to-many)

```yaml
link:
  lidar_points: !pubsub
    type: sensor_msgs/msg/PointCloud2
    src: [lidar/points]
    dst:
      - localization/scan_in
      - mapping/points_in
      - perception/obstacles_in

# One topic: /lidar/points
# Three subscribers
```

### Example 3: TF Aggregation (many-to-many)

```yaml
# sensing.yaml
socket:
  tf: !pub
    type: tf2_msgs/msg/TFMessage
    src: [camera/driver/tf, lidar/driver/tf, gnss/driver/tf]
    topic: /tf  # Explicit for multiple sources

# All three nodes publish to /tf
```

```yaml
# main.yaml
link:
  tf_system: !pubsub
    type: tf2_msgs/msg/TFMessage
    topic: /tf  # Add more sources
    src: [sensing/tf, localization/tf_out]
    dst: [planning/tf_in, visualization/tf_in]

# Five publishers total (sensing's 3 + localization)
# Two subscribers
# All use /tf
```

### Example 4: Transparent Deep Connection

```yaml
# main.yaml
include:
  sensing: !file
    path: sensing.yaml
    transparent: true

link:
  # Direct connection to nested node
  camera: !pubsub
    type: sensor_msgs/msg/Image
    src: [sensing/camera/front/driver/output]
    dst: [perception/detector/camera_in]

# ROS Topic: /sensing/camera/front/driver/output
```

### Example 5: Autoware-Style Sensing

```yaml
# sensing.yaml
include:
  camera:
    front: !file
      path: camera_instance.yaml
      transparent: true
    rear: !file
      path: camera_instance.yaml
      transparent: true

# Expose with clean names
socket:
  camera_front: !pub
    type: sensor_msgs/msg/Image
    src: [camera/front/driver/output]
    ros_name: camera_front  # Clean name

  camera_rear: !pub
    type: sensor_msgs/msg/Image
    src: [camera/rear/driver/output]
    ros_name: camera_rear

# Internal TF aggregation
link:
  tf: !pubsub
    type: tf2_msgs/msg/TFMessage
    topic: /tf
    src:
      - camera/front/driver/tf
      - camera/rear/driver/tf
    dst: []
```

```yaml
# main.yaml
link:
  front_camera: !pubsub
    type: sensor_msgs/msg/Image
    src: [sensing/camera_front]  # Clean interface
    dst: [perception/front_in]

# ROS Topic: /sensing/camera_front
```

## Topic Resolution Algorithm

```python
def resolve_topic_name(link, link_namespace):
    """
    Resolve a link to its ROS topic name.

    Returns: str (ROS topic name)
    """

    # Priority 1: Explicit link topic
    if link.topic is not None:
        if link.topic.startswith('/'):
            return link.topic  # Absolute path
        else:
            return f"{link_namespace}/{link.topic}"  # Relative path

    # Priority 2: Single source derivation
    if len(link.src) == 1:
        socket = resolve_socket_reference(link.src[0], link_namespace)

        # Get effective topic name
        if isinstance(socket, PlanSocket):
            # Plan socket may specify topic
            if socket.topic:
                return resolve_absolute_or_relative(socket.topic, socket.namespace)
            else:
                topic_name = socket.ros_name or socket.name
                return f"{socket.namespace}/{topic_name}"

        elif isinstance(socket, NodeSocket):
            # Node socket uses ros_name or socket name
            topic_name = socket.ros_name or socket.socket_name
            return f"{socket.node_namespace}/{topic_name}"

    # Priority 3: Multiple sources without explicit topic - ERROR
    if len(link.src) > 1:
        raise CompileError(
            f"Link '{link.name}' has {len(link.src)} sources but no explicit "
            f"'topic' attribute. Links with multiple sources must specify 'topic'."
        )

    # No sources
    if len(link.src) == 0:
        # Consume-only link - topic must be explicit
        if link.topic:
            return resolve_absolute_or_relative(link.topic, link_namespace)
        raise CompileError(f"Link '{link.name}' has no sources and no topic")

    raise CompileError(f"Unable to resolve topic for link '{link.name}'")
```

## Design Summary

| Aspect | Design Decision | Rationale |
|--------|-----------------|-----------|
| **Link-Topic Mapping** | One link = one topic | Simple, predictable |
| **Topic Naming** | Source-based | Autoware convention, clear ownership |
| **Multiple Sources** | Explicit topic required | Avoid ambiguity |
| **Encapsulation** | Opaque by default | Information hiding, modularity |
| **Transparency** | Optional, per-include | Practical for deep hierarchies |
| **Socket Forwarding** | Plan sockets expose internals | Explicit public interface |
| **Self-Contained Plans** | Can specify own topics | Testable in isolation |
| **Namespace Hierarchy** | Follows include tree | Natural organization |

## Key Benefits

1. **Type Safety**: Message type mismatches caught at compile time
2. **Clear Ownership**: Topics live in publisher namespace
3. **Explicit Connections**: All wiring visible in links
4. **Flexible Encapsulation**: Opaque for release, transparent for development
5. **Scalable**: No socket explosion in deep hierarchies
6. **Testable**: Subplans compile and run independently
7. **ROS Compatible**: Maps cleanly to ROS 2 topics and nodes

## Connection Patterns Summary

```yaml
# Pattern 1: Simple connection (inferred topic)
link:
  simple: !pubsub
    src: [node_a/out]
    dst: [node_b/in]
    # Topic: /namespace/node_a/out

# Pattern 2: Broadcast (inferred topic)
link:
  broadcast: !pubsub
    src: [sensor/data]
    dst: [consumer_a/in, consumer_b/in, consumer_c/in]
    # Topic: /namespace/sensor/data

# Pattern 3: Multiple sources (explicit topic)
link:
  aggregated: !pubsub
    topic: /shared_topic
    src: [source_a/out, source_b/out, source_c/out]
    dst: [consumer/in]
    # Topic: /shared_topic

# Pattern 4: Through plan socket
link:
  via_interface: !pubsub
    src: [subplan/exported_socket]
    dst: [consumer/in]
    # Topic: /subplan/exported_socket (from plan socket)

# Pattern 5: Transparent deep reference
link:
  direct: !pubsub
    src: [subplan/internal_node/socket]  # subplan is transparent
    dst: [consumer/in]
    # Topic: /subplan/internal_node/socket

# Pattern 6: Explicit override
link:
  custom: !pubsub
    topic: /custom/topic/name
    src: [source/out]
    dst: [consumer/in]
    # Topic: /custom/topic/name (overrides default)
```

This design provides a powerful, type-safe connection system that scales from simple examples to complex robotic systems while maintaining clarity and testability.
