# Plan File Format

ROS-Plan uses YAML-based plan files to declaratively define ROS 2 node configurations and their interconnections. This chapter documents the complete plan file format.

## Overview

A plan file consists of several top-level sections that define arguments, variables, nodes, links, sockets, includes, and groups:

```yaml
arg:       # Input arguments with type declarations
var:       # Local variables with Lua expressions
node:      # ROS 2 node definitions
link:      # Topic and service connections between nodes
socket:    # Plan-level socket definitions
include:   # Include other plan files
group:     # Logical grouping of nodes and links
```

## Arguments (`arg`)

Arguments define typed input parameters that can be passed when compiling the plan.

```yaml
arg:
  fps:
    type: "i64"
    default: !i64 30
    help: "Camera frame rate"

  camera_name:
    type: "str"
    default: !str "camera_0"
```

### Argument Properties

- `type`: Required type annotation (see Data Types below)
- `default`: Optional default value (can be a Lua expression)
- `help`: Optional help text

Arguments can be passed via CLI:
```bash
ros2plan compile plan.yaml fps:i64=60 camera_name:str="cam1"
```

## Variables (`var`)

Variables are local values computed using Lua expressions. They are evaluated in order and can reference previously defined variables and arguments.

```yaml
var:
  half_fps: !i64 $ fps / 2 $
  image_topic: !str $ "/camera/" .. camera_name .. "/image" $
  config_list: !str_list $ {"opt1", "opt2", "opt3"} $
```

Variables are available in the Lua evaluation context for use in expressions throughout the plan.

## Nodes (`node`)

Nodes define ROS 2 executable instances with their configuration.

```yaml
node:
  camera_driver:
    pkg: camera_driver_pkg
    exec: camera_node
    when: $ fps > 0 $
    param:
      frame_rate: !i64 $ fps $
      camera_id: !str $ camera_name $
    socket:
      image_out: !pub
      config_srv: !srv

  image_processor:
    pkg: $ "processor_" .. "v2" $
    exec: processor_node
    socket:
      image_in: !sub
      result_pub: !pub
```

### Node Properties

- `pkg`: Package name (string or Lua expression)
- `exec`: Executable name (string or Lua expression)
- `plugin`: Plugin name for component containers (optional)
- `when`: Boolean Lua expression for conditional inclusion
- `param`: Map of ROS parameters (name → typed value)
- `socket`: Map of node sockets (name → socket config)

### Node Sockets

Node sockets define communication endpoints:

```yaml
socket:
  output: !pub
    type: sensor_msgs/msg/Image
    qos:
      preset: sensor-data
    from: /remapped/topic

  input: !sub
    type: sensor_msgs/msg/Image
    from: $ "/ns/" .. topic_name $

  service: !srv
    type: std_srvs/srv/Trigger

  client: !cli
    type: std_srvs/srv/Trigger
```

Socket types:
- `!pub`: Publisher
- `!sub`: Subscriber
- `!srv`: Service server
- `!cli`: Service client

Socket properties:
- `type`: Message/service type (optional, can be inferred)
- `qos`: QoS settings (for pub/sub)
- `from`: Topic/service remapping (can be Lua expression)

## Links (`link`)

Links define connections between node sockets.

### PubSub Links

```yaml
link:
  image_stream: !pubsub
    type: sensor_msgs/msg/Image
    qos:
      preset: sensor-data
    src:
      - "camera_driver/image_out"
      - $ "other_node/" .. socket_name $
    dst:
      - "image_processor/image_in"
      - "logger/image_in"
    when: $ enable_imaging $
```

### Service Links

```yaml
link:
  trigger_service: !service
    type: std_srvs/srv/Trigger
    listen: "server_node/service"
    connect:
      - "client_node/client"
    when: $ enable_service $
```

### Link Properties

**PubSub links:**
- `type`: Message type
- `qos`: QoS profile (preset or custom)
- `src`: List of publisher socket keys
- `dst`: List of subscriber socket keys
- `when`: Boolean condition for inclusion

**Service links:**
- `type`: Service type
- `listen`: Server socket key
- `connect`: List of client socket keys
- `when`: Boolean condition for inclusion

## Plan Sockets (`socket`)

Plan-level sockets expose internal node sockets to parent plans.

```yaml
socket:
  external_image: !pub
    type: sensor_msgs/msg/Image
    src:
      - "camera_driver/image_out"

  external_trigger: !srv
    type: std_srvs/srv/Trigger
    listen: "server_node/service"
```

Plan socket types and properties mirror node sockets but reference internal node sockets.

## Includes (`include`)

Include other plan files for modular composition.

```yaml
include:
  camera_subsystem: !file
    path: camera.yaml
    when: $ enable_camera $
    arg:
      fps: !i64 $ fps $
      name: !str "main_camera"

  from_package: !file
    pkg: my_package
    file: plans/processor.yaml
```

### Include Properties

- `path`: File path (absolute or relative to current plan)
- `pkg`: ROS package name (alternative to path)
- `file`: File within package (used with pkg)
- `when`: Boolean condition
- `arg`: Arguments to pass to included plan

Included plan sockets can be referenced in links:
```yaml
link:
  connection: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera_subsystem/output"]
    dst: ["local_node/input"]
```

## Groups (`group`)

Groups provide logical organization and scoping without file separation.

```yaml
group:
  visualization:
    when: $ enable_viz $
    node:
      rviz:
        pkg: rviz2
        exec: rviz2
    link:
      viz_link: !pubsub
        type: sensor_msgs/msg/Image
        src: ["/image_topic"]
        dst: ["rviz/image"]
```

Groups can contain nodes, links, and nested groups/includes.

## Data Types

ROS-Plan supports the following value types:

### Scalar Types
- `bool`: Boolean values
- `i64`: 64-bit signed integers
- `f64`: 64-bit floating point
- `str`: UTF-8 strings
- `key`: Hierarchical keys (e.g., `/namespace/node/socket`)
- `path`: File system paths
- `binary`: Base64-encoded binary data

### List Types
- `bool_list`: List of booleans
- `i64_list`: List of integers
- `f64_list`: List of floats
- `str_list`: List of strings

### Type Tags

Values are tagged with their type using YAML tags:

```yaml
var:
  flag: !bool true
  count: !i64 42
  rate: !f64 30.0
  name: !str "camera"
  topic: !key /ns/topic
  config: !path /etc/config.yaml
  data: !binary "SGVsbG8gV29ybGQ="

  flags: !bool_list [true, false, true]
  numbers: !i64_list [1, 2, 3]
  rates: !f64_list [10.0, 20.0, 30.0]
  names: !str_list ["a", "b", "c"]
```

## QoS Configuration

Quality of Service settings for pub/sub communication:

### Preset QoS

```yaml
qos:
  preset: sensor-data  # or: default, services-default, system-default
```

### Custom QoS Profile

```yaml
qos:
  profile:
    depth: 10
    reliability: reliable  # or: best-effort, system-default, unknown
```

### QoS Requirements

Node sockets can specify QoS requirements that must be satisfied by links.

## Socket Keys

Socket references use hierarchical keys:

```yaml
# Local node socket
"node_name/socket_name"

# Included plan socket
"include_name/socket_name"

# Nested include
"include/nested_include/socket_name"

# Lua expression
$ "node_" .. id .. "/output" $
```

## Conditional Compilation

The `when` clause enables conditional inclusion using Lua boolean expressions:

```yaml
node:
  debug_node:
    pkg: debug_tools
    exec: debugger
    when: $ debug_mode and verbose $

include:
  optional_feature:
    path: feature.yaml
    when: $ enable_feature $
```

When the condition evaluates to `false`, the element is excluded from the compiled program.

## Complete Example

```yaml
arg:
  camera_count:
    type: "i64"
    default: !i64 2

  fps:
    type: "i64"
    default: !i64 30

var:
  base_topic: !str "/cameras"
  half_fps: !i64 $ fps / 2 $

node:
  camera_0:
    pkg: camera_driver
    exec: driver_node
    param:
      id: !i64 0
      rate: !i64 $ fps $
    socket:
      output: !pub
        type: sensor_msgs/msg/Image

  camera_1:
    pkg: camera_driver
    exec: driver_node
    when: $ camera_count > 1 $
    param:
      id: !i64 1
      rate: !i64 $ fps $
    socket:
      output: !pub
        type: sensor_msgs/msg/Image

  processor:
    pkg: image_proc
    exec: processor
    socket:
      input_0: !sub
      input_1: !sub
      result: !pub

link:
  camera_0_link: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera_0/output"]
    dst: ["processor/input_0"]

  camera_1_link: !pubsub
    type: sensor_msgs/msg/Image
    when: $ camera_count > 1 $
    src: ["camera_1/output"]
    dst: ["processor/input_1"]

include:
  visualization:
    path: viz.yaml
    when: $ fps >= 30 $
    arg:
      topic: !key $ base_topic .. "/result" $
```

This format provides powerful declarative capabilities for defining complex ROS 2 systems with parameter-driven configuration and dynamic evaluation.
