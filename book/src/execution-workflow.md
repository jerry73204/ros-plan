# Execution Workflow

This chapter describes the complete workflow from plan file to running ROS 2 nodes, including compilation, program serialization, and runtime parameter handling.

## Overview

The ros2plan execution workflow consists of three main phases:

1. **Compilation**: Plan file → Program (in-memory representation)
2. **Serialization**: Program → YAML output (compiled program)
3. **Execution**: Compiled program + runtime parameters → ROS 2 nodes

```
┌─────────────┐
│  Plan File  │
│  (*.yaml)   │
└──────┬──────┘
       │
       │ ros2plan compile
       ▼
┌─────────────┐
│  Compiler   │
│ ┌─────────┐ │
│ │ Expand  │ │  Load includes/groups
│ │ Evaluate│ │  Execute Lua expressions
│ │ Resolve │ │  Resolve sockets & links
│ └─────────┘ │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  Program    │  Fully resolved configuration
│  (YAML)     │  Ready for execution
└──────┬──────┘
       │
       │ Launch with params
       ▼
┌─────────────┐
│ ROS 2 Nodes │  Running system
└─────────────┘
```

## Phase 1: Compilation

The compilation phase transforms a declarative plan file into a fully resolved program.

### Command-Line Interface

```bash
# Basic compilation
ros2plan compile plan.yaml

# With arguments
ros2plan compile plan.yaml camera_count:i64=3 fps:i64=60

# Save to file
ros2plan compile plan.yaml -o output.yaml

# With multiple arguments
ros2plan compile system.yaml \
  debug_mode:bool=true \
  log_level:str="DEBUG" \
  rate:f64=30.5
```

### Compilation Steps

#### 1. Program Expansion

The compiler starts by loading the root plan file and recursively expanding all includes and groups:

```rust
let mut builder = ProgramBuilder::default();
let (mut program, root_include) = builder.load_root_include(path)?;
```

**Include expansion:**
- Loads referenced plan files (by path or from ROS packages)
- Creates isolated scopes for each included plan
- Resolves relative paths and package references
- Handles nested includes recursively

**Group expansion:**
- Creates logical scopes within the same plan
- Maintains hierarchical namespacing
- Applies `when` conditions for conditional inclusion

The expansion process builds a tree of scopes, where each scope contains:
- Nodes with their configurations
- Links between nodes
- Local variables and parameters
- References to child scopes (includes/groups)

#### 2. Argument Assignment

Before evaluation, compile-time arguments are assigned to the root scope:

```rust
root_include.with_write(|mut guard| {
    guard.assign_arg = args
        .into_iter()
        .map(|(name, value)| (name, ValueStore::new(value.into())))
        .collect();
});
```

Arguments flow from parent to child scopes through the `include` directive's `arg` mapping.

#### 3. Lua Evaluation

The evaluator processes each scope, executing Lua expressions in order:

```rust
let mut evaluator = Evaluator::default();
while let Some(include) = queue.pop_front() {
    let deferred = builder.expand_include(&mut program, include.clone())?;
    queue.extend(deferred);
    evaluator.eval(&mut program, include)?;
}
```

For each scope, the evaluator:

1. **Loads arguments** into the Lua global scope
2. **Evaluates variables** in declaration order
3. **Locks globals** to prevent accidental modification
4. **Evaluates node configurations:**
   - Package and executable names
   - Plugin names (for component containers)
   - Parameter values
   - Socket configurations (types, remappings)
5. **Evaluates link configurations:**
   - Source and destination keys
   - `when` conditions
6. **Evaluates socket references:**
   - Plan socket connections
   - Key expressions
7. **Evaluates sub-scope conditions:**
   - Include `when` clauses
   - Group `when` clauses

The evaluation maintains a queue of scopes to process, ensuring proper dependency order.

#### 4. Socket Resolution

After evaluation, the socket resolver connects plan-level sockets to internal node sockets:

```rust
let mut resolver = SocketResolver::default();
resolver.resolve(&mut program)?;
```

This step:
- Maps plan sockets to actual node socket references
- Validates socket types and directions
- Creates the necessary indirection for modular composition

#### 5. Link Resolution

Finally, the link resolver establishes all connections:

```rust
let mut resolver = LinkResolver::default();
resolver.resolve(&mut program)?;
```

This step:
- Resolves all socket key references to concrete socket instances
- Validates message/service type compatibility
- Checks QoS requirement satisfaction
- Builds the final connection graph

### Compilation Output

The compiled program is a `Program` struct containing:

```rust
pub struct Program {
    pub plan_tab: SharedTable<PlanScope>,
    pub group_tab: SharedTable<GroupScope>,
    pub include_tab: SharedTable<IncludeCtx>,
    pub node_tab: SharedTable<NodeCtx>,
    pub pubsub_link_tab: SharedTable<PubSubLinkCtx>,
    pub service_link_tab: SharedTable<ServiceLinkCtx>,
    pub plan_pub_tab: SharedTable<PlanPubCtx>,
    pub plan_sub_tab: SharedTable<PlanSubCtx>,
    pub plan_srv_tab: SharedTable<PlanSrvCtx>,
    pub plan_cli_tab: SharedTable<PlanCliCtx>,
    pub node_pub_tab: SharedTable<NodePubCtx>,
    pub node_sub_tab: SharedTable<NodeSubCtx>,
    pub node_srv_tab: SharedTable<NodeSrvCtx>,
    pub node_cli_tab: SharedTable<NodeCliCtx>,
}
```

This represents a fully resolved, executable configuration.

## Phase 2: Serialization

The compiled program can be serialized to YAML for inspection or later execution:

```rust
let text = program.to_string();
std::fs::write(&output_file, text)?;
```

The serialized format preserves:
- All node configurations with evaluated parameters
- All resolved links and connections
- Socket references and remappings
- The complete scope hierarchy

### Example Output

```yaml
# Compiled program (simplified)
node_tab:
  - id: 0
    pkg: "camera_driver"
    exec: "driver_node"
    param:
      id: !i64 0
      rate: !i64 60
    pub_: {0: 0}  # Socket ID mappings

  - id: 1
    pkg: "image_proc"
    exec: "processor"
    sub: {0: 0, 1: 1}
    pub_: {0: 2}

pubsub_link_tab:
  - id: 0
    ty: "sensor_msgs/msg/Image"
    qos: {preset: "sensor-data"}
    src_key: ["/cameras/0"]
    dst_key: ["/processor/input_0"]
```

This format is the input to the execution phase.

## Phase 3: Execution

The execution phase (future work) will:

1. **Load the compiled program** from YAML
2. **Accept runtime parameters** that override compile-time values
3. **Generate ROS 2 launch configurations** for each node
4. **Start node processes** with proper remappings and parameters
5. **Monitor node lifecycle** and handle failures

### Runtime Parameter Updates

The design supports runtime parameter updates through:

**Compile-time parameters:**
- Fixed at compilation (arguments and variables)
- Resolved through Lua evaluation
- Become part of the compiled program

**Runtime parameters:**
- Can be modified after program compilation
- Passed to nodes via ROS 2 parameter mechanisms
- Support dynamic reconfiguration
- Do not require recompilation

Example workflow:
```bash
# Compile with compile-time args
ros2plan compile plan.yaml camera_count:i64=2 -o system.yaml

# Launch with runtime parameters (future)
ros2plan launch system.yaml camera_0.brightness:f64=0.8

# Update runtime parameter (future)
ros2 param set /camera_0 brightness 0.9
```

### Node Command-Line Generation

For each node in the program, the execution phase generates appropriate command-line arguments:

```bash
# Example generated command
ros2 run camera_driver driver_node \
  --ros-args \
  -r output:=/cameras/0 \
  -p id:=0 \
  -p rate:=60
```

This includes:
- Topic/service remappings (from socket resolution)
- Parameter assignments
- Namespace configuration
- QoS overrides

The `ros-utils` crate provides utilities for:
- Finding ROS 2 packages and executables
- Generating compliant command-line arguments
- Resolving package-relative paths

## Error Handling

The compilation process includes comprehensive error checking:

### Compilation Errors

```
Error: Type mismatch
  Expected: i64
  Found: f64
  Location: plan.yaml:15:12

Error: Required argument not assigned
  Argument: camera_count
  Required by: include 'camera_system'
  Location: plan.yaml:42:5

Error: Socket type mismatch
  Link: image_stream
  Source: sensor_msgs/msg/Image
  Destination: std_msgs/msg/String
  Location: plan.yaml:67:8
```

### Lua Evaluation Errors

```
Error: Lua evaluation failed
  Expression: $ camera_count / 0 $
  Error: attempt to divide by zero
  Location: plan.yaml:23:15

Error: Undefined variable
  Expression: $ unknown_var + 1 $
  Error: variable 'unknown_var' not found
  Location: plan.yaml:31:10
```

### Resolution Errors

```
Error: Socket not found
  Key: "missing_node/output"
  Referenced by: link 'connection'
  Location: plan.yaml:55:8

Error: QoS requirement not satisfied
  Required: reliable
  Provided: best-effort
  Link: critical_data
  Location: plan.yaml:78:12
```

## Performance Considerations

### Compilation Performance

The compilation process is designed for efficiency:

- **Lazy evaluation**: Lua expressions evaluated only when needed
- **Parallel potential**: Independent scopes could be processed concurrently
- **Caching**: Included plans could be cached (future optimization)
- **Incremental compilation**: Only recompile changed portions (future)

### Program Size

Compiled programs are compact:
- Expressions replaced with evaluated values
- No redundant information
- Efficient YAML serialization
- Typical size: 10-100 KB for moderate systems

## Best Practices

### Compilation Strategy

1. **Use arguments for environment-specific values**
   ```yaml
   arg:
     robot_ip: {type: "str"}
     sensor_count: {type: "i64", default: !i64 4}
   ```

2. **Precompute expensive expressions in variables**
   ```yaml
   var:
     config_path: !path $ pkg_dir("my_pkg") .. "/config/robot.yaml" $
   ```

3. **Factor out common patterns into included plans**
   ```yaml
   include:
     sensor_array:
       path: sensor_array.yaml
       arg:
         count: !i64 $ sensor_count $
   ```

4. **Use conditional compilation for variants**
   ```yaml
   node:
     simulator:
       pkg: simulator
       exec: sim_node
       when: $ use_simulator $

     hardware:
       pkg: hardware_driver
       exec: driver_node
       when: $ not use_simulator $
   ```

### Testing Compiled Programs

```bash
# Compile and inspect output
ros2plan compile plan.yaml camera_count:i64=3 -o test.yaml
less test.yaml

# Validate different configurations
ros2plan compile plan.yaml debug:bool=true -o debug.yaml
ros2plan compile plan.yaml debug:bool=false -o release.yaml

# Test with different argument combinations
for count in 1 2 4 8; do
  ros2plan compile plan.yaml sensor_count:i64=$count -o "config_${count}.yaml"
done
```

## Integration with ROS 2 Launch

The compiled program format is designed to integrate with ROS 2 launch (future work):

```python
# Future: Launch from compiled program
from ros2plan_launch import load_program

def generate_launch_description():
    program = load_program('compiled_system.yaml')

    # Override runtime parameters
    program.set_param('camera_0', 'brightness', 0.8)

    return program.to_launch_description()
```

This workflow enables:
- Pre-compiled, version-controlled system configurations
- Separation of compile-time structure from runtime parameters
- Faster launch times (no Lua evaluation at runtime)
- Easier debugging and inspection

## Summary

The ros2plan execution workflow provides a clear separation of concerns:

- **Compile-time**: Structure, connections, and parameter-driven configuration
- **Runtime**: Node execution and dynamic parameter updates

This design enables:
- **Flexibility**: Change system structure through parameters
- **Performance**: Expensive evaluation done once at compile time
- **Debuggability**: Inspectable intermediate representations
- **Modularity**: Composable plans with clear interfaces
- **Type Safety**: Comprehensive validation before execution

The compilation model supports sophisticated ROS 2 system configurations while maintaining clarity and maintainability.
