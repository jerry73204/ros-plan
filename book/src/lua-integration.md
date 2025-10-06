# Lua Integration

ROS-Plan embeds the Lua scripting language (specifically Luau) to enable dynamic configuration, computed values, and conditional logic within plan files. This chapter explains how Lua is integrated and how to use it effectively.

## Overview

Lua expressions in plan files enable:

- **Dynamic values**: Compute configuration from arguments and variables
- **String manipulation**: Build topic names, paths, and identifiers
- **Arithmetic**: Calculate rates, dimensions, and other numeric values
- **Conditionals**: Control what gets included in the compiled system
- **Package resolution**: Dynamically locate ROS package resources

The Lua integration uses **mlua** with the **Luau** dialect, providing a sandboxed, safe execution environment.

## Expression Syntax

Lua expressions are embedded in YAML using special delimiters:

### Single-Line Expressions

Use `$` delimiters for single-line expressions:

```yaml
var:
  double: !i64 $ count * 2 $
  topic: !str $ "/robot/" .. robot_id .. "/cmd_vel" $
  condition: !bool $ fps >= 30 and fps <= 60 $
```

### Multi-Line Expressions

Use `$$$` delimiters for multi-line expressions:

```yaml
var:
  computed: !str $$$
    local base = "topic"
    local suffix = "_processed"
    return base .. suffix
  $$$

  complex_list: !str_list $$$
    local items = {}
    for i = 1, count do
      table.insert(items, "item_" .. i)
    end
    return items
  $$$
```

**Important**: Multi-line expressions must end with a newline before the closing `$$$`.

## Evaluation Context

### Scope and Variables

When a Lua expression is evaluated, the following are available in the global scope:

1. **Arguments** declared in the current or parent scopes
2. **Variables** declared earlier in the same scope
3. **Global functions** provided by ros2plan

Example:
```yaml
arg:
  robot_name:
    type: "str"
    default: !str "robot_0"

  speed_limit:
    type: "f64"
    default: !f64 2.0

var:
  half_speed: !f64 $ speed_limit / 2 $
  topic_name: !str $ "/" .. robot_name .. "/vel" $

  # Can reference both args and vars
  description: !str $ robot_name .. " max speed: " .. speed_limit $
```

### Evaluation Order

1. **Arguments** are loaded into the Lua environment first
2. **Variables** are evaluated in declaration order
3. Each variable can reference:
   - All arguments
   - Previously declared variables
   - Global functions
4. After variables, the **globals are locked** (read-only)
5. **Node/link expressions** are evaluated with locked globals

This ensures:
- Predictable evaluation order
- No accidental modification of shared state
- Clear dependency relationships

### Sandboxing

The Lua environment is sandboxed for security:

- No file system access (except through provided functions)
- No network access
- No ability to load external modules
- No access to operating system functions

This makes plan files safe to execute even from untrusted sources.

## Type System

### Lua to ROS Type Conversion

Lua values are automatically converted to ROS parameter types:

| Lua Type | ROS Type | Example |
|----------|----------|---------|
| `boolean` | `bool` | `true`, `false` |
| `number` | `i64` or `f64` | `42`, `3.14` |
| `string` | `str`, `key`, or `path` | `"text"`, `"/ns/topic"` |
| `table (array)` | `*_list` types | `{1, 2, 3}`, `{"a", "b"}` |

### Explicit Type Annotations

Values are tagged with their expected type:

```yaml
var:
  # Number conversions
  int_value: !i64 $ 42 $          # Lua number → i64
  float_value: !f64 $ 3.14 $      # Lua number → f64
  computed: !i64 $ 10 / 3 $       # Division result → i64 (truncated)

  # String conversions
  text: !str $ "hello" $          # Lua string → str
  topic: !key $ "/ns/topic" $     # Lua string → key (validated)
  file: !path $ "/etc/config" $   # Lua string → path

  # List conversions
  flags: !bool_list $ {true, false, true} $
  numbers: !i64_list $ {1, 2, 3} $
  names: !str_list $ {"a", "b", "c"} $
```

### Type Checking

Type mismatches are caught at compile time:

```yaml
# ERROR: Type mismatch
var:
  bad: !i64 $ "not a number" $
  # Error: Expected i64, found string
```

## Common Patterns

### String Concatenation

```yaml
var:
  # Simple concatenation
  topic: !str $ "/robot/" .. robot_id $

  # Multiple parts
  path: !str $ pkg_dir("my_pkg") .. "/config/" .. robot_name .. ".yaml" $

  # With formatting
  frame_id: !str $ "robot_" .. string.format("%02d", id) $
```

### Arithmetic Operations

```yaml
var:
  # Basic arithmetic
  double: !i64 $ count * 2 $
  half: !f64 $ rate / 2.0 $
  offset: !i64 $ base_id + 10 $

  # More complex
  area: !f64 $ width * height $
  diagonal: !f64 $ math.sqrt(width^2 + height^2) $
  fps: !i64 $ math.floor(1000 / frame_time) $
```

### Conditional Values

```yaml
var:
  # Ternary-style
  topic_name: !str $ debug_mode and "/debug/topic" or "/normal/topic" $

  # Numeric selection
  rate: !i64 $ high_performance and 60 or 30 $

  # Complex conditions
  config_file: !str $$$
    if environment == "production" then
      return "prod_config.yaml"
    elseif environment == "staging" then
      return "staging_config.yaml"
    else
      return "dev_config.yaml"
    end
  $$$
```

### List Generation

```yaml
var:
  # Simple range
  indices: !i64_list $$$
    local result = {}
    for i = 0, count - 1 do
      table.insert(result, i)
    end
    return result
  $$$

  # Generated names
  camera_topics: !str_list $$$
    local topics = {}
    for i = 1, camera_count do
      table.insert(topics, "/camera_" .. i .. "/image")
    end
    return topics
  $$$

  # Filtered list
  valid_ids: !i64_list $$$
    local ids = {1, 2, 3, 4, 5}
    local result = {}
    for _, id in ipairs(ids) do
      if id % 2 == 0 then
        table.insert(result, id)
      end
    end
    return result
  $$$
```

### Node Configuration

```yaml
node:
  camera:
    # Dynamic package name
    pkg: $ "camera_driver_" .. driver_version $

    # Dynamic executable
    exec: $ use_debug and "node_debug" or "node" $

    param:
      # Computed parameters
      rate: !i64 $ base_rate * multiplier $
      topic: !str $ "/" .. namespace .. "/image" $

    socket:
      output: !pub
        # Dynamic remapping
        from: $ "/cameras/" .. camera_id $
```

### Conditional Compilation

```yaml
# Conditional node
node:
  debug_logger:
    pkg: debug_tools
    exec: logger
    when: $ debug_mode and log_level == "VERBOSE" $

# Conditional link
link:
  visualization: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera/output"]
    dst: ["rviz/input"]
    when: $ enable_viz and fps >= 10 $

# Conditional include
include:
  optional_feature:
    path: feature.yaml
    when: $ enable_feature or debug_mode $
```

## Global Functions

### `pkg_dir(package_name)`

Returns the absolute path to a ROS package:

```yaml
var:
  config_path: !path $ pkg_dir("my_package") .. "/config/robot.yaml" $
  data_dir: !path $ pkg_dir("sensor_data") .. "/calibration" $
```

This function:
- Queries the ROS 2 package system
- Returns an absolute path string
- Fails if the package is not found
- Useful for package-relative resource references

Usage example:
```yaml
node:
  processor:
    pkg: image_processing
    exec: processor_node
    param:
      config_file: !path $ pkg_dir("image_processing") .. "/config/default.yaml" $
      model_path: !path $ pkg_dir("ml_models") .. "/models/detector.onnx" $
```

## Lua Standard Library

A subset of the Lua standard library is available:

### String Operations

```yaml
var:
  upper: !str $ string.upper("hello") $              # "HELLO"
  lower: !str $ string.lower("WORLD") $              # "world"
  formatted: !str $ string.format("%03d", 42) $      # "042"
  substring: !str $ string.sub("hello", 1, 3) $      # "hel"
  length: !i64 $ string.len("hello") $               # 5
```

### Math Operations

```yaml
var:
  max_val: !i64 $ math.max(10, 20, 15) $             # 20
  min_val: !i64 $ math.min(10, 20, 15) $             # 10
  sqrt: !f64 $ math.sqrt(16) $                       # 4.0
  rounded: !i64 $ math.floor(3.7) $                  # 3
  ceiling: !i64 $ math.ceil(3.2) $                   # 4
  absolute: !f64 $ math.abs(-5.5) $                  # 5.5
  pi: !f64 $ math.pi $                               # 3.14159...
```

### Table Operations

```yaml
var:
  list_len: !i64 $ #{"a", "b", "c"} $                # 3

  concatenated: !str_list $$$
    local a = {"x", "y"}
    local b = {"z"}
    local result = {}
    for _, v in ipairs(a) do table.insert(result, v) end
    for _, v in ipairs(b) do table.insert(result, v) end
    return result
  $$$  # {"x", "y", "z"}
```

## Advanced Examples

### Dynamic Node Array

```yaml
arg:
  sensor_count:
    type: "i64"
    default: !i64 4

  base_topic:
    type: "str"
    default: !str "/sensors"

var:
  sensor_ids: !i64_list $$$
    local ids = {}
    for i = 0, sensor_count - 1 do
      table.insert(ids, i)
    end
    return ids
  $$$

group:
  sensors:
    node:
      # This would ideally be in a loop, but current design
      # uses explicit declarations
      sensor_0:
        pkg: sensor_driver
        exec: sensor_node
        when: $ sensor_count > 0 $
        param:
          id: !i64 0
          topic: !str $ base_topic .. "/0" $

      sensor_1:
        pkg: sensor_driver
        exec: sensor_node
        when: $ sensor_count > 1 $
        param:
          id: !i64 1
          topic: !str $ base_topic .. "/1" $

      # ... (more sensors as needed)
```

### Environment-Based Configuration

```yaml
arg:
  environment:
    type: "str"
    default: !str "development"

var:
  is_production: !bool $ environment == "production" $
  is_staging: !bool $ environment == "staging" $
  is_development: !bool $ environment == "development" $

  log_level: !str $$$
    if is_production then
      return "WARN"
    elseif is_staging then
      return "INFO"
    else
      return "DEBUG"
    end
  $$$

  enable_profiling: !bool $ not is_production $

  data_dir: !path $$$
    local base = pkg_dir("my_app")
    if is_production then
      return base .. "/data/prod"
    else
      return base .. "/data/dev"
    end
  $$$

node:
  main:
    pkg: my_app
    exec: main_node
    param:
      log_level: !str $ log_level $
      profiling: !bool $ enable_profiling $
      data_directory: !path $ data_dir $
```

### Computed QoS Settings

```yaml
arg:
  reliability_mode:
    type: "str"
    default: !str "reliable"

var:
  qos_reliability: !str $ reliability_mode $
  qos_depth: !i64 $$$
    if reliability_mode == "reliable" then
      return 10
    else
      return 100  # Larger buffer for best-effort
    end
  $$$

link:
  data_stream: !pubsub
    type: sensor_msgs/msg/Image
    src: ["camera/output"]
    dst: ["processor/input"]
    qos:
      profile:
        depth: $ qos_depth $
        reliability: $ qos_reliability $
```

## Error Handling

### Common Errors

**Undefined variable:**
```yaml
var:
  bad: !str $ unknown_var $
# Error: variable 'unknown_var' not found
```

**Type error:**
```yaml
var:
  bad: !i64 $ "not a number" $
# Error: Expected i64, found string
```

**Syntax error:**
```yaml
var:
  bad: !str $ "unterminated string $
# Error: Lua syntax error: unexpected end of line
```

**Division by zero:**
```yaml
var:
  bad: !f64 $ 1.0 / 0.0 $
# Error: attempt to divide by zero
```

### Debugging Tips

1. **Test expressions in isolation:**
   ```yaml
   var:
     test: !str $ "value: " .. debug_value $
   ```

2. **Use multi-line for complex logic:**
   ```yaml
   var:
     result: !str $$$
       -- Add debug prints if needed during development
       local value = compute_something()
       return value
     $$$
   ```

3. **Validate types explicitly:**
   ```yaml
   var:
     count: !i64 $ math.floor(computed_value) $
   ```

4. **Check compiled output:**
   ```bash
   ros2plan compile plan.yaml -o debug.yaml
   # Inspect debug.yaml to see evaluated values
   ```

## Best Practices

### 1. Prefer Variables Over Inline Expressions

**Good:**
```yaml
var:
  camera_topic: !str $ "/robot/" .. robot_id .. "/camera" $

node:
  camera:
    pkg: camera_driver
    exec: camera_node
    socket:
      output: !pub
        from: $ camera_topic $
```

**Avoid:**
```yaml
node:
  camera:
    pkg: camera_driver
    exec: camera_node
    socket:
      output: !pub
        from: $ "/robot/" .. robot_id .. "/camera" $
```

### 2. Keep Expressions Simple

**Good:**
```yaml
var:
  rate_hz: !f64 $ 1000.0 / period_ms $
  is_enabled: !bool $ sensor_count > 0 $
```

**Avoid:**
```yaml
var:
  complex: !f64 $ ((a + b) * c - d) / (e + f) ^ 2 + math.sqrt(g * h) $
  # Consider breaking into multiple variables
```

### 3. Use Meaningful Variable Names

**Good:**
```yaml
var:
  camera_frame_rate: !i64 $ base_rate * 2 $
  image_topic_name: !str $ "/cameras/" .. camera_id $
```

**Avoid:**
```yaml
var:
  x: !i64 $ y * 2 $
  t: !str $ "/c/" .. z $
```

### 4. Document Complex Logic

```yaml
var:
  # Calculate required buffer size based on frame rate and latency
  # Formula: frames_per_second * max_latency_seconds
  buffer_size: !i64 $ fps * max_latency $

  # Determine QoS based on network reliability
  # Use best-effort for WiFi, reliable for wired
  qos_mode: !str $ network_type == "wifi" and "best-effort" or "reliable" $
```

### 5. Validate Input Ranges

```yaml
arg:
  camera_count:
    type: "i64"
    default: !i64 1

var:
  # Ensure camera_count is in valid range
  validated_count: !i64 $$$
    local count = camera_count
    if count < 1 then count = 1 end
    if count > 10 then count = 10 end
    return count
  $$$
```

## Performance Considerations

- **Compilation overhead**: Lua evaluation happens at compile time, not runtime
- **No runtime cost**: Evaluated expressions become static values in the compiled program
- **Caching**: Results are computed once during compilation
- **Optimization**: Keep expressions simple for faster compilation

## Limitations

1. **No external modules**: Cannot `require()` Lua libraries
2. **No side effects**: Cannot modify files, network, or system state
3. **Deterministic**: Same inputs always produce same outputs
4. **No loops in YAML**: Must generate repeated elements explicitly
5. **Static after compilation**: Expressions evaluated once, results fixed

## Summary

Lua integration in ROS-Plan provides:

- **Flexibility**: Dynamic configuration based on parameters
- **Safety**: Sandboxed execution environment
- **Performance**: Zero runtime overhead (compile-time evaluation)
- **Expressiveness**: Full scripting language for complex logic
- **Type safety**: Automatic type checking and conversion

The combination of declarative YAML structure with imperative Lua expressions creates a powerful configuration system that balances simplicity with flexibility.
