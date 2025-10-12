# Runtime User Guide

This guide covers using the ROS-Plan runtime system to execute and manage ROS 2 nodes from plan files.

## Overview

The ROS-Plan runtime provides a process management system for running ROS 2 nodes defined in plan files. It offers:

- **Process lifecycle management** - Start, stop, and restart nodes
- **Dynamic parameter updates** - Change parameters without full restart
- **Launch file integration** - Include existing ROS 2 launch files
- **Status monitoring** - Query runtime and node status
- **Crash recovery** - Configurable restart policies
- **Event logging** - Track all runtime events
- **Metrics collection** - Monitor node lifecycle statistics

## Getting Started

### Running a Plan

To start a runtime with a plan file:

```bash
ros2plan run examples/introduction.yaml
```

The runtime will:
1. Compile the plan file
2. Start all nodes defined in the plan
3. Monitor node processes
4. Handle signals (Ctrl+C for graceful shutdown)

### Passing Parameters

You can provide parameter values at startup:

```bash
ros2plan run examples/eval.yaml rate=20 device=/dev/video0
```

With type annotations:

```bash
ros2plan run plan.yaml rate:i64=20 enabled:bool=true
```

Supported types:
- `i64` - 64-bit signed integer
- `f64` - 64-bit floating point
- `bool` - Boolean (true/false)
- `string` - String value
- `path` - File system path

### Graceful Shutdown

Press `Ctrl+C` to trigger graceful shutdown:

1. Runtime sends SIGTERM to all nodes
2. Waits up to 5 seconds for clean exit
3. Sends SIGKILL to any remaining nodes
4. Cleans up resources and exits

## Configuration

### Restart Policies

The runtime supports three restart policies (configurable via `RuntimeConfig`):

#### Never Restart

Nodes that crash are not restarted:

```rust
RestartPolicy::Never
```

#### Restart on Failure

Nodes that crash are restarted up to a maximum number of retries:

```rust
RestartPolicy::OnFailure {
    max_retries: 3,
    backoff: Duration::from_secs(2),
}
```

#### Always Restart

Nodes are restarted regardless of exit status:

```rust
RestartPolicy::Always {
    backoff: Duration::from_secs(2),
}
```

### Timeouts

Configure shutdown and startup timeouts:

```rust
RuntimeConfig {
    restart_policy: RestartPolicy::OnFailure {
        max_retries: 3,
        backoff: Duration::from_secs(2),
    },
    graceful_shutdown_timeout: Duration::from_secs(5),
    startup_timeout: Duration::from_secs(10),
}
```

## Runtime API

### Creating a Runtime

```rust
use ros_plan_runtime::Runtime;
use std::path::PathBuf;
use indexmap::IndexMap;

let plan_path = PathBuf::from("examples/introduction.yaml");
let args = IndexMap::new();

let mut runtime = Runtime::new(plan_path, args)?;
```

### Starting the Runtime

```rust
runtime.start()?;
```

### Stopping the Runtime

```rust
runtime.stop()?;
```

### Querying Status

Get comprehensive runtime status:

```rust
let status = runtime.get_status();

println!("Uptime: {:?}", status.uptime);
println!("Nodes: {}", status.nodes.len());
println!("Running: {}", status.running_count());
println!("Crashed: {}", status.crashed_count());
```

Format status as a table:

```rust
let table = status.format_table();
println!("{}", table);
```

Format status as JSON:

```rust
let json = status.format_json()?;
println!("{}", json);
```

### Updating Parameters

Update a single parameter:

```rust
use ros_plan_format::expr::Value;

runtime.update_parameter_and_apply(
    &"rate".parse()?,
    Value::I64(20)
)?;
```

Update multiple parameters:

```rust
let mut params = IndexMap::new();
params.insert("rate".parse()?, Value::I64(20));
params.insert("enabled".parse()?, Value::Bool(true));

runtime.update_parameters_and_apply(params)?;
```

The runtime will:
1. Recompile the plan with new parameters
2. Compute diff between old and new programs
3. Restart only affected nodes
4. Return results showing which nodes were restarted

### Launch Include Tracking

Check which launch includes need reloading:

```rust
let check = runtime.check_launch_reload(&new_params);

println!("Needs reload: {:?}", check.needs_reload);
println!("Unchanged: {:?}", check.unchanged);
```

Access launch tracker:

```rust
let tracker = runtime.launch_tracker();

for (name, include) in &tracker.includes {
    println!("Include: {}", name);
    println!("  Path: {:?}", include.file_path);
    println!("  Nodes: {}", include.node_idents.len());
}
```

## Status Monitoring

### Node Status

Each node status includes:

```rust
pub struct NodeStatus {
    pub ident: KeyOwned,           // Node identifier
    pub name: String,               // Node name
    pub namespace: Option<String>,  // ROS namespace
    pub state: NodeState,           // Current state
    pub pid: Option<u32>,           // Process ID (if running)
    pub uptime: Option<Duration>,   // Time since start
    pub restart_count: u32,         // Number of restarts
    pub source: NodeSource,         // Plan or Include source
}
```

Node states:
- `Starting` - Node process spawned, starting up
- `Running` - Node is running normally
- `Stopping` - Shutdown in progress
- `Stopped` - Node stopped cleanly
- `Crashed { exit_code }` - Node exited with error

### Event Logging

Access event log from runtime state:

```rust
let events = runtime.state().event_log.events();

for event in events {
    println!("{:?}: {} - {}",
        event.timestamp,
        event.event_type,
        event.message
    );
}
```

Filter by event type:

```rust
use ros_plan_runtime::EventType;

let crashes = runtime.state().event_log.events_by_type(&EventType::NodeCrash);
println!("Total crashes: {}", crashes.len());
```

Get recent events:

```rust
let recent = runtime.state().event_log.recent(10);
```

### Metrics Collection

Access runtime metrics:

```rust
let metrics = &runtime.state().metrics;

println!("Total starts: {}", metrics.total_starts);
println!("Total stops: {}", metrics.total_stops);
println!("Total crashes: {}", metrics.total_crashes);
println!("Total restarts: {}", metrics.total_restarts);
```

Per-node metrics:

```rust
if let Some(node_metrics) = metrics.get_node_metrics(&node_id) {
    println!("Node starts: {}", node_metrics.start_count);
    println!("Node crashes: {}", node_metrics.crash_count);
    println!("Node restarts: {}", node_metrics.restart_count);
}
```

Reset metrics:

```rust
runtime.state_mut().metrics.reset();
```

## Troubleshooting

### Node Fails to Start

Check the node state and error:

```rust
let status = runtime.get_status();
for node in status.nodes {
    if let NodeState::Crashed { exit_code } = node.state {
        println!("Node {} crashed with code: {:?}", node.name, exit_code);
    }
}
```

Common causes:
- Invalid executable or package name
- Missing dependencies
- Invalid parameters
- Namespace conflicts

### Parameter Update Doesn't Take Effect

1. Check if parameter is used by any nodes
2. Verify parameter type matches plan definition
3. Check event log for compilation errors:

```rust
let errors = runtime.state()
    .event_log
    .events_by_type(&EventType::Error);
```

### Nodes Not Restarting

Check restart policy configuration:

```rust
let config = runtime.config();
match &config.restart_policy {
    RestartPolicy::Never => println!("Restart disabled"),
    RestartPolicy::OnFailure { max_retries, .. } => {
        println!("Max retries: {}", max_retries);
    }
    RestartPolicy::Always { .. } => println!("Always restart enabled"),
}
```

Check restart count:

```rust
let status = runtime.get_status();
for node in status.nodes {
    if node.restart_count >= max_retries {
        println!("Node {} exceeded max retries", node.name);
    }
}
```

### High Memory Usage

Event log and metrics use bounded storage:
- Event log: 1000 events by default (configurable)
- Metrics: Per-node counters only (minimal memory)

To reduce memory:

```rust
// Create event log with smaller size
let event_log = EventLog::new(100);

// Reset metrics periodically
runtime.state_mut().metrics.reset();
```

## Best Practices

### 1. Use Appropriate Restart Policies

- Development: `RestartPolicy::Always` for quick iteration
- Production: `RestartPolicy::OnFailure` with reasonable max_retries
- Critical systems: `RestartPolicy::Never` with external monitoring

### 2. Monitor Metrics

Regularly check metrics to identify problematic nodes:

```rust
for (node_id, metrics) in runtime.state().metrics.all_node_metrics() {
    if metrics.crash_count > 5 {
        println!("Warning: {} has crashed {} times",
            node_id, metrics.crash_count);
    }
}
```

### 3. Handle Partial Failures

When updating parameters, check the result:

```rust
let result = runtime.update_parameter_and_apply(&param, value)?;

if !result.success {
    println!("Some operations failed:");
    for error in &result.errors {
        println!("  - {}", error);
    }
}
```

### 4. Use Event Log for Debugging

Check event log when issues occur:

```rust
let recent = runtime.state().event_log.recent(20);
for event in recent {
    if matches!(event.event_type, EventType::Error | EventType::NodeCrash) {
        println!("{:?}: {}", event.timestamp, event.message);
    }
}
```

### 5. Test Parameter Updates

Before deploying parameter updates, test with a subset of nodes or in a staging environment.

## Example: Complete Runtime Application

```rust
use ros_plan_runtime::{Runtime, RuntimeConfig, RestartPolicy, EventType};
use ros_plan_format::expr::Value;
use std::path::PathBuf;
use std::time::Duration;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use indexmap::IndexMap;

fn main() -> eyre::Result<()> {
    // Create runtime with configuration
    let plan_path = PathBuf::from("plan.yaml");
    let mut args = IndexMap::new();
    args.insert("rate".parse()?, Value::I64(10));

    let config = RuntimeConfig {
        restart_policy: RestartPolicy::OnFailure {
            max_retries: 3,
            backoff: Duration::from_secs(2),
        },
        graceful_shutdown_timeout: Duration::from_secs(5),
        startup_timeout: Duration::from_secs(10),
    };

    let mut runtime = Runtime::with_config(plan_path, args, config)?;

    // Start runtime
    println!("Starting runtime...");
    runtime.start()?;

    // Set up signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // Main loop
    while running.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(100));

        // Check for crashes
        let status = runtime.get_status();
        for node in &status.nodes {
            if matches!(node.state, ros_plan_runtime::NodeState::Crashed { .. }) {
                println!("Node {} crashed!", node.name);
            }
        }
    }

    // Graceful shutdown
    println!("\nShutting down...");
    runtime.stop()?;

    // Print final metrics
    let metrics = &runtime.state().metrics;
    println!("\nFinal metrics:");
    println!("  Total starts: {}", metrics.total_starts);
    println!("  Total crashes: {}", metrics.total_crashes);
    println!("  Total restarts: {}", metrics.total_restarts);

    Ok(())
}
```

## See Also

- [Runtime Architecture](runtime_architecture.md) - Internal architecture
- [Configuration Reference](runtime_config.md) - Detailed configuration options
- [Plan File Format](../plan_file_format.md) - Plan file syntax
