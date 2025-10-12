# Runtime Configuration Reference

Complete reference for configuring the ROS-Plan runtime system.

## RuntimeConfig

The `RuntimeConfig` struct controls runtime behavior:

```rust
pub struct RuntimeConfig {
    pub restart_policy: RestartPolicy,
    pub graceful_shutdown_timeout: Duration,
    pub startup_timeout: Duration,
}
```

### Default Configuration

```rust
use ros_plan_runtime::RuntimeConfig;
use std::time::Duration;

let config = RuntimeConfig::default();
// Equivalent to:
// RuntimeConfig {
//     restart_policy: RestartPolicy::Never,
//     graceful_shutdown_timeout: Duration::from_secs(5),
//     startup_timeout: Duration::from_secs(10),
// }
```

### Creating Custom Configuration

```rust
use ros_plan_runtime::{RuntimeConfig, RestartPolicy};
use std::time::Duration;

let config = RuntimeConfig {
    restart_policy: RestartPolicy::OnFailure {
        max_retries: 3,
        backoff: Duration::from_secs(2),
    },
    graceful_shutdown_timeout: Duration::from_secs(10),
    startup_timeout: Duration::from_secs(15),
};
```

## Restart Policies

### RestartPolicy::Never

Nodes are not restarted when they crash or exit.

**Use Cases:**
- Development/debugging (want to see failures)
- Critical systems with external monitoring
- Short-lived nodes (expected to exit)

**Example:**

```rust
use ros_plan_runtime::RestartPolicy;

let policy = RestartPolicy::Never;
```

**Behavior:**
- Node crashes → stays in Crashed state
- Node exits cleanly → stays in Stopped state
- No automatic recovery

### RestartPolicy::OnFailure

Nodes are restarted only if they exit with a non-zero exit code, up to a maximum number of retries.

**Use Cases:**
- Production systems (automatic recovery)
- Known flaky nodes (network issues, etc.)
- Nodes that should normally run indefinitely

**Parameters:**
- `max_retries: u32` - Maximum number of restart attempts
- `backoff: Duration` - Delay between restart attempts

**Example:**

```rust
use ros_plan_runtime::RestartPolicy;
use std::time::Duration;

let policy = RestartPolicy::OnFailure {
    max_retries: 5,
    backoff: Duration::from_secs(3),
};
```

**Behavior:**
- Node crashes (exit code != 0) → restart after backoff
- Node exits cleanly (exit code == 0) → stays in Stopped state
- After max_retries reached → stays in Crashed state
- Restart counter persists across restarts

**Recommended Settings:**

| Scenario | max_retries | backoff |
|----------|-------------|---------|
| Development | 1-2 | 1s |
| Production | 3-5 | 2-5s |
| Critical | 10+ | 5-10s |

### RestartPolicy::Always

Nodes are always restarted regardless of exit status.

**Use Cases:**
- Development with rapid iteration
- Nodes expected to run forever
- Maximum uptime requirements

**Parameters:**
- `backoff: Duration` - Delay between restart attempts

**Example:**

```rust
use ros_plan_runtime::RestartPolicy;
use std::time::Duration;

let policy = RestartPolicy::Always {
    backoff: Duration::from_secs(1),
};
```

**Behavior:**
- Node crashes → restart after backoff
- Node exits cleanly → restart after backoff
- Restarts indefinitely
- Restart counter tracks total restarts

**Warning:** This policy can mask issues by continuously restarting failing nodes. Monitor metrics to detect restart loops.

## Timeout Configuration

### graceful_shutdown_timeout

Maximum time to wait for nodes to exit gracefully after SIGTERM.

**Type:** `Duration`

**Default:** 5 seconds

**Range:** 1-60 seconds recommended

**Behavior:**
1. Send SIGTERM to node
2. Wait up to `graceful_shutdown_timeout`
3. If still running, send SIGKILL
4. Wait for process cleanup

**Example:**

```rust
use std::time::Duration;

// Short timeout for fast shutdown
let config = RuntimeConfig {
    graceful_shutdown_timeout: Duration::from_secs(2),
    ..Default::default()
};

// Long timeout for complex cleanup
let config = RuntimeConfig {
    graceful_shutdown_timeout: Duration::from_secs(30),
    ..Default::default()
};
```

**Considerations:**
- Shorter timeout = faster shutdown, may interrupt cleanup
- Longer timeout = cleaner shutdown, slower response
- ROS 2 nodes typically need 2-5 seconds
- Database connections may need 5-10 seconds

### startup_timeout

Maximum time to wait for nodes to complete startup.

**Type:** `Duration`

**Default:** 10 seconds

**Range:** 5-120 seconds recommended

**Status:** *Currently not enforced* (planned for future implementation)

**Planned Behavior:**
1. Spawn node process
2. Wait for startup confirmation
3. If timeout expires, consider startup failed
4. Stop node and report error

**Example:**

```rust
use std::time::Duration;

// Fast startup nodes
let config = RuntimeConfig {
    startup_timeout: Duration::from_secs(5),
    ..Default::default()
};

// Slow startup nodes (loading large models, etc.)
let config = RuntimeConfig {
    startup_timeout: Duration::from_secs(60),
    ..Default::default()
};
```

## Complete Examples

### Development Configuration

Fast iteration with minimal recovery:

```rust
use ros_plan_runtime::{RuntimeConfig, RestartPolicy};
use std::time::Duration;

let config = RuntimeConfig {
    restart_policy: RestartPolicy::OnFailure {
        max_retries: 2,
        backoff: Duration::from_secs(1),
    },
    graceful_shutdown_timeout: Duration::from_secs(2),
    startup_timeout: Duration::from_secs(10),
};
```

### Production Configuration

Robust recovery with reasonable timeouts:

```rust
use ros_plan_runtime::{RuntimeConfig, RestartPolicy};
use std::time::Duration;

let config = RuntimeConfig {
    restart_policy: RestartPolicy::OnFailure {
        max_retries: 5,
        backoff: Duration::from_secs(3),
    },
    graceful_shutdown_timeout: Duration::from_secs(10),
    startup_timeout: Duration::from_secs(30),
};
```

### High-Availability Configuration

Maximum uptime with aggressive recovery:

```rust
use ros_plan_runtime::{RuntimeConfig, RestartPolicy};
use std::time::Duration;

let config = RuntimeConfig {
    restart_policy: RestartPolicy::Always {
        backoff: Duration::from_secs(2),
    },
    graceful_shutdown_timeout: Duration::from_secs(5),
    startup_timeout: Duration::from_secs(60),
};
```

### Debugging Configuration

No restarts, observe all failures:

```rust
use ros_plan_runtime::{RuntimeConfig, RestartPolicy};
use std::time::Duration;

let config = RuntimeConfig {
    restart_policy: RestartPolicy::Never,
    graceful_shutdown_timeout: Duration::from_secs(2),
    startup_timeout: Duration::from_secs(10),
};
```

## Using Configuration

### With Runtime::new()

The default configuration is used:

```rust
use ros_plan_runtime::Runtime;
use std::path::PathBuf;
use indexmap::IndexMap;

let runtime = Runtime::new(
    PathBuf::from("plan.yaml"),
    IndexMap::new()
)?;
```

### With Runtime::with_config()

Provide custom configuration:

```rust
use ros_plan_runtime::{Runtime, RuntimeConfig, RestartPolicy};
use std::path::PathBuf;
use std::time::Duration;
use indexmap::IndexMap;

let config = RuntimeConfig {
    restart_policy: RestartPolicy::OnFailure {
        max_retries: 3,
        backoff: Duration::from_secs(2),
    },
    graceful_shutdown_timeout: Duration::from_secs(10),
    startup_timeout: Duration::from_secs(15),
};

let runtime = Runtime::with_config(
    PathBuf::from("plan.yaml"),
    IndexMap::new(),
    config
)?;
```

## Runtime Modification

Configuration is currently immutable after runtime creation. To change configuration:

1. Stop the runtime
2. Create new runtime with new configuration
3. Start new runtime

Future versions may support dynamic configuration updates for some fields.

## Monitoring and Tuning

### Detecting Restart Loops

Monitor metrics to identify nodes that restart frequently:

```rust
let metrics = &runtime.state().metrics;

for (node_id, node_metrics) in metrics.all_node_metrics() {
    if node_metrics.restart_count > 10 {
        println!("Warning: {} has restarted {} times",
            node_id, node_metrics.restart_count);
    }
}
```

### Adjusting Based on Metrics

Analyze crash patterns:

```rust
let crashes = runtime.state()
    .event_log
    .events_by_type(&EventType::NodeCrash);

// If frequent crashes within short time
if crashes.len() > 5 {
    // Consider:
    // - Increasing backoff duration
    // - Reducing max_retries
    // - Fixing underlying issue
}
```

### Tuning Timeouts

Monitor shutdown times:

```rust
// Log shutdown duration
let start = Instant::now();
runtime.stop()?;
let duration = start.elapsed();

println!("Shutdown took {:?}", duration);

// If consistently hitting timeout:
// - Increase graceful_shutdown_timeout
// - Investigate why nodes don't respond to SIGTERM
```

## Best Practices

### 1. Start Conservative

Begin with `RestartPolicy::Never` or low `max_retries` to identify issues early.

### 2. Use Appropriate Backoff

- Too short: May overwhelm system resources
- Too long: Reduces availability
- Typical: 2-5 seconds for most applications

### 3. Monitor Metrics

Always monitor restart counts and crash patterns. High restart rates indicate underlying problems.

### 4. Test Recovery

Manually kill nodes to verify restart policies work as expected:

```bash
# Find node PID
ps aux | grep your_node

# Kill node
kill -9 <PID>

# Verify restart behavior
```

### 5. Match Timeouts to Workload

- Simple nodes: Short timeouts (2-5s)
- Database connections: Medium timeouts (5-15s)
- Complex cleanup: Long timeouts (15-30s)

### 6. Document Configuration

Include configuration rationale in code comments:

```rust
let config = RuntimeConfig {
    // Using OnFailure with max_retries=3 because network nodes
    // occasionally fail on connection issues but recover quickly
    restart_policy: RestartPolicy::OnFailure {
        max_retries: 3,
        backoff: Duration::from_secs(2),
    },
    // Increased timeout because database cleanup can take 10s
    graceful_shutdown_timeout: Duration::from_secs(15),
    startup_timeout: Duration::from_secs(30),
};
```

## See Also

- [User Guide](runtime_user_guide.md) - General usage
- [Architecture](runtime_architecture.md) - Internal design
- [Troubleshooting](runtime_user_guide.md#troubleshooting) - Common issues
