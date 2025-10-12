# Runtime Architecture

This document describes the internal architecture of the ROS-Plan runtime system.

## Overview

The runtime system is responsible for executing compiled ROS-Plan programs by managing ROS 2 node processes. It provides lifecycle management, parameter updates, crash recovery, and monitoring capabilities.

## Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         Runtime                              │
│  ┌──────────────┐  ┌────────────────┐  ┌─────────────────┐ │
│  │   Compiler   │  │ Process Manager│  │  Runtime State  │ │
│  │              │  │                │  │                 │ │
│  │ - Compile    │  │ - Spawn nodes  │  │ - Parameters    │ │
│  │ - Recompile  │  │ - Monitor PIDs │  │ - Launch tracker│ │
│  │ - Lua eval   │  │ - Stop nodes   │  │ - Event log     │ │
│  └──────────────┘  │ - Restart      │  │ - Metrics       │ │
│         │          └────────────────┘  └─────────────────┘ │
│         │                   │                    │          │
│         └───────────────────┴────────────────────┘          │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
                    ┌────────────────┐
                    │   ROS 2 Nodes  │
                    │                │
                    │  node1  node2  │
                    │  node3  node4  │
                    └────────────────┘
```

## Core Components

### Runtime

**Location:** `ros-plan-runtime/src/runtime.rs`

The `Runtime` struct is the main entry point that orchestrates all components:

```rust
pub struct Runtime {
    compiler: Compiler,
    program: Option<Program>,
    process_manager: ProcessManager,
    state: RuntimeState,
    config: RuntimeConfig,
}
```

**Responsibilities:**
- Compile plan files into programs
- Start and stop all nodes
- Handle parameter updates with recompilation
- Provide status reporting API
- Coordinate crash recovery

**Key Methods:**
- `new()` - Create runtime from plan file
- `start()` - Start all nodes
- `stop()` - Stop all nodes gracefully
- `run()` - Main event loop with signal handling
- `update_parameter_and_apply()` - Update parameter and restart affected nodes
- `get_status()` - Get comprehensive runtime status
- `check_launch_reload()` - Check if launch includes need reload

### Process Manager

**Location:** `ros-plan-runtime/src/process_manager.rs`

Manages individual node processes and their lifecycle:

```rust
pub struct ProcessManager {
    nodes: HashMap<KeyOwned, ManagedNode>,
}

pub struct ManagedNode {
    handle: NodeHandle,
    state: NodeState,
    restart_count: u32,
    last_start: Instant,
}

pub struct NodeHandle {
    name: String,
    namespace: Option<String>,
    process: Child,
    pid: u32,
}

pub enum NodeState {
    Starting,
    Running,
    Stopping,
    Stopped,
    Crashed { exit_code: Option<i32> },
}
```

**Responsibilities:**
- Spawn ROS 2 node processes
- Track process IDs and states
- Monitor process health
- Implement graceful shutdown (SIGTERM → SIGKILL)
- Detect crashes via exit codes
- Clean up zombie processes

**Key Methods:**
- `spawn_node()` - Start a new node process
- `stop_node()` - Stop a node gracefully
- `poll_nodes()` - Check all nodes for state changes
- `get_node()` - Get node by identifier
- `node_ids()` - Get all node identifiers

### Runtime State

**Location:** `ros-plan-runtime/src/state.rs`

Maintains runtime state including parameters, tracking, events, and metrics:

```rust
pub struct RuntimeState {
    pub start_time: Instant,
    pub parameters: IndexMap<ParamName, Value>,
    pub launch_tracker: LaunchTracker,
    pub event_log: EventLog,
    pub metrics: RuntimeMetrics,
}
```

**Responsibilities:**
- Track current parameter values
- Maintain launch include tracking
- Record runtime events
- Collect node lifecycle metrics
- Calculate uptime

### Launch Tracker

**Location:** `ros-plan-runtime/src/launch_tracking.rs`

Tracks launch file includes and their associated nodes:

```rust
pub struct LaunchTracker {
    pub includes: HashMap<String, LaunchInclude>,
    pub node_sources: HashMap<KeyOwned, String>,
}

pub struct LaunchInclude {
    pub file_path: PathBuf,
    pub arguments: IndexMap<String, String>,
    pub node_idents: Vec<KeyOwned>,
    pub parameter_deps: Vec<ParamName>,
}
```

**Responsibilities:**
- Map nodes to their source include
- Extract parameter dependencies from launch arguments
- Detect when includes need reloading
- Compute diffs between launch tracker states

**Key Methods:**
- `register_include()` - Register a launch include
- `register_node()` - Associate node with include
- `needs_reload()` - Check if include needs reload
- `diff()` - Compute diff between two trackers

### Event Log

**Location:** `ros-plan-runtime/src/events.rs`

Circular buffer event log for runtime events:

```rust
pub struct EventLog {
    events: Vec<RuntimeEvent>,
    max_events: usize,
}

pub struct RuntimeEvent {
    pub timestamp: SystemTime,
    pub event_type: EventType,
    pub node_id: Option<KeyOwned>,
    pub message: String,
}

pub enum EventType {
    RuntimeStart, RuntimeStop,
    NodeStart, NodeStop, NodeCrash, NodeRestart,
    ParameterUpdate, LaunchReload, Error,
}
```

**Responsibilities:**
- Record all significant runtime events
- Maintain bounded circular buffer
- Provide filtering and querying
- Support debugging and monitoring

**Key Methods:**
- `log()` - Record an event
- `events()` - Get all events
- `events_by_type()` - Filter by event type
- `recent()` - Get last N events

### Runtime Metrics

**Location:** `ros-plan-runtime/src/metrics.rs`

Collects aggregate and per-node metrics:

```rust
pub struct RuntimeMetrics {
    pub total_starts: u64,
    pub total_stops: u64,
    pub total_crashes: u64,
    pub total_restarts: u64,
    pub node_metrics: HashMap<KeyOwned, NodeMetrics>,
}

pub struct NodeMetrics {
    pub start_count: u64,
    pub stop_count: u64,
    pub crash_count: u64,
    pub restart_count: u64,
}
```

**Responsibilities:**
- Track node lifecycle events
- Maintain per-node counters
- Aggregate statistics
- Support performance monitoring

### Status Reporting

**Location:** `ros-plan-runtime/src/status.rs`

Provides comprehensive runtime status information:

```rust
pub struct RuntimeStatus {
    pub uptime: Duration,
    pub parameters: IndexMap<ParamName, Value>,
    pub nodes: Vec<NodeStatus>,
    pub includes: Vec<IncludeStatus>,
}

pub struct NodeStatus {
    pub ident: KeyOwned,
    pub name: String,
    pub namespace: Option<String>,
    pub state: NodeState,
    pub pid: Option<u32>,
    pub uptime: Option<Duration>,
    pub restart_count: u32,
    pub source: NodeSource,
}
```

**Responsibilities:**
- Aggregate status from all components
- Format output (table, JSON)
- Filter and query capabilities
- Provide snapshot of runtime state

### Program Diffing

**Location:** `ros-plan-runtime/src/diff.rs`

Computes differences between compiled programs:

```rust
pub struct ProgramDiff {
    pub added_nodes: Vec<KeyOwned>,
    pub removed_nodes: Vec<KeyOwned>,
    pub modified_nodes: Vec<NodeModification>,
    pub unchanged_nodes: Vec<KeyOwned>,
}

pub struct NodeModification {
    pub ident: KeyOwned,
    pub changes: NodeChanges,
}

pub struct NodeChanges {
    pub params_changed: bool,
    pub remappings_changed: bool,
    pub namespace_changed: bool,
    pub executable_changed: bool,
}
```

**Responsibilities:**
- Compare old and new programs
- Identify node changes
- Determine affected nodes
- Enable selective restarts

## Key Workflows

### Startup Workflow

1. **Plan Loading**
   - Runtime loads plan file
   - Compiler parses YAML into Plan structure

2. **Compilation**
   - Compiler evaluates expressions with Lua
   - Resolves includes and links
   - Generates Program with NodeCtx entries

3. **Node Spawning**
   - For each NodeCtx in program:
     - Build `ros2 run` command with parameters
     - Spawn process via ProcessManager
     - Track PID and state
     - Record NodeStart event
     - Update metrics

4. **Monitoring Loop**
   - Poll node states every 100ms
   - Detect crashes via try_wait()
   - Apply restart policies
   - Update state and metrics

### Parameter Update Workflow

1. **Recompilation**
   - Merge new parameters with existing
   - Recompile plan with updated parameters
   - Generate new Program

2. **Diff Computation**
   - Compare old and new programs
   - Identify added/removed/modified nodes
   - Determine what changed (params, namespace, etc.)

3. **Selective Restart**
   - Stop removed nodes
   - Restart modified nodes with new config
   - Start added nodes
   - Leave unchanged nodes running

4. **State Update**
   - Update RuntimeState parameters
   - Replace program with new version
   - Log ParameterUpdate event
   - Return results to caller

### Graceful Shutdown Workflow

1. **Signal Reception**
   - Ctrl+C triggers signal handler
   - Signal handler sets shutdown flag

2. **Node Termination**
   - For each running node:
     - Send SIGTERM (Unix) or TerminateProcess (Windows)
     - Wait up to graceful_shutdown_timeout
     - If still running, send SIGKILL
     - Call wait() to clean up zombie

3. **Cleanup**
   - Log RuntimeStop event
   - Clear process manager state
   - Return from run() method

### Crash Detection and Recovery

1. **Detection**
   - poll_nodes() calls try_wait() on all processes
   - Non-zero exit code detected
   - State updated to Crashed

2. **Logging**
   - Record NodeCrash event
   - Update crash metrics
   - Increment node crash counter

3. **Recovery Decision**
   - Check restart policy:
     - Never: Do nothing
     - OnFailure: Check retry count < max_retries
     - Always: Always restart

4. **Restart**
   - Wait for backoff duration
   - Respawn node with same config
   - Increment restart_count
   - Record NodeRestart event
   - Update metrics

### Launch Include Reload

1. **Detection**
   - Parameter update triggers needs_reload() check
   - LaunchTracker compares parameter dependencies
   - Returns list of includes that need reload

2. **Reload**
   - Re-evaluate include arguments with new parameters
   - Call launch2dump via PyO3 (if integrated)
   - Parse new launch file metadata

3. **Diff Computation**
   - Compare old and new LaunchLoadResult
   - Identify added/removed/modified nodes
   - Map to program nodes

4. **Apply Changes**
   - Use same selective restart workflow
   - Log LaunchReload event
   - Update launch tracker cache

## Configuration

**Location:** `ros-plan-runtime/src/config.rs`

```rust
pub struct RuntimeConfig {
    pub restart_policy: RestartPolicy,
    pub graceful_shutdown_timeout: Duration,
    pub startup_timeout: Duration,
}

pub enum RestartPolicy {
    Never,
    OnFailure { max_retries: u32, backoff: Duration },
    Always { backoff: Duration },
}
```

**Default Values:**
- `graceful_shutdown_timeout`: 5 seconds
- `startup_timeout`: 10 seconds
- `restart_policy`: Never

## Error Handling

The runtime uses `eyre::Result` for error propagation. Key error types:

```rust
pub enum Error {
    CompilationError(ros_plan_compiler::Error),
    ProcessSpawnError(String),
    ProcessStopError(String),
    NodeNotFound(String),
    IoError(std::io::Error),
    InvalidParameter(String),
}
```

**Error Handling Strategy:**
- Compilation errors: Preserve old program, return error
- Spawn errors: Log, record in ApplyResult.failures
- Stop errors: Log, continue with best-effort
- Parameter errors: Validate early, fail before changes

## Platform Considerations

### Unix (Linux, macOS)

- Uses SIGTERM for graceful shutdown
- Uses SIGKILL for forceful termination
- Process group handling for child processes
- Fork/exec model

### Windows

- Uses TerminateProcess for immediate termination
- No SIGTERM equivalent
- Different process handle management

## Performance Characteristics

### Memory Usage

- **Event Log**: O(max_events) - default 1000 events
- **Metrics**: O(unique_nodes) - minimal per-node counters
- **Launch Tracker**: O(includes × nodes_per_include)
- **Process Manager**: O(active_nodes) - one Child per node

### CPU Usage

- **Idle**: Minimal - 100ms polling interval
- **Compilation**: Varies with plan complexity
- **Node Management**: O(nodes) per poll cycle
- **Diff Computation**: O(nodes) comparison

### Startup Time

- Plan parsing: ~1ms for typical plans
- Compilation: ~10-50ms depending on complexity
- Node spawning: ~100-500ms per node (ROS 2 startup)
- Total: Dominated by ROS 2 node startup

### Update Time

- Recompilation: ~10-50ms
- Diff computation: ~1ms
- Node restart: ~100-500ms per affected node
- Total: ~100ms + (500ms × affected_nodes)

## Future Enhancements

### Inter-Process Communication (IPC)

Planned for future releases:
- Unix domain sockets for control interface
- Client-server architecture for CLI commands
- Request/response protocol for status queries
- Support for remote control

### State Persistence

Planned for future releases:
- Periodic state snapshots to disk
- Crash history persistence
- Resume from saved state
- Audit logging

### Advanced Monitoring

Future enhancements:
- CPU/memory usage per node
- Topic throughput monitoring
- Node dependency graph
- Performance dashboards

### Health Checks

Future features:
- Custom health check scripts
- Detect hanging nodes (not just crashes)
- Automatic recovery strategies
- Alerting integration

## Testing Strategy

### Unit Tests

- Individual component testing
- Mock process spawning for fast tests
- State transitions and edge cases
- Configuration validation

### Integration Tests

- Full workflow testing
- Multiple node scenarios
- Parameter update workflows
- Crash and recovery scenarios

### End-to-End Tests

- Actual ROS 2 node execution
- Launch file integration
- Performance benchmarks
- Stress testing

## See Also

- [User Guide](runtime_user_guide.md) - Using the runtime system
- [Configuration Reference](runtime_config.md) - Detailed configuration
- [Compiler Architecture](../compiler/architecture.md) - Compilation process
