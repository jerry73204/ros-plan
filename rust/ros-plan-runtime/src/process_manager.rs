use crate::{Error, Result};
use ros_plan_compiler::context::node::NodeCtx;
use ros_plan_format::key::KeyOwned;
use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    process::{Child, Command},
    time::{Duration, Instant},
};

/// Manages ROS node processes
#[derive(Debug)]
pub struct ProcessManager {
    nodes: HashMap<KeyOwned, ManagedNode>,
}

impl ProcessManager {
    pub fn new() -> Self {
        Self {
            nodes: HashMap::new(),
        }
    }

    /// Spawn a new ROS node process
    pub fn spawn_node(&mut self, node_id: KeyOwned, node_ctx: &NodeCtx) -> Result<()> {
        // Build command line using ros-utils
        let cmdline = build_node_command(node_ctx)?;

        // Spawn the process
        let mut command = Command::new("ros2");
        command.args(&cmdline);

        let child = command
            .spawn()
            .map_err(|e| Error::ProcessSpawnError(format!("Failed to spawn {}: {}", node_id, e)))?;

        let pid = child.id();

        let handle = NodeHandle {
            name: node_id.to_string(),
            namespace: node_ctx.namespace.clone(),
            process: child,
            pid,
        };

        let managed = ManagedNode {
            handle,
            state: NodeState::Starting,
            restart_count: 0,
            last_start: Instant::now(),
        };

        self.nodes.insert(node_id, managed);

        Ok(())
    }

    /// Stop a node gracefully with timeout
    pub fn stop_node(&mut self, node_id: &KeyOwned, timeout: Duration) -> Result<()> {
        let managed = self
            .nodes
            .get_mut(node_id)
            .ok_or_else(|| Error::NodeNotFound(node_id.to_string()))?;

        managed.state = NodeState::Stopping;

        // Try graceful shutdown first (SIGTERM)
        #[cfg(unix)]
        {
            let pid = managed.handle.pid as i32;
            unsafe {
                libc::kill(pid, libc::SIGTERM);
            }

            // Wait for process to exit with timeout
            let start = Instant::now();
            while start.elapsed() < timeout {
                match managed.handle.process.try_wait() {
                    Ok(Some(status)) => {
                        managed.state = if status.success() {
                            NodeState::Stopped
                        } else {
                            NodeState::Crashed {
                                exit_code: status.code(),
                            }
                        };
                        return Ok(());
                    }
                    Ok(None) => {
                        // Still running, wait a bit
                        std::thread::sleep(Duration::from_millis(100));
                    }
                    Err(e) => {
                        return Err(Error::ProcessStopError(format!(
                            "Error waiting for {}: {}",
                            node_id, e
                        )));
                    }
                }
            }

            // Timeout expired, force kill (SIGKILL)
            unsafe {
                libc::kill(pid, libc::SIGKILL);
            }

            // Wait for final exit
            match managed.handle.process.wait() {
                Ok(_status) => {
                    managed.state = NodeState::Stopped;
                    Ok(())
                }
                Err(e) => Err(Error::ProcessStopError(format!(
                    "Failed to kill {}: {}",
                    node_id, e
                ))),
            }
        }

        #[cfg(not(unix))]
        {
            // On Windows, just kill immediately
            managed.handle.process.kill()?;
            managed.handle.process.wait()?;
            managed.state = NodeState::Stopped;
            Ok(())
        }
    }

    /// Check process health and update states
    pub fn poll_nodes(&mut self) {
        for (node_id, managed) in self.nodes.iter_mut() {
            if matches!(managed.state, NodeState::Running | NodeState::Starting) {
                match managed.handle.process.try_wait() {
                    Ok(Some(status)) => {
                        // Process exited
                        if status.success() {
                            managed.state = NodeState::Stopped;
                        } else {
                            managed.state = NodeState::Crashed {
                                exit_code: status.code(),
                            };
                        }
                    }
                    Ok(None) => {
                        // Still running
                        if matches!(managed.state, NodeState::Starting) {
                            // Assume it's running now (TODO: better health check)
                            managed.state = NodeState::Running;
                        }
                    }
                    Err(e) => {
                        eprintln!("Error polling node {}: {}", node_id, e);
                    }
                }
            }
        }
    }

    /// Get reference to a managed node
    pub fn get_node(&self, node_id: &KeyOwned) -> Option<&ManagedNode> {
        self.nodes.get(node_id)
    }

    /// Get all node IDs
    pub fn node_ids(&self) -> impl Iterator<Item = &KeyOwned> {
        self.nodes.keys()
    }

    /// Stop all nodes
    pub fn stop_all(&mut self, timeout: Duration) -> Vec<(KeyOwned, Result<()>)> {
        let node_ids: Vec<_> = self.nodes.keys().cloned().collect();
        let mut results = Vec::new();

        for node_id in node_ids {
            let result = self.stop_node(&node_id, timeout);
            results.push((node_id, result));
        }

        results
    }
}

impl Default for ProcessManager {
    fn default() -> Self {
        Self::new()
    }
}

/// A managed ROS node with process handle and state
#[derive(Debug)]
pub struct ManagedNode {
    pub handle: NodeHandle,
    pub state: NodeState,
    pub restart_count: u32,
    pub last_start: Instant,
}

/// Handle to a running node process
#[derive(Debug)]
pub struct NodeHandle {
    pub name: String,
    pub namespace: Option<String>,
    pub process: Child,
    pub pid: u32,
}

/// State of a node process
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum NodeState {
    Starting,
    Running,
    Stopping,
    Stopped,
    Crashed { exit_code: Option<i32> },
}

/// Build ros2 run command for a node
fn build_node_command(node_ctx: &NodeCtx) -> Result<Vec<String>> {
    let mut args = vec!["run".to_string()];

    // Get package and executable
    let pkg = node_ctx
        .pkg
        .as_ref()
        .and_then(|p| p.get_stored().ok())
        .ok_or_else(|| Error::ProcessSpawnError("Node missing package".to_string()))?;

    let exec = node_ctx
        .exec
        .as_ref()
        .and_then(|e| e.get_stored().ok())
        .ok_or_else(|| Error::ProcessSpawnError("Node missing executable".to_string()))?;

    args.push(pkg.to_string());
    args.push(exec.to_string());

    // Add ROS args
    args.push("--ros-args".to_string());

    // Add node name
    args.push("-r".to_string());
    args.push(format!("__node:={}", node_ctx.key));

    // Add namespace if present
    if let Some(ns) = &node_ctx.namespace {
        args.push("-r".to_string());
        args.push(format!("__ns:={}", ns));
    }

    // Add parameters
    for (param_name, param_value) in &node_ctx.param {
        if let Ok(value) = param_value.get_stored() {
            args.push("-p".to_string());
            args.push(format!("{}:={}", param_name, format_param_value(value)));
        }
    }

    Ok(args)
}

/// Format a parameter value for command line
fn format_param_value(value: &ros_plan_format::expr::Value) -> String {
    use ros_plan_format::expr::Value;
    match value {
        Value::Bool(b) => b.to_string(),
        Value::I64(i) => i.to_string(),
        Value::F64(f) => f.to_string(),
        Value::String(s) => s.clone(),
        Value::BoolList(list) => format!(
            "[{}]",
            list.iter()
                .map(|b| b.to_string())
                .collect::<Vec<_>>()
                .join(",")
        ),
        Value::I64List(list) => format!(
            "[{}]",
            list.iter()
                .map(|i| i.to_string())
                .collect::<Vec<_>>()
                .join(",")
        ),
        Value::F64List(list) => format!(
            "[{}]",
            list.iter()
                .map(|f| f.to_string())
                .collect::<Vec<_>>()
                .join(",")
        ),
        Value::StringList(list) => format!(
            "[{}]",
            list.iter()
                .map(|s| format!("'{}'", s))
                .collect::<Vec<_>>()
                .join(",")
        ),
        _ => "".to_string(), // TODO: Handle other types
    }
}
