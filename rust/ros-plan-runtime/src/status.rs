use crate::{ManagedNode, NodeState};
use indexmap::IndexMap;
use ros_plan_format::{expr::Value, key::KeyOwned, parameter::ParamName};
use serde::{Deserialize, Serialize};
use std::{path::PathBuf, time::Duration};

/// Complete runtime status information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimeStatus {
    /// Total runtime uptime
    pub uptime: Duration,
    /// Current parameter values
    pub parameters: IndexMap<ParamName, Value>,
    /// Status of all nodes
    pub nodes: Vec<NodeStatus>,
    /// Status of launch includes
    pub includes: Vec<IncludeStatus>,
}

/// Status information for a single node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeStatus {
    /// Node identifier
    pub ident: KeyOwned,
    /// Node name
    pub name: String,
    /// Namespace (if any)
    pub namespace: Option<String>,
    /// Current state
    pub state: NodeState,
    /// Process ID (if running)
    pub pid: Option<u32>,
    /// Node uptime (if running)
    pub uptime: Option<Duration>,
    /// Number of times restarted
    pub restart_count: u32,
    /// Source of the node
    pub source: NodeSource,
}

/// Source of a node
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum NodeSource {
    /// Defined directly in plan
    Plan,
    /// Loaded from launch include
    Include(String),
}

/// Status of a launch include
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IncludeStatus {
    /// Include name/identifier
    pub name: String,
    /// Path to launch file
    pub file_path: PathBuf,
    /// Number of nodes from this include
    pub node_count: usize,
    /// Parameter dependencies
    pub parameter_deps: Vec<ParamName>,
}

impl NodeStatus {
    /// Create NodeStatus from ManagedNode
    pub fn from_managed(node_id: &KeyOwned, managed: &ManagedNode, source: NodeSource) -> Self {
        let uptime = if matches!(managed.state, NodeState::Running | NodeState::Starting) {
            Some(managed.last_start.elapsed())
        } else {
            None
        };

        let pid = if matches!(managed.state, NodeState::Running | NodeState::Starting) {
            Some(managed.handle.pid)
        } else {
            None
        };

        Self {
            ident: node_id.clone(),
            name: managed.handle.name.clone(),
            namespace: managed.handle.namespace.clone(),
            state: managed.state.clone(),
            pid,
            uptime,
            restart_count: managed.restart_count,
            source,
        }
    }
}

impl RuntimeStatus {
    /// Format status as a human-readable table
    pub fn format_table(&self) -> String {
        let mut output = String::new();

        // Header
        output.push_str("=== ROS-Plan Runtime Status ===\n\n");
        output.push_str(&format!("Uptime: {:?}\n", self.uptime));
        output.push_str(&format!("Parameters: {}\n", self.parameters.len()));
        output.push_str(&format!("Nodes: {}\n", self.nodes.len()));
        output.push_str(&format!("Includes: {}\n\n", self.includes.len()));

        // Nodes table
        if !self.nodes.is_empty() {
            output.push_str("Nodes:\n");
            output.push_str(&format!(
                "{:<30} {:<15} {:<10} {:>8} {:>12}\n",
                "NAME", "STATE", "PID", "RESTARTS", "UPTIME"
            ));
            output.push_str(&"-".repeat(80));
            output.push('\n');

            for node in &self.nodes {
                let state_str = format!("{:?}", node.state);
                let pid_str = node.pid.map(|p| p.to_string()).unwrap_or("-".to_string());
                let uptime_str = node
                    .uptime
                    .map(|d| format!("{:.1}s", d.as_secs_f64()))
                    .unwrap_or("-".to_string());

                output.push_str(&format!(
                    "{:<30} {:<15} {:<10} {:>8} {:>12}\n",
                    node.name, state_str, pid_str, node.restart_count, uptime_str
                ));
            }
            output.push('\n');
        }

        // Includes
        if !self.includes.is_empty() {
            output.push_str("Launch Includes:\n");
            for include in &self.includes {
                output.push_str(&format!(
                    "  {} ({} nodes)\n    Path: {}\n",
                    include.name,
                    include.node_count,
                    include.file_path.display()
                ));
            }
        }

        output
    }

    /// Format status as JSON
    pub fn format_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Filter nodes by state
    pub fn filter_by_state(&self, state: &NodeState) -> Vec<&NodeStatus> {
        self.nodes.iter().filter(|n| &n.state == state).collect()
    }

    /// Get running node count
    pub fn running_count(&self) -> usize {
        self.nodes
            .iter()
            .filter(|n| matches!(n.state, NodeState::Running))
            .count()
    }

    /// Get crashed node count
    pub fn crashed_count(&self) -> usize {
        self.nodes
            .iter()
            .filter(|n| matches!(n.state, NodeState::Crashed { .. }))
            .count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_runtime_status_creation() {
        let status = RuntimeStatus {
            uptime: Duration::from_secs(100),
            parameters: IndexMap::new(),
            nodes: Vec::new(),
            includes: Vec::new(),
        };

        assert_eq!(status.uptime, Duration::from_secs(100));
        assert_eq!(status.nodes.len(), 0);
    }

    #[test]
    fn test_node_source() {
        let plan_source = NodeSource::Plan;
        let include_source = NodeSource::Include("camera.launch".to_string());

        assert_eq!(plan_source, NodeSource::Plan);
        assert_ne!(plan_source, include_source);
    }

    #[test]
    fn test_running_count() {
        let status = RuntimeStatus {
            uptime: Duration::from_secs(10),
            parameters: IndexMap::new(),
            nodes: vec![
                NodeStatus {
                    ident: "node1".parse().unwrap(),
                    name: "node1".to_string(),
                    namespace: None,
                    state: NodeState::Running,
                    pid: Some(123),
                    uptime: Some(Duration::from_secs(5)),
                    restart_count: 0,
                    source: NodeSource::Plan,
                },
                NodeStatus {
                    ident: "node2".parse().unwrap(),
                    name: "node2".to_string(),
                    namespace: None,
                    state: NodeState::Crashed { exit_code: Some(1) },
                    pid: None,
                    uptime: None,
                    restart_count: 1,
                    source: NodeSource::Plan,
                },
            ],
            includes: Vec::new(),
        };

        assert_eq!(status.running_count(), 1);
        assert_eq!(status.crashed_count(), 1);
    }

    #[test]
    fn test_format_table() {
        let status = RuntimeStatus {
            uptime: Duration::from_secs(60),
            parameters: IndexMap::new(),
            nodes: Vec::new(),
            includes: Vec::new(),
        };

        let table = status.format_table();
        assert!(table.contains("Runtime Status"));
        assert!(table.contains("Uptime"));
    }
}
