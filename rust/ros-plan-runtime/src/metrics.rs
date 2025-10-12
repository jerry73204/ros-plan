use ros_plan_format::key::KeyOwned;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Runtime metrics collection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimeMetrics {
    /// Total nodes started
    pub total_starts: u64,
    /// Total nodes stopped
    pub total_stops: u64,
    /// Total crashes
    pub total_crashes: u64,
    /// Total restarts
    pub total_restarts: u64,
    /// Per-node metrics
    pub node_metrics: HashMap<KeyOwned, NodeMetrics>,
}

/// Metrics for a single node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeMetrics {
    /// Number of times started
    pub start_count: u64,
    /// Number of times stopped
    pub stop_count: u64,
    /// Number of crashes
    pub crash_count: u64,
    /// Number of restarts
    pub restart_count: u64,
}

impl RuntimeMetrics {
    pub fn new() -> Self {
        Self {
            total_starts: 0,
            total_stops: 0,
            total_crashes: 0,
            total_restarts: 0,
            node_metrics: HashMap::new(),
        }
    }

    /// Record a node start
    pub fn record_start(&mut self, node_id: &KeyOwned) {
        self.total_starts += 1;
        self.node_metrics
            .entry(node_id.clone())
            .or_default()
            .start_count += 1;
    }

    /// Record a node stop
    pub fn record_stop(&mut self, node_id: &KeyOwned) {
        self.total_stops += 1;
        self.node_metrics
            .entry(node_id.clone())
            .or_default()
            .stop_count += 1;
    }

    /// Record a node crash
    pub fn record_crash(&mut self, node_id: &KeyOwned) {
        self.total_crashes += 1;
        self.node_metrics
            .entry(node_id.clone())
            .or_default()
            .crash_count += 1;
    }

    /// Record a node restart
    pub fn record_restart(&mut self, node_id: &KeyOwned) {
        self.total_restarts += 1;
        self.node_metrics
            .entry(node_id.clone())
            .or_default()
            .restart_count += 1;
    }

    /// Get metrics for a specific node
    pub fn get_node_metrics(&self, node_id: &KeyOwned) -> Option<&NodeMetrics> {
        self.node_metrics.get(node_id)
    }

    /// Get all node metrics
    pub fn all_node_metrics(&self) -> &HashMap<KeyOwned, NodeMetrics> {
        &self.node_metrics
    }

    /// Reset all metrics
    pub fn reset(&mut self) {
        self.total_starts = 0;
        self.total_stops = 0;
        self.total_crashes = 0;
        self.total_restarts = 0;
        self.node_metrics.clear();
    }
}

impl Default for RuntimeMetrics {
    fn default() -> Self {
        Self::new()
    }
}

impl NodeMetrics {
    pub fn new() -> Self {
        Self {
            start_count: 0,
            stop_count: 0,
            crash_count: 0,
            restart_count: 0,
        }
    }
}

impl Default for NodeMetrics {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_metrics_creation() {
        let metrics = RuntimeMetrics::new();
        assert_eq!(metrics.total_starts, 0);
        assert_eq!(metrics.total_stops, 0);
    }

    #[test]
    fn test_record_start() {
        let mut metrics = RuntimeMetrics::new();
        let node_id: KeyOwned = "test_node".parse().unwrap();

        metrics.record_start(&node_id);

        assert_eq!(metrics.total_starts, 1);
        assert_eq!(metrics.get_node_metrics(&node_id).unwrap().start_count, 1);
    }

    #[test]
    fn test_record_crash() {
        let mut metrics = RuntimeMetrics::new();
        let node_id: KeyOwned = "test_node".parse().unwrap();

        metrics.record_crash(&node_id);
        metrics.record_crash(&node_id);

        assert_eq!(metrics.total_crashes, 2);
        assert_eq!(metrics.get_node_metrics(&node_id).unwrap().crash_count, 2);
    }

    #[test]
    fn test_multiple_nodes() {
        let mut metrics = RuntimeMetrics::new();
        let node1: KeyOwned = "node1".parse().unwrap();
        let node2: KeyOwned = "node2".parse().unwrap();

        metrics.record_start(&node1);
        metrics.record_start(&node2);
        metrics.record_crash(&node1);

        assert_eq!(metrics.total_starts, 2);
        assert_eq!(metrics.total_crashes, 1);
        assert_eq!(metrics.node_metrics.len(), 2);
    }

    #[test]
    fn test_reset() {
        let mut metrics = RuntimeMetrics::new();
        let node_id: KeyOwned = "test".parse().unwrap();

        metrics.record_start(&node_id);
        metrics.record_crash(&node_id);
        metrics.reset();

        assert_eq!(metrics.total_starts, 0);
        assert_eq!(metrics.total_crashes, 0);
        assert_eq!(metrics.node_metrics.len(), 0);
    }
}
