use crate::{events::EventLog, launch_tracking::LaunchTracker, metrics::RuntimeMetrics};
use indexmap::IndexMap;
use ros_plan_format::{expr::Value, parameter::ParamName};
use std::time::Instant;

/// Runtime state tracking
#[derive(Debug, Clone)]
pub struct RuntimeState {
    /// When the runtime was started
    pub start_time: Instant,
    /// Current parameter values
    pub parameters: IndexMap<ParamName, Value>,
    /// Launch file include tracking
    pub launch_tracker: LaunchTracker,
    /// Event log
    pub event_log: EventLog,
    /// Runtime metrics
    pub metrics: RuntimeMetrics,
}

impl RuntimeState {
    pub fn new(parameters: IndexMap<ParamName, Value>) -> Self {
        Self {
            start_time: Instant::now(),
            parameters,
            launch_tracker: LaunchTracker::new(),
            event_log: EventLog::default(),
            metrics: RuntimeMetrics::default(),
        }
    }

    pub fn uptime(&self) -> std::time::Duration {
        self.start_time.elapsed()
    }
}
