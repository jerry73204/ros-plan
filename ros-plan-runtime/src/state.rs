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
}

impl RuntimeState {
    pub fn new(parameters: IndexMap<ParamName, Value>) -> Self {
        Self {
            start_time: Instant::now(),
            parameters,
        }
    }

    pub fn uptime(&self) -> std::time::Duration {
        self.start_time.elapsed()
    }
}
