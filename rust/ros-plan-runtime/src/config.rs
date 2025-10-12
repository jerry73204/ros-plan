use serde::{Deserialize, Serialize};
use std::time::Duration;

/// Configuration for the runtime system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimeConfig {
    /// Policy for restarting crashed nodes
    pub restart_policy: RestartPolicy,
    /// Timeout for graceful shutdown (default: 5 seconds)
    pub graceful_shutdown_timeout: Duration,
    /// Timeout for node startup (default: 10 seconds)
    pub startup_timeout: Duration,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            restart_policy: RestartPolicy::OnFailure {
                max_retries: 3,
                backoff: Duration::from_secs(1),
            },
            graceful_shutdown_timeout: Duration::from_secs(5),
            startup_timeout: Duration::from_secs(10),
        }
    }
}

/// Policy for restarting nodes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RestartPolicy {
    /// Never restart nodes
    Never,
    /// Restart nodes that fail, up to max_retries times
    OnFailure { max_retries: u32, backoff: Duration },
    /// Always restart nodes, even if they exit successfully
    Always { backoff: Duration },
}
