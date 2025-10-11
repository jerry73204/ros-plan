pub mod config;
pub mod diff;
pub mod launch_tracking;
pub mod process_manager;
pub mod runtime;
pub mod state;

pub use config::{RestartPolicy, RuntimeConfig};
pub use diff::{
    diff_programs, ApplyResult, NodeChanges, NodeModification, ProgramDiff, UpdateResult,
};
pub use launch_tracking::{LaunchDiff, LaunchInclude, LaunchTracker, ReloadCheck};
pub use process_manager::{ManagedNode, NodeHandle, NodeState, ProcessManager};
pub use runtime::Runtime;
pub use state::RuntimeState;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("Compilation error: {0}")]
    CompilationError(#[from] ros_plan_compiler::Error),

    #[error("Process spawn error: {0}")]
    ProcessSpawnError(String),

    #[error("Process stop error: {0}")]
    ProcessStopError(String),

    #[error("Node not found: {0}")]
    NodeNotFound(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("Invalid parameter: {0}")]
    InvalidParameter(String),
}

pub type Result<T> = std::result::Result<T, Error>;
