use crate::{
    config::RuntimeConfig,
    diff::{diff_programs, ApplyResult, ProgramDiff, UpdateResult},
    process_manager::ProcessManager,
    state::RuntimeState,
    Error, Result,
};
use indexmap::IndexMap;
use ros_plan_compiler::{Compiler, Program};
use ros_plan_format::{expr::Value, parameter::ParamName};
use std::path::PathBuf;

/// The ROS-Plan runtime system
pub struct Runtime {
    plan_path: PathBuf,
    compiler: Compiler,
    program: Option<Program>,
    process_manager: ProcessManager,
    state: RuntimeState,
    config: RuntimeConfig,
}

impl Runtime {
    /// Create a new runtime for the given plan file
    pub fn new(plan_path: PathBuf, args: IndexMap<ParamName, Value>) -> Result<Self> {
        let compiler = Compiler::new();
        let state = RuntimeState::new(args);
        let config = RuntimeConfig::default();

        Ok(Self {
            plan_path,
            compiler,
            program: None,
            process_manager: ProcessManager::new(),
            state,
            config,
        })
    }

    /// Create runtime with custom configuration
    pub fn with_config(
        plan_path: PathBuf,
        args: IndexMap<ParamName, Value>,
        config: RuntimeConfig,
    ) -> Result<Self> {
        let mut runtime = Self::new(plan_path, args)?;
        runtime.config = config;
        Ok(runtime)
    }

    /// Start the runtime - compile plan and spawn all nodes
    pub fn start(&mut self) -> Result<()> {
        // Compile the plan
        println!("Compiling plan: {}", self.plan_path.display());
        let program = self
            .compiler
            .compile(&self.plan_path, self.state.parameters.clone())?;

        println!("Plan compiled successfully");
        println!("Nodes to start: {}", program.node_tab.read().iter().count());

        // Spawn all nodes
        let mut successful = Vec::new();
        let mut failed = Vec::new();

        let node_count = program.node_tab.read().iter().count();
        for (idx, (_index, node_shared)) in program.node_tab.read().iter().enumerate() {
            node_shared.with_read(|node| {
                let node_id = node.key.clone();

                println!("Starting node {} ({}/{})...", node_id, idx + 1, node_count);

                match self.process_manager.spawn_node(node_id.clone(), &node) {
                    Ok(()) => {
                        successful.push(node_id);
                    }
                    Err(e) => {
                        eprintln!("Failed to start node {}: {}", node_id, e);
                        failed.push((node_id, e));
                    }
                }
            });
        }

        // Store the program
        self.program = Some(program);

        if !failed.is_empty() {
            // Partial failure - clean up successfully started nodes
            eprintln!("Some nodes failed to start. Cleaning up...");
            let _ = self
                .process_manager
                .stop_all(self.config.graceful_shutdown_timeout);

            return Err(Error::ProcessSpawnError(format!(
                "Failed to start {} nodes: {:?}",
                failed.len(),
                failed
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
            )));
        }

        println!("All nodes started successfully!");
        Ok(())
    }

    /// Stop the runtime - gracefully shutdown all nodes
    pub fn stop(&mut self) -> Result<()> {
        println!("Stopping runtime...");

        let results = self
            .process_manager
            .stop_all(self.config.graceful_shutdown_timeout);

        let mut failures = Vec::new();
        for (node_id, result) in results {
            match result {
                Ok(()) => {
                    println!("Stopped node: {}", node_id);
                }
                Err(e) => {
                    eprintln!("Error stopping node {}: {}", node_id, e);
                    failures.push((node_id, e));
                }
            }
        }

        if !failures.is_empty() {
            return Err(Error::ProcessStopError(format!(
                "Failed to stop {} nodes",
                failures.len()
            )));
        }

        println!("Runtime stopped successfully");
        Ok(())
    }

    /// Run the main event loop
    pub fn run(&mut self) -> Result<()> {
        use std::{
            sync::{
                atomic::{AtomicBool, Ordering},
                Arc,
            },
            time::Duration,
        };

        // Set up signal handler for graceful shutdown
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        ctrlc::set_handler(move || {
            println!("\nReceived Ctrl+C, shutting down gracefully...");
            r.store(false, Ordering::SeqCst);
        })
        .map_err(|e| Error::ProcessSpawnError(format!("Failed to set Ctrl-C handler: {}", e)))?;

        // Start all nodes
        self.start()?;

        // Main event loop
        while running.load(Ordering::SeqCst) {
            // Poll node states
            self.process_manager.poll_nodes();

            // TODO: Check for crashes and apply restart policy

            // Sleep briefly
            std::thread::sleep(Duration::from_millis(100));
        }

        // Graceful shutdown
        self.stop()?;

        Ok(())
    }

    /// Get the current runtime state
    pub fn state(&self) -> &RuntimeState {
        &self.state
    }

    /// Get the process manager
    pub fn process_manager(&self) -> &ProcessManager {
        &self.process_manager
    }

    /// Get the compiled program
    pub fn program(&self) -> Option<&Program> {
        self.program.as_ref()
    }

    /// Recompile the plan with updated parameters
    pub fn recompile_with_params(
        &mut self,
        new_params: IndexMap<ParamName, Value>,
    ) -> Result<Program> {
        // Merge new params with existing params (new params override)
        let mut merged_params = self.state.parameters.clone();
        for (key, value) in new_params {
            merged_params.insert(key, value);
        }

        // Compile with updated parameters
        let new_program = self
            .compiler
            .compile(&self.plan_path, merged_params.clone())?;

        // Update stored parameters
        self.state.parameters = merged_params;

        Ok(new_program)
    }

    /// Update a single parameter and recompile
    pub fn update_parameter(&mut self, param_name: ParamName, new_value: Value) -> Result<Program> {
        let mut params = IndexMap::new();
        params.insert(param_name, new_value);
        self.recompile_with_params(params)
    }

    /// Apply a program diff by stopping/starting/restarting nodes
    pub fn apply_diff(&mut self, diff: &ProgramDiff, new_program: &Program) -> Result<ApplyResult> {
        let mut stopped = Vec::new();
        let mut restarted = Vec::new();
        let mut started = Vec::new();
        let mut failures = Vec::new();

        // Stop removed nodes
        for node_id in &diff.removed_nodes {
            match self
                .process_manager
                .stop_node(node_id, self.config.graceful_shutdown_timeout)
            {
                Ok(()) => {
                    println!("Stopped removed node: {}", node_id);
                    stopped.push(node_id.clone());
                }
                Err(e) => {
                    eprintln!("Failed to stop node {}: {}", node_id, e);
                    failures.push((node_id.clone(), e.to_string()));
                }
            }
        }

        // Restart modified nodes
        for modification in &diff.modified_nodes {
            let node_id = &modification.ident;

            // Stop the old node
            if let Err(e) = self
                .process_manager
                .stop_node(node_id, self.config.graceful_shutdown_timeout)
            {
                eprintln!("Failed to stop modified node {}: {}", node_id, e);
                failures.push((node_id.clone(), format!("Stop failed: {}", e)));
                continue;
            }

            // Get new node context
            let new_node_ctx = find_node_by_key(&new_program.node_tab, node_id);
            match new_node_ctx {
                Some(node_ctx) => {
                    // Start the new node
                    match self.process_manager.spawn_node(node_id.clone(), &node_ctx) {
                        Ok(()) => {
                            println!("Restarted modified node: {}", node_id);
                            restarted.push(node_id.clone());
                        }
                        Err(e) => {
                            eprintln!("Failed to restart node {}: {}", node_id, e);
                            failures.push((node_id.clone(), format!("Restart failed: {}", e)));
                        }
                    }
                }
                None => {
                    eprintln!("Could not find new node context for {}", node_id);
                    failures.push((
                        node_id.clone(),
                        "Node context not found in new program".to_string(),
                    ));
                }
            }
        }

        // Start added nodes
        for node_id in &diff.added_nodes {
            let new_node_ctx = find_node_by_key(&new_program.node_tab, node_id);
            match new_node_ctx {
                Some(node_ctx) => {
                    match self.process_manager.spawn_node(node_id.clone(), &node_ctx) {
                        Ok(()) => {
                            println!("Started new node: {}", node_id);
                            started.push(node_id.clone());
                        }
                        Err(e) => {
                            eprintln!("Failed to start new node {}: {}", node_id, e);
                            failures.push((node_id.clone(), format!("Start failed: {}", e)));
                        }
                    }
                }
                None => {
                    eprintln!("Could not find node context for {}", node_id);
                    failures.push((
                        node_id.clone(),
                        "Node context not found in new program".to_string(),
                    ));
                }
            }
        }

        Ok(ApplyResult {
            stopped,
            restarted,
            started,
            failures,
        })
    }

    /// Update parameters and apply changes to running nodes
    pub fn update_parameters_and_apply(
        &mut self,
        new_params: IndexMap<ParamName, Value>,
    ) -> Result<UpdateResult> {
        let mut errors = Vec::new();

        // Check if we have a program
        if self.program.is_none() {
            return Err(Error::InvalidParameter(
                "No program compiled yet".to_string(),
            ));
        }

        // Recompile with new parameters
        let new_program = match self.recompile_with_params(new_params) {
            Ok(prog) => prog,
            Err(e) => {
                errors.push(format!("Compilation failed: {}", e));
                return Ok(UpdateResult {
                    success: false,
                    diff: ProgramDiff {
                        added_nodes: Vec::new(),
                        removed_nodes: Vec::new(),
                        modified_nodes: Vec::new(),
                        unchanged_nodes: Vec::new(),
                    },
                    applied: ApplyResult {
                        stopped: Vec::new(),
                        restarted: Vec::new(),
                        started: Vec::new(),
                        failures: Vec::new(),
                    },
                    errors,
                });
            }
        };

        // Compute diff (old program is guaranteed to exist at this point)
        let diff = diff_programs(self.program.as_ref().unwrap(), &new_program);

        // Apply diff
        let applied = match self.apply_diff(&diff, &new_program) {
            Ok(result) => result,
            Err(e) => {
                errors.push(format!("Failed to apply diff: {}", e));
                return Ok(UpdateResult {
                    success: false,
                    diff,
                    applied: ApplyResult {
                        stopped: Vec::new(),
                        restarted: Vec::new(),
                        started: Vec::new(),
                        failures: Vec::new(),
                    },
                    errors,
                });
            }
        };

        // Update stored program
        self.program = Some(new_program);

        let success = applied.success() && errors.is_empty();

        Ok(UpdateResult {
            success,
            diff,
            applied,
            errors,
        })
    }

    /// Update a single parameter and apply changes
    pub fn update_parameter_and_apply(
        &mut self,
        param_name: ParamName,
        new_value: Value,
    ) -> Result<UpdateResult> {
        let mut params = IndexMap::new();
        params.insert(param_name, new_value);
        self.update_parameters_and_apply(params)
    }

    /// Check which launch includes would be reloaded with new parameters
    pub fn check_launch_reload(
        &self,
        new_params: &IndexMap<ParamName, Value>,
    ) -> crate::launch_tracking::ReloadCheck {
        self.state.launch_tracker.check_reload_needed(new_params)
    }

    /// Get launch tracker (for manual inspection/modification)
    pub fn launch_tracker(&self) -> &crate::launch_tracking::LaunchTracker {
        &self.state.launch_tracker
    }

    /// Get mutable launch tracker (for manual registration during compilation)
    pub fn launch_tracker_mut(&mut self) -> &mut crate::launch_tracking::LaunchTracker {
        &mut self.state.launch_tracker
    }
}

/// Helper function to find node by key
fn find_node_by_key(
    node_tab: &ros_plan_compiler::utils::shared_table::SharedTable<
        ros_plan_compiler::context::node::NodeCtx,
    >,
    key: &ros_plan_format::key::KeyOwned,
) -> Option<ros_plan_compiler::context::node::NodeCtx> {
    for (_, node_shared) in node_tab.read().iter() {
        let mut result = None;
        node_shared.with_read(|node| {
            if &node.key == key {
                result = Some(node.clone());
            }
        });
        if result.is_some() {
            return result;
        }
    }
    None
}
