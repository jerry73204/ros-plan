use crate::{
    config::RuntimeConfig, process_manager::ProcessManager, state::RuntimeState, Error, Result,
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
}
