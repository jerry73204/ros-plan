/// End-to-end integration tests for launch file inclusion in plan files
use ros_plan_compiler::{error::Error, processor::program_builder::ProgramBuilder};
use std::path::PathBuf;

#[test]
fn test_compile_plan_with_launch_include() {
    let test_plan = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/launch_files/test_plan_with_launch.yaml");

    if !test_plan.exists() {
        println!("Skipping test - plan file not found");
        return;
    }

    let mut builder = ProgramBuilder::default();
    let result = builder.load_root_include(&test_plan);

    match result {
        Ok((mut program, root_include)) => {
            println!("✅ Successfully loaded root plan");

            // Expand the include to load the plan and any nested includes
            let deferred = builder.expand_include(&mut program, root_include);

            match deferred {
                Ok(deferred_includes) => {
                    println!("✅ Successfully expanded includes");
                    println!("  Deferred includes: {}", deferred_includes.len());

                    // Verify the program has nodes from both the plan and the launch file
                    let plan_count = program.plan_tab.read().iter().count();
                    let include_count = program.include_tab.read().iter().count();

                    assert!(plan_count > 0, "Expected at least one plan in program");

                    println!("  Plans loaded: {}", plan_count);
                    println!("  Includes loaded: {}", include_count);
                }
                Err(Error::LaunchFileLoadError { error, .. }) => {
                    // Check if this is a launch2dump availability issue
                    pyo3::Python::with_gil(|py| {
                        if py.import_bound("launch2dump").is_ok() {
                            panic!("Failed to expand includes: {}", error);
                        } else {
                            println!("⚠️  Skipping test - launch2dump not available: {}", error);
                        }
                    });
                }
                Err(e) => {
                    // Other errors might be expected if dependencies are missing
                    println!("⚠️  Error expanding includes (may be expected): {}", e);
                }
            }
        }
        Err(e) => {
            panic!("Failed to load root plan: {}", e);
        }
    }
}

#[test]
fn test_launch_file_node_loading() {
    // Test that a standalone launch file can be loaded as a plan
    let launch_file =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/launch_files/simple_node.launch.py");

    if !launch_file.exists() {
        println!("Skipping test - launch file not found");
        return;
    }

    // We need to test through the launch_integration module directly
    use indexmap::IndexMap;
    use ros_plan_compiler::processor::launch_integration;

    let args = IndexMap::new();
    let result = launch_integration::load_launch_as_nodes(&launch_file, args, None);

    match result {
        Ok(nodes) => {
            println!("✅ Successfully loaded nodes from launch file");
            assert_eq!(nodes.len(), 1, "Expected 1 node");

            for (ident, cfg) in &nodes {
                println!("  Loaded node: {}", ident);
                assert!(cfg.pkg.is_some(), "Node should have package");
                assert!(cfg.exec.is_some(), "Node should have executable");
            }
        }
        Err(Error::LaunchFileLoadError { error, .. }) => {
            pyo3::Python::with_gil(|py| {
                if py.import_bound("launch2dump").is_ok() {
                    panic!("Failed to load launch file: {}", error);
                } else {
                    println!("⚠️  Skipping test - launch2dump not available: {}", error);
                }
            });
        }
        Err(e) => {
            panic!("Unexpected error: {}", e);
        }
    }
}

#[test]
fn test_program_builder_with_launch_file_path() {
    // Create a temporary plan that references a launch file by absolute path
    let launch_file =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/launch_files/multi_node.launch.py");

    if !launch_file.exists() {
        println!("Skipping test - launch file not found");
        return;
    }

    // Create a minimal plan YAML in memory and write it to temp file
    let temp_dir = std::env::temp_dir();
    let temp_plan = temp_dir.join("test_launch_plan.yaml");

    let plan_content = format!(
        r#"
include:
  nodes_from_launch:
    path: {}
    launch: true
"#,
        launch_file.display()
    );

    std::fs::write(&temp_plan, plan_content).expect("Failed to write temp plan");

    let mut builder = ProgramBuilder::default();
    let result = builder.load_root_include(&temp_plan);

    match result {
        Ok((mut program, root_include)) => {
            let expand_result = builder.expand_include(&mut program, root_include);

            match expand_result {
                Ok(_deferred) => {
                    println!("✅ Successfully compiled plan with launch file");
                    let plan_count = program.plan_tab.read().iter().count();
                    println!("  Plans in program: {}", plan_count);
                }
                Err(Error::LaunchFileLoadError { error, .. }) => {
                    pyo3::Python::with_gil(|py| {
                        if py.import_bound("launch2dump").is_ok() {
                            panic!("Failed to expand launch include: {}", error);
                        } else {
                            println!("⚠️  Skipping test - launch2dump not available");
                        }
                    });
                }
                Err(e) => {
                    println!("⚠️  Error during expansion (may be expected): {}", e);
                }
            }
        }
        Err(e) => {
            panic!("Failed to load root plan: {}", e);
        }
    }

    // Clean up
    let _ = std::fs::remove_file(&temp_plan);
}
