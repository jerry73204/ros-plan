/// ROS 2 Launch File Loader using PyO3 FFI
///
/// This module provides Rust bindings to the Python launch2dump library,
/// allowing the compiler to load ROS 2 launch files and extract node metadata.
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList, PyTuple};
use std::{collections::HashMap, path::Path};

/// Result from loading a launch file
#[derive(Debug, Clone)]
pub struct LaunchLoadResult {
    pub nodes: Vec<NodeInfo>,
    pub containers: Vec<ContainerInfo>,
    pub lifecycle_nodes: Vec<NodeInfo>,
    pub errors: Vec<String>,
    pub parameter_dependencies: Vec<String>,
}

/// Regular ROS 2 node information
#[derive(Debug, Clone)]
pub struct NodeInfo {
    pub package: String,
    pub executable: String,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub parameters: Vec<serde_json::Value>,
    pub remappings: Vec<(String, String)>,
    pub arguments: Vec<String>,
    pub env_vars: HashMap<String, String>,
}

/// Composable node information
#[derive(Debug, Clone)]
pub struct ComposableNodeInfo {
    pub plugin: String,
    pub name: String,
    pub namespace: Option<String>,
    pub parameters: Vec<serde_json::Value>,
    pub remappings: Vec<(String, String)>,
    pub extra_arguments: Vec<String>,
}

/// Composable node container information
#[derive(Debug, Clone)]
pub struct ContainerInfo {
    pub package: String,
    pub executable: String,
    pub name: String,
    pub namespace: Option<String>,
    pub composable_nodes: Vec<ComposableNodeInfo>,
}

/// Load a ROS 2 launch file using the Python launch2dump loader
///
/// # Arguments
/// * `launch_file` - Path to the .launch.py or .launch.xml file
/// * `arguments` - Launch arguments as key-value pairs
///
/// # Returns
/// `LaunchLoadResult` containing extracted nodes, containers, and errors
///
/// # Errors
/// Returns an error if:
/// - Python interpreter cannot be initialized
/// - launch2dump module cannot be imported
/// - Launch file cannot be loaded
/// - Conversion from Python to Rust fails
pub fn load_launch_file(
    launch_file: &Path,
    arguments: HashMap<String, String>,
) -> Result<LaunchLoadResult, String> {
    Python::with_gil(|py| {
        // Import launch2dump module
        let launch2dump = py
            .import_bound("launch2dump")
            .map_err(|e| format!("Failed to import launch2dump module: {}. Make sure launch2dump is installed and in PYTHONPATH.", e))?;

        // Create LaunchLoader instance
        let loader_class = launch2dump
            .getattr("LaunchLoader")
            .map_err(|e| format!("Failed to get LaunchLoader class: {}", e))?;

        let loader = loader_class
            .call0()
            .map_err(|e| format!("Failed to create LaunchLoader instance: {}", e))?;

        // Convert arguments to Python dict
        let py_args = PyDict::new_bound(py);
        for (key, value) in arguments {
            py_args
                .set_item(key, value)
                .map_err(|e| format!("Failed to set launch argument: {}", e))?;
        }

        // Call load_launch_file
        let launch_file_str = launch_file
            .to_str()
            .ok_or_else(|| format!("Invalid launch file path: {:?}", launch_file))?;

        let result = loader
            .call_method1("load_launch_file", (launch_file_str, py_args))
            .map_err(|e| format!("Failed to load launch file '{}': {}", launch_file_str, e))?;

        // Convert Python result to Rust structures
        convert_launch_result(py, &result)
    })
}

/// Convert Python LaunchResult to Rust LaunchLoadResult
fn convert_launch_result(py: Python, py_result: &Bound<PyAny>) -> Result<LaunchLoadResult, String> {
    // Extract nodes
    let py_nodes = py_result
        .getattr("nodes")
        .map_err(|e| format!("Failed to get nodes attribute: {}", e))?;
    let nodes = convert_node_list(py, &py_nodes)?;

    // Extract containers
    let py_containers = py_result
        .getattr("containers")
        .map_err(|e| format!("Failed to get containers attribute: {}", e))?;
    let containers = convert_container_list(py, &py_containers)?;

    // Extract lifecycle_nodes
    let py_lifecycle_nodes = py_result
        .getattr("lifecycle_nodes")
        .map_err(|e| format!("Failed to get lifecycle_nodes attribute: {}", e))?;
    let lifecycle_nodes = convert_node_list(py, &py_lifecycle_nodes)?;

    // Extract errors
    let py_errors = py_result
        .getattr("errors")
        .map_err(|e| format!("Failed to get errors attribute: {}", e))?;
    let errors = convert_string_list(&py_errors)?;

    // Extract parameter_dependencies
    let py_deps = py_result
        .getattr("parameter_dependencies")
        .map_err(|e| format!("Failed to get parameter_dependencies attribute: {}", e))?;
    let parameter_dependencies = convert_string_set_to_list(&py_deps)?;

    Ok(LaunchLoadResult {
        nodes,
        containers,
        lifecycle_nodes,
        errors,
        parameter_dependencies,
    })
}

/// Convert Python list of NodeInfo to Rust Vec<NodeInfo>
fn convert_node_list(py: Python, py_list: &Bound<PyAny>) -> Result<Vec<NodeInfo>, String> {
    let list = py_list
        .downcast::<PyList>()
        .map_err(|e| format!("Expected list of nodes: {}", e))?;

    let mut nodes = Vec::new();
    for item in list.iter() {
        nodes.push(convert_node_info(py, &item)?);
    }

    Ok(nodes)
}

/// Convert Python NodeInfo to Rust NodeInfo
fn convert_node_info(py: Python, py_node: &Bound<PyAny>) -> Result<NodeInfo, String> {
    let package = get_string_attr(py_node, "package")?;
    let executable = get_string_attr(py_node, "executable")?;
    let name = get_optional_string_attr(py_node, "name")?;
    let namespace = get_optional_string_attr(py_node, "namespace")?;

    let py_parameters = py_node
        .getattr("parameters")
        .map_err(|e| format!("Failed to get parameters: {}", e))?;
    let parameters = convert_parameters(py, &py_parameters)?;

    let py_remappings = py_node
        .getattr("remappings")
        .map_err(|e| format!("Failed to get remappings: {}", e))?;
    let remappings = convert_remappings(&py_remappings)?;

    let py_arguments = py_node
        .getattr("arguments")
        .map_err(|e| format!("Failed to get arguments: {}", e))?;
    let arguments = convert_string_list(&py_arguments)?;

    let py_env_vars = py_node
        .getattr("env_vars")
        .map_err(|e| format!("Failed to get env_vars: {}", e))?;
    let env_vars = convert_string_dict(&py_env_vars)?;

    Ok(NodeInfo {
        package,
        executable,
        name,
        namespace,
        parameters,
        remappings,
        arguments,
        env_vars,
    })
}

/// Convert Python list of ContainerInfo to Rust Vec<ContainerInfo>
fn convert_container_list(
    py: Python,
    py_list: &Bound<PyAny>,
) -> Result<Vec<ContainerInfo>, String> {
    let list = py_list
        .downcast::<PyList>()
        .map_err(|e| format!("Expected list of containers: {}", e))?;

    let mut containers = Vec::new();
    for item in list.iter() {
        containers.push(convert_container_info(py, &item)?);
    }

    Ok(containers)
}

/// Convert Python ContainerInfo to Rust ContainerInfo
fn convert_container_info(
    py: Python,
    py_container: &Bound<PyAny>,
) -> Result<ContainerInfo, String> {
    let package = get_string_attr(py_container, "package")?;
    let executable = get_string_attr(py_container, "executable")?;
    let name = get_string_attr(py_container, "name")?;
    let namespace = get_optional_string_attr(py_container, "namespace")?;

    let py_composable_nodes = py_container
        .getattr("composable_nodes")
        .map_err(|e| format!("Failed to get composable_nodes: {}", e))?;
    let composable_nodes = convert_composable_node_list(py, &py_composable_nodes)?;

    Ok(ContainerInfo {
        package,
        executable,
        name,
        namespace,
        composable_nodes,
    })
}

/// Convert Python list of ComposableNodeInfo to Rust Vec<ComposableNodeInfo>
fn convert_composable_node_list(
    py: Python,
    py_list: &Bound<PyAny>,
) -> Result<Vec<ComposableNodeInfo>, String> {
    let list = py_list
        .downcast::<PyList>()
        .map_err(|e| format!("Expected list of composable nodes: {}", e))?;

    let mut nodes = Vec::new();
    for item in list.iter() {
        nodes.push(convert_composable_node_info(py, &item)?);
    }

    Ok(nodes)
}

/// Convert Python ComposableNodeInfo to Rust ComposableNodeInfo
fn convert_composable_node_info(
    py: Python,
    py_node: &Bound<PyAny>,
) -> Result<ComposableNodeInfo, String> {
    let plugin = get_string_attr(py_node, "plugin")?;
    let name = get_string_attr(py_node, "name")?;
    let namespace = get_optional_string_attr(py_node, "namespace")?;

    let py_parameters = py_node
        .getattr("parameters")
        .map_err(|e| format!("Failed to get parameters: {}", e))?;
    let parameters = convert_parameters(py, &py_parameters)?;

    let py_remappings = py_node
        .getattr("remappings")
        .map_err(|e| format!("Failed to get remappings: {}", e))?;
    let remappings = convert_remappings(&py_remappings)?;

    let py_extra_arguments = py_node
        .getattr("extra_arguments")
        .map_err(|e| format!("Failed to get extra_arguments: {}", e))?;
    let extra_arguments = convert_string_list(&py_extra_arguments)?;

    Ok(ComposableNodeInfo {
        plugin,
        name,
        namespace,
        parameters,
        remappings,
        extra_arguments,
    })
}

/// Convert Python parameters list to Vec<serde_json::Value>
/// Parameters can be dicts or strings (file paths)
fn convert_parameters(
    _py: Python,
    py_params: &Bound<PyAny>,
) -> Result<Vec<serde_json::Value>, String> {
    let list = py_params
        .downcast::<PyList>()
        .map_err(|e| format!("Expected list of parameters: {}", e))?;

    let mut parameters = Vec::new();
    for item in list.iter() {
        // Convert Python object to JSON Value
        let json_str = pythonize::depythonize::<serde_json::Value>(&item)
            .map_err(|e| format!("Failed to convert parameter to JSON: {}", e))?;
        parameters.push(json_str);
    }

    Ok(parameters)
}

/// Convert Python remappings list to Vec<(String, String)>
fn convert_remappings(py_remappings: &Bound<PyAny>) -> Result<Vec<(String, String)>, String> {
    let list = py_remappings
        .downcast::<PyList>()
        .map_err(|e| format!("Expected list of remappings: {}", e))?;

    let mut remappings = Vec::new();
    for item in list.iter() {
        let tuple = item
            .downcast::<PyTuple>()
            .map_err(|e| format!("Expected tuple for remapping: {}", e))?;

        if tuple.len() != 2 {
            return Err(format!(
                "Expected 2-tuple for remapping, got {}",
                tuple.len()
            ));
        }

        let from: String = tuple
            .get_item(0)
            .map_err(|e| format!("Failed to get remapping from: {}", e))?
            .extract()
            .map_err(|e| format!("Failed to extract remapping from: {}", e))?;
        let to: String = tuple
            .get_item(1)
            .map_err(|e| format!("Failed to get remapping to: {}", e))?
            .extract()
            .map_err(|e| format!("Failed to extract remapping to: {}", e))?;

        remappings.push((from, to));
    }

    Ok(remappings)
}

/// Convert Python list of strings to Rust Vec<String>
fn convert_string_list(py_list: &Bound<PyAny>) -> Result<Vec<String>, String> {
    let list = py_list
        .downcast::<PyList>()
        .map_err(|e| format!("Expected list of strings: {}", e))?;

    let mut strings = Vec::new();
    for item in list.iter() {
        let s: String = item
            .extract()
            .map_err(|e| format!("Failed to extract string from list: {}", e))?;
        strings.push(s);
    }

    Ok(strings)
}

/// Convert Python set of strings to Rust Vec<String>
fn convert_string_set_to_list(py_set: &Bound<PyAny>) -> Result<Vec<String>, String> {
    // Try to convert as iterable
    let iter = py_set
        .iter()
        .map_err(|e| format!("Failed to iterate over set: {}", e))?;

    let mut strings = Vec::new();
    for item in iter {
        let item_bound = item.map_err(|e| format!("Failed to get item from iterator: {}", e))?;
        let s: String = item_bound
            .extract()
            .map_err(|e| format!("Failed to extract string from set: {}", e))?;
        strings.push(s);
    }

    Ok(strings)
}

/// Convert Python dict of strings to Rust HashMap<String, String>
fn convert_string_dict(py_dict: &Bound<PyAny>) -> Result<HashMap<String, String>, String> {
    let dict = py_dict
        .downcast::<PyDict>()
        .map_err(|e| format!("Expected dict: {}", e))?;

    let mut map = HashMap::new();
    for (key, value) in dict.iter() {
        let k: String = key
            .extract()
            .map_err(|e| format!("Failed to extract dict key: {}", e))?;
        let v: String = value
            .extract()
            .map_err(|e| format!("Failed to extract dict value: {}", e))?;
        map.insert(k, v);
    }

    Ok(map)
}

/// Helper: Get required string attribute from Python object
fn get_string_attr(py_obj: &Bound<PyAny>, attr_name: &str) -> Result<String, String> {
    py_obj
        .getattr(attr_name)
        .map_err(|e| format!("Failed to get attribute '{}': {}", attr_name, e))?
        .extract()
        .map_err(|e| {
            format!(
                "Failed to extract string from attribute '{}': {}",
                attr_name, e
            )
        })
}

/// Helper: Get optional string attribute from Python object
fn get_optional_string_attr(
    py_obj: &Bound<PyAny>,
    attr_name: &str,
) -> Result<Option<String>, String> {
    let attr = py_obj
        .getattr(attr_name)
        .map_err(|e| format!("Failed to get attribute '{}': {}", attr_name, e))?;

    if attr.is_none() {
        Ok(None)
    } else {
        let s: String = attr.extract().map_err(|e| {
            format!(
                "Failed to extract string from attribute '{}': {}",
                attr_name, e
            )
        })?;
        Ok(Some(s))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_python_import() {
        // This test verifies that Python interpreter can be initialized
        // and launch2dump module can be imported
        Python::with_gil(|py| {
            let result = py.import_bound("launch2dump");
            match result {
                Ok(_) => {
                    // Module imported successfully
                    println!("✅ launch2dump module imported successfully");
                }
                Err(e) => {
                    println!("⚠️  launch2dump module not available: {}", e);
                    println!("This is expected if launch2dump is not installed in the Python environment");
                }
            }
        });
    }

    #[test]
    fn test_load_simple_launch_file() {
        // Use the test launch file from the tests directory
        let test_launch =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/launch_files/simple_node.launch.py");

        if !test_launch.exists() {
            println!("Skipping test - launch file not found at {:?}", test_launch);
            return;
        }

        let args = HashMap::new();
        let result = load_launch_file(&test_launch, args);

        match result {
            Ok(launch_result) => {
                println!("✅ Successfully loaded launch file");
                println!("  Nodes: {}", launch_result.nodes.len());
                println!("  Containers: {}", launch_result.containers.len());
                println!("  Lifecycle nodes: {}", launch_result.lifecycle_nodes.len());

                // Verify we got the expected node
                assert_eq!(launch_result.nodes.len(), 1, "Expected 1 node");
                let node = &launch_result.nodes[0];
                assert_eq!(node.package, "demo_nodes_cpp");
                assert_eq!(node.executable, "talker");
                assert_eq!(node.name, Some("test_talker".to_string()));
                assert_eq!(node.namespace, Some("/test".to_string()));

                // Verify parameters were extracted
                assert!(!node.parameters.is_empty(), "Expected parameters");

                // Verify remappings were extracted
                assert_eq!(node.remappings.len(), 1, "Expected 1 remapping");
                assert_eq!(
                    node.remappings[0],
                    ("/chatter".to_string(), "/test_topic".to_string())
                );
            }
            Err(e) => {
                // Only fail if launch2dump is available
                Python::with_gil(|py| {
                    if py.import_bound("launch2dump").is_ok() {
                        panic!("Failed to load launch file: {}", e);
                    } else {
                        println!("⚠️  Skipping test - launch2dump not available: {}", e);
                    }
                });
            }
        }
    }

    #[test]
    fn test_load_multi_node_launch_file() {
        let test_launch =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/launch_files/multi_node.launch.py");

        if !test_launch.exists() {
            println!("Skipping test - launch file not found");
            return;
        }

        let args = HashMap::new();
        let result = load_launch_file(&test_launch, args);

        match result {
            Ok(launch_result) => {
                println!("✅ Successfully loaded multi-node launch file");

                // Verify we got multiple nodes
                assert_eq!(launch_result.nodes.len(), 2, "Expected 2 nodes");

                // Check first node
                let talker = &launch_result.nodes[0];
                assert_eq!(talker.package, "demo_nodes_cpp");
                assert_eq!(talker.executable, "talker");
                assert_eq!(talker.name, Some("talker_node".to_string()));

                // Check second node
                let listener = &launch_result.nodes[1];
                assert_eq!(listener.package, "demo_nodes_cpp");
                assert_eq!(listener.executable, "listener");
                assert_eq!(listener.name, Some("listener_node".to_string()));
            }
            Err(e) => {
                Python::with_gil(|py| {
                    if py.import_bound("launch2dump").is_ok() {
                        panic!("Failed to load launch file: {}", e);
                    } else {
                        println!("⚠️  Skipping test - launch2dump not available: {}", e);
                    }
                });
            }
        }
    }

    #[test]
    fn test_load_launch_file_with_arguments() {
        let test_launch =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/launch_files/with_args.launch.py");

        if !test_launch.exists() {
            println!("Skipping test - launch file not found");
            return;
        }

        // Provide launch arguments
        let mut args = HashMap::new();
        args.insert("node_name".to_string(), "custom_talker".to_string());
        args.insert("rate".to_string(), "20".to_string());

        let result = load_launch_file(&test_launch, args);

        match result {
            Ok(launch_result) => {
                println!("✅ Successfully loaded launch file with arguments");
                assert_eq!(launch_result.nodes.len(), 1, "Expected 1 node");

                let node = &launch_result.nodes[0];
                assert_eq!(node.package, "demo_nodes_cpp");
                assert_eq!(node.executable, "talker");

                // Node name should reflect the argument
                assert_eq!(node.name, Some("custom_talker".to_string()));

                // Parameters should include the rate argument
                assert!(!node.parameters.is_empty(), "Expected parameters");
            }
            Err(e) => {
                Python::with_gil(|py| {
                    if py.import_bound("launch2dump").is_ok() {
                        panic!("Failed to load launch file: {}", e);
                    } else {
                        println!("⚠️  Skipping test - launch2dump not available: {}", e);
                    }
                });
            }
        }
    }
}
