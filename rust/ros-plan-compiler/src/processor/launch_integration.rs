/// Launch File Integration
///
/// This module handles integration of ROS 2 launch files (.launch.py/.launch.xml)
/// into the compiler pipeline by loading them via PyO3 and converting to Plan nodes.
use crate::{
    error::Error,
    launch_loader::{self, ContainerInfo, NodeInfo},
};
use indexmap::IndexMap;
use ros_plan_format::{
    expr::{TextOrExpr, Value, ValueOrExpr},
    node::{NodeCfg, NodeIdent},
    parameter::ParamName,
};
use std::{collections::HashMap, path::Path, str::FromStr};

/// Load a ROS 2 launch file and convert it to Plan node configurations
pub fn load_launch_as_nodes(
    launch_path: &Path,
    arguments: IndexMap<ParamName, Value>,
    namespace_prefix: Option<&str>,
) -> Result<IndexMap<NodeIdent, NodeCfg>, Error> {
    // Convert IndexMap<ParamName, Value> to HashMap<String, String>
    let args: HashMap<String, String> = arguments
        .into_iter()
        .map(|(k, v)| (k.to_string(), value_to_string(v)))
        .collect();

    // Load the launch file using PyO3
    let load_result = launch_loader::load_launch_file(launch_path, args).map_err(|e| {
        Error::LaunchFileLoadError {
            path: launch_path.to_path_buf(),
            error: e,
        }
    })?;

    // Convert loaded nodes to NodeCfg
    let mut nodes = IndexMap::new();

    // Process regular nodes
    for node_info in load_result.nodes {
        let (ident, cfg) = convert_node_info_to_cfg(node_info, namespace_prefix)?;
        nodes.insert(ident, cfg);
    }

    // Process lifecycle nodes
    for node_info in load_result.lifecycle_nodes {
        let (ident, cfg) = convert_node_info_to_cfg(node_info, namespace_prefix)?;
        nodes.insert(ident, cfg);
    }

    // Process containers (each container becomes a node)
    for container_info in load_result.containers {
        let (ident, cfg) = convert_container_to_cfg(container_info, namespace_prefix)?;
        nodes.insert(ident, cfg);
    }

    Ok(nodes)
}

/// Convert a launch NodeInfo to a Plan NodeCfg
fn convert_node_info_to_cfg(
    node_info: NodeInfo,
    _namespace_prefix: Option<&str>,
) -> Result<(NodeIdent, NodeCfg), Error> {
    // Determine node identifier (use node name or generate from package/executable)
    let node_name = node_info
        .name
        .clone()
        .unwrap_or_else(|| format!("{}__{}", node_info.package, node_info.executable));

    let ident = NodeIdent::try_from(node_name).map_err(|e| Error::KeyResolutionError {
        key: ros_plan_format::key::KeyOwned::new_root(),
        reason: format!("Invalid node identifier: {}", e),
    })?;

    // For now, we don't support namespace, remappings, or ros_args in the plan format
    // These would need to be added to NodeCfg structure first
    // TODO: Extend NodeCfg to support these ROS 2 features

    let cfg = NodeCfg {
        pkg: Some(TextOrExpr::from(node_info.package)),
        exec: Some(TextOrExpr::from(node_info.executable)),
        plugin: None,
        when: None,
        param: convert_parameters(node_info.parameters)?,
        socket: IndexMap::new(),
    };

    Ok((ident, cfg))
}

/// Convert a container to a node configuration
fn convert_container_to_cfg(
    container_info: ContainerInfo,
    _namespace_prefix: Option<&str>,
) -> Result<(NodeIdent, NodeCfg), Error> {
    let node_name = container_info.name.clone();

    let ident = NodeIdent::try_from(node_name).map_err(|e| Error::KeyResolutionError {
        key: ros_plan_format::key::KeyOwned::new_root(),
        reason: format!("Invalid container identifier: {}", e),
    })?;

    // For containers, we only create the container node itself
    // Composable nodes are not yet supported in the plan format
    // TODO: Add support for composable nodes when the format supports it
    let cfg = NodeCfg {
        pkg: Some(TextOrExpr::from(container_info.package)),
        exec: Some(TextOrExpr::from(container_info.executable)),
        plugin: None,
        when: None,
        param: IndexMap::new(),
        socket: IndexMap::new(),
    };

    Ok((ident, cfg))
}

/// Convert serde_json parameters to Plan parameters
fn convert_parameters(
    params: Vec<serde_json::Value>,
) -> Result<IndexMap<ParamName, ValueOrExpr>, Error> {
    let mut param_map = IndexMap::new();

    for param in params {
        match param {
            serde_json::Value::Object(obj) => {
                for (key, value) in obj {
                    let param_name =
                        ParamName::from_str(&key).map_err(|e| Error::KeyResolutionError {
                            key: ros_plan_format::key::KeyOwned::new_root(),
                            reason: format!("Invalid parameter name '{}': {}", key, e),
                        })?;
                    let param_value = json_to_value(value)?;
                    param_map.insert(param_name, ValueOrExpr::Value(param_value));
                }
            }
            _ => {
                // Skip non-object parameters (e.g., parameter files marked as __param_file)
                continue;
            }
        }
    }

    Ok(param_map)
}

/// Convert JSON value to Plan Value
fn json_to_value(json: serde_json::Value) -> Result<Value, Error> {
    match json {
        serde_json::Value::Bool(b) => Ok(Value::Bool(b)),
        serde_json::Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Ok(Value::I64(i))
            } else if let Some(f) = n.as_f64() {
                Ok(Value::F64(f))
            } else {
                Err(Error::KeyResolutionError {
                    key: ros_plan_format::key::KeyOwned::new_root(),
                    reason: format!("Unsupported number type: {}", n),
                })
            }
        }
        serde_json::Value::String(s) => Ok(Value::String(s)),
        serde_json::Value::Array(arr) => {
            let values: Result<Vec<_>, _> = arr.into_iter().map(json_to_value).collect();
            Ok(Value::StringList(
                values?
                    .into_iter()
                    .map(|v| match v {
                        Value::String(s) => Ok(s),
                        _ => Err(Error::KeyResolutionError {
                            key: ros_plan_format::key::KeyOwned::new_root(),
                            reason: "Array elements must be strings for StringList".to_string(),
                        }),
                    })
                    .collect::<Result<Vec<_>, _>>()?,
            ))
        }
        serde_json::Value::Null => Err(Error::KeyResolutionError {
            key: ros_plan_format::key::KeyOwned::new_root(),
            reason: "Null values not supported".to_string(),
        }),
        serde_json::Value::Object(_) => Err(Error::KeyResolutionError {
            key: ros_plan_format::key::KeyOwned::new_root(),
            reason: "Nested objects not yet supported".to_string(),
        }),
    }
}

/// Convert Plan Value to String for launch arguments
fn value_to_string(value: Value) -> String {
    match value {
        Value::Bool(b) => b.to_string(),
        Value::I64(i) => i.to_string(),
        Value::F64(f) => f.to_string(),
        Value::String(s) => s,
        Value::StringList(list) => list.join(","),
        Value::Binary(_) => "[binary]".to_string(),
        _ => format!("{:?}", value), // Fallback for other types
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_value_to_string_conversions() {
        assert_eq!(value_to_string(Value::Bool(true)), "true");
        assert_eq!(value_to_string(Value::Bool(false)), "false");
        assert_eq!(value_to_string(Value::I64(42)), "42");
        assert_eq!(value_to_string(Value::F64(2.5)), "2.5");
        assert_eq!(value_to_string(Value::String("test".to_string())), "test");
        assert_eq!(
            value_to_string(Value::StringList(vec!["a".to_string(), "b".to_string()])),
            "a,b"
        );
    }

    #[test]
    fn test_json_to_value_conversions() {
        use serde_json::json;

        // Test bool
        let result = json_to_value(json!(true)).unwrap();
        assert!(matches!(result, Value::Bool(true)));

        // Test integer
        let result = json_to_value(json!(42)).unwrap();
        assert!(matches!(result, Value::I64(42)));

        // Test float
        let result = json_to_value(json!(2.5)).unwrap();
        assert!(matches!(result, Value::F64(_)));

        // Test string
        let result = json_to_value(json!("hello")).unwrap();
        assert!(matches!(result, Value::String(s) if s == "hello"));

        // Test string array
        let result = json_to_value(json!(["a", "b", "c"])).unwrap();
        match result {
            Value::StringList(list) => {
                assert_eq!(list.len(), 3);
                assert_eq!(list[0], "a");
                assert_eq!(list[1], "b");
                assert_eq!(list[2], "c");
            }
            _ => panic!("Expected StringList"),
        }

        // Test null should error
        assert!(json_to_value(json!(null)).is_err());

        // Test nested object should error
        assert!(json_to_value(json!({"key": "value"})).is_err());
    }

    #[test]
    fn test_convert_parameters_from_json() {
        use serde_json::json;

        // Test single parameter object
        let params = vec![json!({
            "use_sim_time": false,
            "publish_rate": 10,
        })];

        let result = convert_parameters(params).unwrap();
        assert_eq!(result.len(), 2);

        // Check use_sim_time
        let use_sim_time = result.get(&ParamName::from_str("use_sim_time").unwrap());
        assert!(use_sim_time.is_some());
        match use_sim_time.unwrap() {
            ValueOrExpr::Value(Value::Bool(false)) => {}
            _ => panic!("Expected bool false"),
        }

        // Check publish_rate
        let publish_rate = result.get(&ParamName::from_str("publish_rate").unwrap());
        assert!(publish_rate.is_some());
        match publish_rate.unwrap() {
            ValueOrExpr::Value(Value::I64(10)) => {}
            _ => panic!("Expected i64 10"),
        }
    }

    #[test]
    fn test_load_launch_as_nodes_integration() {
        let test_launch = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/launch_files/simple_node.launch.py");

        if !test_launch.exists() {
            println!("Skipping integration test - launch file not found");
            return;
        }

        let args = IndexMap::new();
        let result = load_launch_as_nodes(&test_launch, args, None);

        match result {
            Ok(nodes) => {
                println!("✅ Successfully loaded nodes from launch file");
                assert_eq!(nodes.len(), 1, "Expected 1 node");

                // Get the node (we don't know the exact identifier)
                let (ident, cfg) = nodes.iter().next().unwrap();
                println!("  Node identifier: {}", ident);

                // Verify the node configuration
                assert!(cfg.pkg.is_some());
                assert!(cfg.exec.is_some());

                if let Some(TextOrExpr::Text(pkg)) = &cfg.pkg {
                    assert_eq!(pkg, "demo_nodes_cpp");
                }

                if let Some(TextOrExpr::Text(exec)) = &cfg.exec {
                    assert_eq!(exec, "talker");
                }

                // Verify parameters were converted
                assert!(!cfg.param.is_empty(), "Expected parameters");
            }
            Err(Error::LaunchFileLoadError { error, .. }) => {
                // Check if launch2dump is available
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
    fn test_load_multi_node_launch_integration() {
        let test_launch = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/launch_files/multi_node.launch.py");

        if !test_launch.exists() {
            println!("Skipping integration test - launch file not found");
            return;
        }

        let args = IndexMap::new();
        let result = load_launch_as_nodes(&test_launch, args, None);

        match result {
            Ok(nodes) => {
                println!("✅ Successfully loaded multiple nodes from launch file");
                assert_eq!(nodes.len(), 2, "Expected 2 nodes");

                // Verify both nodes have valid configurations
                for (ident, cfg) in &nodes {
                    println!("  Node: {}", ident);
                    assert!(cfg.pkg.is_some());
                    assert!(cfg.exec.is_some());
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
    fn test_load_launch_with_arguments() {
        let test_launch = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/launch_files/with_args.launch.py");

        if !test_launch.exists() {
            println!("Skipping integration test - launch file not found");
            return;
        }

        let mut args = IndexMap::new();
        args.insert(
            ParamName::from_str("node_name").unwrap(),
            Value::String("my_custom_node".to_string()),
        );
        args.insert(ParamName::from_str("rate").unwrap(), Value::I64(20));

        let result = load_launch_as_nodes(&test_launch, args, None);

        match result {
            Ok(nodes) => {
                println!("✅ Successfully loaded launch file with arguments");
                assert_eq!(nodes.len(), 1, "Expected 1 node");

                // The node should be configured with the custom arguments
                let (ident, cfg) = nodes.iter().next().unwrap();
                println!("  Node identifier: {}", ident);

                // Should have parameters
                assert!(!cfg.param.is_empty(), "Expected parameters");
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
}
