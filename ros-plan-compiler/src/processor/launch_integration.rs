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
