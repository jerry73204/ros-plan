use indexmap::IndexMap;
use ros_plan_compiler::{
    context::node::NodeCtx,
    eval::{TextStore, ValueStore},
    utils::shared_table::SharedTable,
    Program,
};
use ros_plan_format::{key::KeyOwned, parameter::ParamName};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// Diff between two compiled programs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProgramDiff {
    pub added_nodes: Vec<KeyOwned>,
    pub removed_nodes: Vec<KeyOwned>,
    pub modified_nodes: Vec<NodeModification>,
    pub unchanged_nodes: Vec<KeyOwned>,
}

/// Information about a modified node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeModification {
    pub ident: KeyOwned,
    pub changes: NodeChanges,
}

/// Specific changes to a node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeChanges {
    pub params_changed: bool,
    pub namespace_changed: bool,
    pub executable_changed: bool,
    pub package_changed: bool,
    pub plugin_changed: bool,
}

impl NodeChanges {
    /// Check if any changes occurred
    pub fn has_changes(&self) -> bool {
        self.params_changed
            || self.namespace_changed
            || self.executable_changed
            || self.package_changed
            || self.plugin_changed
    }
}

/// Compare two programs and identify differences
pub fn diff_programs(old: &Program, new: &Program) -> ProgramDiff {
    // Extract node identifiers from both programs
    let old_nodes: HashSet<KeyOwned> = old
        .node_tab
        .read()
        .iter()
        .map(|(_, node_shared)| {
            let mut key = None;
            node_shared.with_read(|node| {
                key = Some(node.key.clone());
            });
            key.unwrap()
        })
        .collect();

    let new_nodes: HashSet<KeyOwned> = new
        .node_tab
        .read()
        .iter()
        .map(|(_, node_shared)| {
            let mut key = None;
            node_shared.with_read(|node| {
                key = Some(node.key.clone());
            });
            key.unwrap()
        })
        .collect();

    // Detect added nodes (in new but not in old)
    let added_nodes: Vec<KeyOwned> = new_nodes.difference(&old_nodes).cloned().collect();

    // Detect removed nodes (in old but not in new)
    let removed_nodes: Vec<KeyOwned> = old_nodes.difference(&new_nodes).cloned().collect();

    // Detect modified and unchanged nodes
    let mut modified_nodes = Vec::new();
    let mut unchanged_nodes = Vec::new();

    for node_id in old_nodes.intersection(&new_nodes) {
        // Get old and new node contexts
        let old_node = find_node_by_key(&old.node_tab, node_id);
        let new_node = find_node_by_key(&new.node_tab, node_id);

        if let (Some(old_ctx), Some(new_ctx)) = (old_node, new_node) {
            let changes = compare_nodes(&old_ctx, &new_ctx);
            if changes.has_changes() {
                modified_nodes.push(NodeModification {
                    ident: node_id.clone(),
                    changes,
                });
            } else {
                unchanged_nodes.push(node_id.clone());
            }
        }
    }

    ProgramDiff {
        added_nodes,
        removed_nodes,
        modified_nodes,
        unchanged_nodes,
    }
}

/// Find a node by its key in the node table
fn find_node_by_key(node_tab: &SharedTable<NodeCtx>, key: &KeyOwned) -> Option<NodeCtx> {
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

/// Compare two node contexts and identify changes
fn compare_nodes(old: &NodeCtx, new: &NodeCtx) -> NodeChanges {
    let params_changed = !params_equal(&old.param, &new.param);
    let namespace_changed = old.namespace != new.namespace;
    let executable_changed = !text_store_equal(&old.exec, &new.exec);
    let package_changed = !text_store_equal(&old.pkg, &new.pkg);
    let plugin_changed = !text_store_equal(&old.plugin, &new.plugin);

    NodeChanges {
        params_changed,
        namespace_changed,
        executable_changed,
        package_changed,
        plugin_changed,
    }
}

/// Compare two parameter maps for equality
fn params_equal(
    old: &IndexMap<ParamName, ValueStore>,
    new: &IndexMap<ParamName, ValueStore>,
) -> bool {
    if old.len() != new.len() {
        return false;
    }

    for (key, old_value) in old.iter() {
        match new.get(key) {
            Some(new_value) => {
                // Compare stored values using custom comparison
                let old_stored = old_value.get_stored().ok();
                let new_stored = new_value.get_stored().ok();

                match (old_stored, new_stored) {
                    (Some(old_val), Some(new_val)) => {
                        if !values_equal(old_val, new_val) {
                            return false;
                        }
                    }
                    (None, None) => {}
                    _ => return false,
                }
            }
            None => return false,
        }
    }

    true
}

/// Compare two Values for equality (custom implementation since Value doesn't derive PartialEq)
fn values_equal(a: &ros_plan_format::expr::Value, b: &ros_plan_format::expr::Value) -> bool {
    use ros_plan_format::expr::Value;

    match (a, b) {
        (Value::Bool(a), Value::Bool(b)) => a == b,
        (Value::I64(a), Value::I64(b)) => a == b,
        (Value::F64(a), Value::F64(b)) => {
            // For F64, use bit pattern comparison to handle NaN correctly
            a.to_bits() == b.to_bits()
        }
        (Value::String(a), Value::String(b)) => a == b,
        (Value::Key(a), Value::Key(b)) => a == b,
        (Value::Path(a), Value::Path(b)) => a == b,
        (Value::Binary(a), Value::Binary(b)) => a == b,
        (Value::BoolList(a), Value::BoolList(b)) => a == b,
        (Value::I64List(a), Value::I64List(b)) => a == b,
        (Value::F64List(a), Value::F64List(b)) => {
            if a.len() != b.len() {
                return false;
            }
            a.iter()
                .zip(b.iter())
                .all(|(x, y)| x.to_bits() == y.to_bits())
        }
        (Value::StringList(a), Value::StringList(b)) => a == b,
        _ => false, // Different variants
    }
}

/// Compare two TextStore options for equality
fn text_store_equal(old: &Option<TextStore>, new: &Option<TextStore>) -> bool {
    match (old, new) {
        (Some(old_text), Some(new_text)) => {
            old_text.get_stored().ok() == new_text.get_stored().ok()
        }
        (None, None) => true,
        _ => false,
    }
}

/// Result of applying a diff
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApplyResult {
    pub stopped: Vec<KeyOwned>,
    pub restarted: Vec<KeyOwned>,
    pub started: Vec<KeyOwned>,
    pub failures: Vec<(KeyOwned, String)>,
}

impl ApplyResult {
    pub fn success(&self) -> bool {
        self.failures.is_empty()
    }
}

/// Result of a parameter update operation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UpdateResult {
    pub success: bool,
    pub diff: ProgramDiff,
    pub applied: ApplyResult,
    pub errors: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_changes_has_changes() {
        let changes = NodeChanges {
            params_changed: true,
            namespace_changed: false,
            executable_changed: false,
            package_changed: false,
            plugin_changed: false,
        };
        assert!(changes.has_changes());

        let no_changes = NodeChanges {
            params_changed: false,
            namespace_changed: false,
            executable_changed: false,
            package_changed: false,
            plugin_changed: false,
        };
        assert!(!no_changes.has_changes());
    }
}
