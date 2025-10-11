use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, parameter::ParamName};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::PathBuf};

/// Tracks launch file includes and their associated nodes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchTracker {
    /// Maps include identifier to launch include metadata
    pub includes: HashMap<String, LaunchInclude>,
    /// Maps node identifier to the include that created it
    pub node_sources: HashMap<KeyOwned, String>,
}

/// Information about a launch file include
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchInclude {
    /// Path to the launch file
    pub file_path: PathBuf,
    /// Arguments passed to the launch file
    pub arguments: IndexMap<String, String>,
    /// Nodes created by this include
    pub node_idents: Vec<KeyOwned>,
    /// Plan parameters used in include arguments
    pub parameter_deps: Vec<ParamName>,
}

impl LaunchTracker {
    pub fn new() -> Self {
        Self {
            includes: HashMap::new(),
            node_sources: HashMap::new(),
        }
    }

    /// Register a launch include
    pub fn register_include(
        &mut self,
        include_name: String,
        file_path: PathBuf,
        arguments: IndexMap<String, String>,
    ) {
        // Extract parameter dependencies from arguments
        let parameter_deps = Self::extract_param_deps(&arguments);

        self.includes.insert(
            include_name,
            LaunchInclude {
                file_path,
                arguments,
                node_idents: Vec::new(),
                parameter_deps,
            },
        );
    }

    /// Associate a node with an include
    pub fn register_node(&mut self, include_name: &str, node_id: KeyOwned) {
        if let Some(include) = self.includes.get_mut(include_name) {
            if !include.node_idents.contains(&node_id) {
                include.node_idents.push(node_id.clone());
            }
        }
        self.node_sources.insert(node_id, include_name.to_string());
    }

    /// Get the include that created a node
    pub fn get_node_source(&self, node_id: &KeyOwned) -> Option<&str> {
        self.node_sources.get(node_id).map(|s| s.as_str())
    }

    /// Get nodes created by an include
    pub fn get_include_nodes(&self, include_name: &str) -> Vec<KeyOwned> {
        self.includes
            .get(include_name)
            .map(|inc| inc.node_idents.clone())
            .unwrap_or_default()
    }

    /// Check if an include needs reload based on parameter changes
    pub fn needs_reload(
        &self,
        include_name: &str,
        new_params: &IndexMap<ParamName, ros_plan_format::expr::Value>,
    ) -> bool {
        if let Some(include) = self.includes.get(include_name) {
            // Check if any of the include's parameter dependencies changed
            for param_dep in &include.parameter_deps {
                if new_params.contains_key(param_dep) {
                    return true;
                }
            }
        }
        false
    }

    /// Extract parameter dependencies from arguments
    fn extract_param_deps(arguments: &IndexMap<String, String>) -> Vec<ParamName> {
        let mut deps = Vec::new();
        for value in arguments.values() {
            // Look for $(param name) patterns
            if let Some(param_name) = Self::extract_param_reference(value) {
                if let Ok(parsed) = param_name.parse::<ParamName>() {
                    if !deps.contains(&parsed) {
                        deps.push(parsed);
                    }
                }
            }
        }
        deps
    }

    /// Extract parameter name from $(param name) pattern
    fn extract_param_reference(text: &str) -> Option<String> {
        // Simple regex-free extraction for $(param xxx)
        if let Some(start) = text.find("$(param ") {
            if let Some(end) = text[start..].find(')') {
                let param_text = &text[start + 8..start + end];
                return Some(param_text.trim().to_string());
            }
        }
        None
    }
}

impl Default for LaunchTracker {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of checking which includes need reload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReloadCheck {
    /// Includes that need reload due to parameter changes
    pub needs_reload: Vec<String>,
    /// Includes that don't need reload
    pub unchanged: Vec<String>,
}

/// Diff for launch include changes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchDiff {
    /// Includes that were added
    pub added_includes: Vec<String>,
    /// Includes that were removed
    pub removed_includes: Vec<String>,
    /// Includes that had their arguments changed
    pub modified_includes: Vec<String>,
    /// Nodes affected by changes
    pub affected_nodes: Vec<KeyOwned>,
}

impl LaunchTracker {
    /// Check which includes need reload given new parameters
    pub fn check_reload_needed(
        &self,
        new_params: &IndexMap<ParamName, ros_plan_format::expr::Value>,
    ) -> ReloadCheck {
        let mut needs_reload = Vec::new();
        let mut unchanged = Vec::new();

        for include_name in self.includes.keys() {
            if self.needs_reload(include_name, new_params) {
                needs_reload.push(include_name.clone());
            } else {
                unchanged.push(include_name.clone());
            }
        }

        ReloadCheck {
            needs_reload,
            unchanged,
        }
    }

    /// Compute diff between two launch trackers
    pub fn diff(&self, other: &LaunchTracker) -> LaunchDiff {
        let old_keys: std::collections::HashSet<_> = self.includes.keys().collect();
        let new_keys: std::collections::HashSet<_> = other.includes.keys().collect();

        let added_includes: Vec<String> = new_keys
            .difference(&old_keys)
            .map(|s| s.to_string())
            .collect();

        let removed_includes: Vec<String> = old_keys
            .difference(&new_keys)
            .map(|s| s.to_string())
            .collect();

        let mut modified_includes = Vec::new();
        let mut affected_nodes = Vec::new();

        for include_name in old_keys.intersection(&new_keys) {
            if let (Some(old_inc), Some(new_inc)) = (
                self.includes.get(*include_name),
                other.includes.get(*include_name),
            ) {
                if old_inc.arguments != new_inc.arguments {
                    modified_includes.push(include_name.to_string());
                    affected_nodes.extend(old_inc.node_idents.clone());
                }
            }
        }

        // Add nodes from removed includes
        for include_name in &removed_includes {
            if let Some(inc) = self.includes.get(include_name) {
                affected_nodes.extend(inc.node_idents.clone());
            }
        }

        LaunchDiff {
            added_includes,
            removed_includes,
            modified_includes,
            affected_nodes,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_launch_tracker_creation() {
        let tracker = LaunchTracker::new();
        assert!(tracker.includes.is_empty());
        assert!(tracker.node_sources.is_empty());
    }

    #[test]
    fn test_register_include() {
        let mut tracker = LaunchTracker::new();
        let mut args = IndexMap::new();
        args.insert("device".to_string(), "/dev/video0".to_string());

        tracker.register_include(
            "camera_include".to_string(),
            PathBuf::from("/path/to/camera.launch.py"),
            args,
        );

        assert_eq!(tracker.includes.len(), 1);
        assert!(tracker.includes.contains_key("camera_include"));
    }

    #[test]
    fn test_register_node() {
        let mut tracker = LaunchTracker::new();
        tracker.register_include(
            "camera_include".to_string(),
            PathBuf::from("/path/to/camera.launch.py"),
            IndexMap::new(),
        );

        let node_id: KeyOwned = "camera_node".parse().unwrap();
        tracker.register_node("camera_include", node_id.clone());

        assert_eq!(tracker.get_node_source(&node_id), Some("camera_include"));
        assert_eq!(tracker.get_include_nodes("camera_include").len(), 1);
    }

    #[test]
    fn test_extract_param_reference() {
        assert_eq!(
            LaunchTracker::extract_param_reference("$(param camera_id)"),
            Some("camera_id".to_string())
        );
        assert_eq!(
            LaunchTracker::extract_param_reference("prefix $(param name) suffix"),
            Some("name".to_string())
        );
        assert_eq!(LaunchTracker::extract_param_reference("no params"), None);
    }

    #[test]
    fn test_extract_param_deps() {
        let mut args = IndexMap::new();
        args.insert("device".to_string(), "$(param camera_device)".to_string());
        args.insert("rate".to_string(), "$(param frame_rate)".to_string());
        args.insert("static".to_string(), "30".to_string());

        let deps = LaunchTracker::extract_param_deps(&args);
        assert_eq!(deps.len(), 2);
    }
}
