use crate::{
    expr::{BoolExpr, TextOrExpr, ValueOrExpr},
    ident::IdentOwned,
    node_socket::{NodeSocketCfg, NodeSocketIdent},
    parameter::ParamName,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type NodeIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeCfg {
    pub pkg: Option<TextOrExpr>,
    pub exec: Option<TextOrExpr>,
    pub plugin: Option<TextOrExpr>,
    pub when: Option<BoolExpr>,

    #[serde(default)]
    pub param: IndexMap<ParamName, ValueOrExpr>,

    #[serde(default)]
    pub socket: IndexMap<NodeSocketIdent, NodeSocketCfg>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_node_minimal() {
        let yaml = r#"
pkg: test_pkg
exec: test_exec
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let node = result.unwrap();
        assert!(node.pkg.is_some());
        assert!(node.exec.is_some());
        assert!(node.plugin.is_none());
        assert!(node.when.is_none());
        assert!(node.param.is_empty());
        assert!(node.socket.is_empty());
    }

    #[test]
    fn parse_node_with_params() {
        let yaml = r#"
pkg: test_pkg
exec: test_exec
param:
  rate: !i64 10
  topic: !str "/test/topic"
  enabled: !bool true
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let node = result.unwrap();
        assert_eq!(node.param.len(), 3);
    }

    #[test]
    fn parse_node_with_sockets() {
        let yaml = r#"
pkg: test_pkg
exec: test_exec
socket:
  input: !sub
    type: std_msgs/msg/String
  output: !pub
    type: std_msgs/msg/String
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let node = result.unwrap();
        assert_eq!(node.socket.len(), 2);
    }

    #[test]
    fn parse_node_with_when_condition() {
        let yaml = r#"
pkg: test_pkg
exec: test_exec
when: $ enable_node $
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let node = result.unwrap();
        assert!(node.when.is_some());
    }

    #[test]
    fn parse_node_with_plugin() {
        let yaml = r#"
plugin: test_plugin::PluginClass
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let node = result.unwrap();
        assert!(node.plugin.is_some());
        assert!(node.pkg.is_none());
        assert!(node.exec.is_none());
    }

    #[test]
    fn reject_node_unknown_fields() {
        let yaml = r#"
pkg: test_pkg
exec: test_exec
unknown_field: value
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn parse_node_with_expressions() {
        let yaml = r#"
pkg: $ pkg_name $
exec: $ exec_name $
param:
  rate: !i64 $ base_rate * 2 $
"#;
        let result: Result<NodeCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let node = result.unwrap();
        assert!(node.pkg.is_some());
        assert!(node.exec.is_some());
        assert_eq!(node.param.len(), 1);
    }
}
