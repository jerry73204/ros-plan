use crate::{
    argument::ArgEntry,
    expr::ValueOrExpr,
    key::RelativeKeyOwned,
    link::{LinkCfg, LinkIdent},
    node::{NodeCfg, NodeIdent},
    parameter::ParamName,
    plan_socket::{PlanSocketCfg, PlanSocketIdent},
    subplan::{GroupCfg, IncludeCfg},
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Plan {
    #[serde(default)]
    pub arg: IndexMap<ParamName, ArgEntry>,

    #[serde(default)]
    pub var: IndexMap<ParamName, ValueOrExpr>,

    #[serde(default)]
    pub socket: IndexMap<PlanSocketIdent, PlanSocketCfg>,

    #[serde(default)]
    pub node: IndexMap<NodeIdent, NodeCfg>,

    #[serde(default)]
    pub link: IndexMap<LinkIdent, LinkCfg>,

    #[serde(default)]
    pub include: IndexMap<RelativeKeyOwned, IncludeCfg>,

    #[serde(default)]
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_empty_plan() {
        let yaml = "";
        let result: Result<Plan, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let plan = result.unwrap();
        assert!(plan.arg.is_empty());
        assert!(plan.var.is_empty());
        assert!(plan.socket.is_empty());
        assert!(plan.node.is_empty());
        assert!(plan.link.is_empty());
        assert!(plan.include.is_empty());
        assert!(plan.group.is_empty());
    }

    #[test]
    fn parse_plan_with_node() {
        let yaml = r#"
node:
  talker:
    pkg: demo_nodes_cpp
    exec: talker
"#;
        let result: Result<Plan, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let plan = result.unwrap();
        assert_eq!(plan.node.len(), 1);
        assert!(plan.node.iter().any(|(k, _)| k.as_str() == "talker"));
    }

    #[test]
    fn parse_plan_with_link() {
        let yaml = r#"
link:
  chatter: !pubsub
    type: std_msgs/msg/String
    src: [talker/output]
    dst: [listener/input]
"#;
        let result: Result<Plan, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let plan = result.unwrap();
        assert_eq!(plan.link.len(), 1);
        assert!(plan.link.iter().any(|(k, _)| k.as_str() == "chatter"));
    }

    #[test]
    fn parse_plan_with_all_sections() {
        let yaml = r#"
arg:
  count:
    type: "i64"
    default: !i64 1

var:
  topic: !str "/test/topic"

socket:
  output: !pub
    type: std_msgs/msg/String
    src: [test_node/internal_out]

node:
  test_node:
    pkg: test_pkg
    exec: test_exec

link:
  test_link: !pubsub
    type: std_msgs/msg/String
    src: [test_node/out]
    dst: [output]

include:
  subplan: !file
    path: sub.yaml

group:
  test_group:
    node:
      grouped_node:
        pkg: test_pkg
        exec: test_exec
"#;
        let result: Result<Plan, _> = serde_yaml::from_str(yaml);
        if let Err(e) = &result {
            eprintln!("Parse error: {}", e);
        }
        assert!(result.is_ok());
        let plan = result.unwrap();
        assert_eq!(plan.arg.len(), 1);
        assert_eq!(plan.var.len(), 1);
        assert_eq!(plan.socket.len(), 1);
        assert_eq!(plan.node.len(), 1);
        assert_eq!(plan.link.len(), 1);
        assert_eq!(plan.include.len(), 1);
        assert_eq!(plan.group.len(), 1);
    }

    #[test]
    fn reject_unknown_fields() {
        let yaml = r#"
unknown_field: value
node:
  test:
    pkg: test
    exec: test
"#;
        let result: Result<Plan, _> = serde_yaml::from_str(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn parse_plan_with_multiple_nodes() {
        let yaml = r#"
node:
  node1:
    pkg: pkg1
    exec: exec1
  node2:
    pkg: pkg2
    exec: exec2
  node3:
    pkg: pkg3
    exec: exec3
"#;
        let result: Result<Plan, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let plan = result.unwrap();
        assert_eq!(plan.node.len(), 3);
        assert!(plan.node.iter().any(|(k, _)| k.as_str() == "node1"));
        assert!(plan.node.iter().any(|(k, _)| k.as_str() == "node2"));
        assert!(plan.node.iter().any(|(k, _)| k.as_str() == "node3"));
    }
}
