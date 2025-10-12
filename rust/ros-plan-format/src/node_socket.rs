use crate::{
    expr::{KeyOrExpr, TextOrExpr},
    ident::IdentOwned,
    interface_type::InterfaceTypeOwned,
    qos_requirement::QosRequirement,
};
use serde::{Deserialize, Serialize};

pub type NodeSocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeSocketCfg {
    #[serde(rename = "pub")]
    Pub(NodePubCfg),

    #[serde(rename = "sub")]
    Sub(NodeSubCfg),

    #[serde(rename = "srv")]
    Srv(NodeSrvCfg),

    #[serde(rename = "cli")]
    Cli(NodeCliCfg),
}

impl From<NodeCliCfg> for NodeSocketCfg {
    fn from(v: NodeCliCfg) -> Self {
        Self::Cli(v)
    }
}

impl From<NodeSrvCfg> for NodeSocketCfg {
    fn from(v: NodeSrvCfg) -> Self {
        Self::Srv(v)
    }
}

impl From<NodeSubCfg> for NodeSocketCfg {
    fn from(v: NodeSubCfg) -> Self {
        Self::Sub(v)
    }
}

impl From<NodePubCfg> for NodeSocketCfg {
    fn from(v: NodePubCfg) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodePubCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub from: Option<KeyOrExpr>,
    pub ros_name: Option<TextOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeSubCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub from: Option<KeyOrExpr>,
    pub ros_name: Option<TextOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeSrvCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOrExpr>,
    pub ros_name: Option<TextOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeCliCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOrExpr>,
    pub ros_name: Option<TextOrExpr>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_pub_cfg_with_ros_name_literal() {
        let yaml = r#"
type: std_msgs/msg/String
ros_name: custom_topic
"#;
        let result: Result<NodePubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_pub_cfg_with_ros_name_expr() {
        let yaml = r#"
type: std_msgs/msg/String
ros_name: $ "topic_" .. robot_id $
"#;
        let result: Result<NodePubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_sub_cfg_with_ros_name_literal() {
        let yaml = r#"
type: std_msgs/msg/String
ros_name: custom_topic
"#;
        let result: Result<NodeSubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_sub_cfg_with_ros_name_expr() {
        let yaml = r#"
type: std_msgs/msg/String
ros_name: $ "topic_" .. robot_id $
"#;
        let result: Result<NodeSubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_srv_cfg_with_ros_name_literal() {
        let yaml = r#"
type: std_srvs/srv/Trigger
ros_name: custom_service
"#;
        let result: Result<NodeSrvCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_srv_cfg_with_ros_name_expr() {
        let yaml = r#"
type: std_srvs/srv/Trigger
ros_name: $ "service_" .. robot_id $
"#;
        let result: Result<NodeSrvCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_cli_cfg_with_ros_name_literal() {
        let yaml = r#"
type: std_srvs/srv/Trigger
ros_name: custom_service
"#;
        let result: Result<NodeCliCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_cli_cfg_with_ros_name_expr() {
        let yaml = r#"
type: std_srvs/srv/Trigger
ros_name: $ "service_" .. robot_id $
"#;
        let result: Result<NodeCliCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_some());
    }

    #[test]
    fn node_pub_cfg_without_ros_name() {
        let yaml = r#"
type: std_msgs/msg/String
"#;
        let result: Result<NodePubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.ros_name.is_none());
    }
}
