use crate::{
    expr::{KeyOrExpr, TextOrExpr},
    ident::IdentOwned,
    interface_type::InterfaceTypeOwned,
    qos_requirement::QosRequirement,
};
use serde::{Deserialize, Serialize};

pub type PlanSocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PlanSocketCfg {
    #[serde(rename = "pub")]
    Pub(PlanPubCfg),

    #[serde(rename = "sub")]
    Sub(PlanSubCfg),

    #[serde(rename = "srv")]
    Srv(PlanSrvCfg),

    #[serde(rename = "cli")]
    Cli(PlanCliCfg),
}

impl From<PlanCliCfg> for PlanSocketCfg {
    fn from(v: PlanCliCfg) -> Self {
        Self::Cli(v)
    }
}

impl From<PlanSrvCfg> for PlanSocketCfg {
    fn from(v: PlanSrvCfg) -> Self {
        Self::Srv(v)
    }
}

impl From<PlanSubCfg> for PlanSocketCfg {
    fn from(v: PlanSubCfg) -> Self {
        Self::Sub(v)
    }
}

impl From<PlanPubCfg> for PlanSocketCfg {
    fn from(v: PlanPubCfg) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanPubCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub topic: Option<TextOrExpr>,
    pub src: Vec<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSubCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub topic: Option<TextOrExpr>,
    pub dst: Vec<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSrvCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub listen: KeyOrExpr,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCliCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub connect: Vec<KeyOrExpr>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_plan_pub_with_topic_literal() {
        let yaml = r#"
type: std_msgs/msg/String
topic: /shared_topic
src: [node_a/output, node_b/output]
"#;
        let result: Result<PlanPubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.topic.is_some());
        assert_eq!(cfg.src.len(), 2);
    }

    #[test]
    fn parse_plan_pub_with_topic_expr() {
        let yaml = r#"
type: std_msgs/msg/String
topic: $ "/robot_" .. robot_id .. "/data" $
src: [node_a/output]
"#;
        let result: Result<PlanPubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.topic.is_some());
    }

    #[test]
    fn parse_plan_pub_without_topic() {
        let yaml = r#"
type: std_msgs/msg/String
src: [node_a/output]
"#;
        let result: Result<PlanPubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.topic.is_none());
    }

    #[test]
    fn parse_plan_pub_multi_source_with_topic() {
        let yaml = r#"
type: tf2_msgs/msg/TFMessage
topic: /tf
src: [camera/tf, lidar/tf, gnss/tf]
"#;
        let result: Result<PlanPubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.topic.is_some());
        assert_eq!(cfg.src.len(), 3);
    }

    #[test]
    fn parse_plan_sub_with_topic() {
        let yaml = r#"
type: std_msgs/msg/String
topic: /commands
dst: [node_a/input, node_b/input]
"#;
        let result: Result<PlanSubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.topic.is_some());
        assert_eq!(cfg.dst.len(), 2);
    }

    #[test]
    fn parse_plan_sub_without_topic() {
        let yaml = r#"
type: std_msgs/msg/String
dst: [node_a/input]
"#;
        let result: Result<PlanSubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let cfg = result.unwrap();
        assert!(cfg.topic.is_none());
    }

    #[test]
    fn plan_pub_roundtrip_serialization() {
        let cfg = PlanPubCfg {
            ty: Some("std_msgs/msg/String".parse().unwrap()),
            qos: None,
            topic: Some("/shared_topic".to_string().into()),
            src: vec!["node_a/out".parse().unwrap()],
        };

        let yaml = serde_yaml::to_string(&cfg).unwrap();
        let parsed: PlanPubCfg = serde_yaml::from_str(&yaml).unwrap();
        assert!(parsed.topic.is_some());
    }

    #[test]
    fn reject_plan_pub_unknown_fields() {
        let yaml = r#"
type: std_msgs/msg/String
src: [node_a/out]
unknown_field: value
"#;
        let result: Result<PlanPubCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_err());
    }
}
