use crate::{
    expr::{BoolExpr, KeyOrExpr},
    ident::IdentOwned,
    interface_type::InterfaceTypeOwned,
    qos::Qos,
};
use serde::{Deserialize, Serialize};

pub type LinkIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LinkCfg {
    #[serde(rename = "pubsub")]
    PubSub(PubSubLinkCfg),

    #[serde(rename = "service")]
    Service(ServiceLinkCfg),
}

impl From<PubSubLinkCfg> for LinkCfg {
    fn from(v: PubSubLinkCfg) -> Self {
        Self::PubSub(v)
    }
}

impl From<ServiceLinkCfg> for LinkCfg {
    fn from(v: ServiceLinkCfg) -> Self {
        Self::Service(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PubSubLinkCfg {
    pub when: Option<BoolExpr>,

    #[serde(rename = "type")]
    pub ty: InterfaceTypeOwned,

    #[serde(default)]
    pub qos: Qos,

    pub src: Vec<KeyOrExpr>,
    pub dst: Vec<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServiceLinkCfg {
    pub when: Option<BoolExpr>,

    #[serde(rename = "type")]
    pub ty: InterfaceTypeOwned,
    pub listen: KeyOrExpr,
    pub connect: Vec<KeyOrExpr>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_pubsub_link_minimal() {
        let yaml = r#"
type: std_msgs/msg/String
src: [node_a/output]
dst: [node_b/input]
"#;
        let result: Result<PubSubLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let link = result.unwrap();
        assert_eq!(link.src.len(), 1);
        assert_eq!(link.dst.len(), 1);
        assert!(link.when.is_none());
    }

    #[test]
    fn parse_pubsub_link_multiple_sources() {
        let yaml = r#"
type: std_msgs/msg/String
src: [node_a/out, node_b/out, node_c/out]
dst: [processor/input]
"#;
        let result: Result<PubSubLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let link = result.unwrap();
        assert_eq!(link.src.len(), 3);
        assert_eq!(link.dst.len(), 1);
    }

    #[test]
    fn parse_pubsub_link_multiple_destinations() {
        let yaml = r#"
type: sensor_msgs/msg/Image
src: [camera/output]
dst: [viewer/input, recorder/input, processor/input]
"#;
        let result: Result<PubSubLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let link = result.unwrap();
        assert_eq!(link.src.len(), 1);
        assert_eq!(link.dst.len(), 3);
    }

    #[test]
    fn parse_pubsub_link_with_qos() {
        let yaml = r#"
type: sensor_msgs/msg/LaserScan
src: [lidar/scan]
dst: [slam/scan]
qos: !profile
  reliability: reliable
  depth: 10
"#;
        let result: Result<PubSubLinkCfg, _> = serde_yaml::from_str(yaml);
        if let Err(e) = &result {
            eprintln!("Parse error: {}", e);
        }
        assert!(result.is_ok());
    }

    #[test]
    fn parse_pubsub_link_with_when() {
        let yaml = r#"
type: std_msgs/msg/String
src: [node_a/out]
dst: [node_b/in]
when: $ enable_link $
"#;
        let result: Result<PubSubLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let link = result.unwrap();
        assert!(link.when.is_some());
    }

    #[test]
    fn parse_service_link() {
        let yaml = r#"
type: std_srvs/srv/SetBool
listen: server/service
connect: [client_a/service, client_b/service]
"#;
        let result: Result<ServiceLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let link = result.unwrap();
        assert_eq!(link.connect.len(), 2);
    }

    #[test]
    fn parse_service_link_with_when() {
        let yaml = r#"
type: std_srvs/srv/Trigger
listen: server/trigger
connect: [client/trigger]
when: $ enable_service $
"#;
        let result: Result<ServiceLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let link = result.unwrap();
        assert!(link.when.is_some());
    }

    #[test]
    fn reject_pubsub_unknown_fields() {
        let yaml = r#"
type: std_msgs/msg/String
src: [node_a/out]
dst: [node_b/in]
unknown_field: value
"#;
        let result: Result<PubSubLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn reject_service_unknown_fields() {
        let yaml = r#"
type: std_srvs/srv/Trigger
listen: server/service
connect: [client/service]
unknown_field: value
"#;
        let result: Result<ServiceLinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn parse_link_enum_pubsub() {
        let yaml = r#"!pubsub
type: std_msgs/msg/String
src: [node_a/out]
dst: [node_b/in]
"#;
        let result: Result<LinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), LinkCfg::PubSub(_)));
    }

    #[test]
    fn parse_link_enum_service() {
        let yaml = r#"!service
type: std_srvs/srv/Trigger
listen: server/service
connect: [client/service]
"#;
        let result: Result<LinkCfg, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), LinkCfg::Service(_)));
    }
}
