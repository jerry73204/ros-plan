use crate::{
    expr::KeyOrExpr, ident::IdentOwned, interface_type::InterfaceTypeOwned,
    qos_requirement::QosRequirement,
};
use serde::{Deserialize, Serialize};

pub type NodeSocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeSocketCfg {
    #[serde(rename = "pub")]
    Publication(NodePublicationCfg),

    #[serde(rename = "sub")]
    Subscription(NodeSubscriptionCfg),

    #[serde(rename = "srv")]
    Server(NodeServerCfg),

    #[serde(rename = "cli")]
    Client(NodeClientCfg),
}

impl From<NodeClientCfg> for NodeSocketCfg {
    fn from(v: NodeClientCfg) -> Self {
        Self::Client(v)
    }
}

impl From<NodeServerCfg> for NodeSocketCfg {
    fn from(v: NodeServerCfg) -> Self {
        Self::Server(v)
    }
}

impl From<NodeSubscriptionCfg> for NodeSocketCfg {
    fn from(v: NodeSubscriptionCfg) -> Self {
        Self::Subscription(v)
    }
}

impl From<NodePublicationCfg> for NodeSocketCfg {
    fn from(v: NodePublicationCfg) -> Self {
        Self::Publication(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodePublicationCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub from: Option<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeSubscriptionCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub from: Option<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeServerCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeClientCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOrExpr>,
}
