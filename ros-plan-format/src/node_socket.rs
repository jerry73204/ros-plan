use crate::{
    ident::IdentOwned, interface_type::InterfaceTypeOwned, key::KeyOwned,
    qos_requirement::QosRequirement,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type NodeSocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(transparent)]
pub struct NodeSocketTable(pub IndexMap<NodeSocketIdent, NodeSocketCfg>);

impl Default for NodeSocketTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

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
    pub from: Option<KeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeSubscriptionCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub from: Option<KeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeServerCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeClientCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOwned>,
}
