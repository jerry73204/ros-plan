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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeSubCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub from: Option<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeSrvCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeCliCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub from: Option<KeyOrExpr>,
}
