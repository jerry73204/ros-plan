use crate::{
    expr::KeyOrExpr, ident::IdentOwned, interface_type::InterfaceTypeOwned,
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
    pub src: Vec<KeyOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSubCfg {
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
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
