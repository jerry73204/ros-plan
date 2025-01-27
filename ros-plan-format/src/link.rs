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
