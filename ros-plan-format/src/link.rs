use crate::{ident::IdentOwned, interface_type::InterfaceTypeOwned, key::KeyOwned, qos::Qos};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type LinkIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(transparent)]
pub struct LinkTable(pub IndexMap<LinkIdent, LinkCfg>);

impl Default for LinkTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

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
    #[serde(rename = "type")]
    pub ty: InterfaceTypeOwned,

    #[serde(default)]
    pub qos: Qos,

    pub src: Vec<KeyOwned>,
    pub dst: Vec<KeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServiceLinkCfg {
    #[serde(rename = "type")]
    pub ty: InterfaceTypeOwned,
    pub listen: KeyOwned,
    pub connect: Vec<KeyOwned>,
}
