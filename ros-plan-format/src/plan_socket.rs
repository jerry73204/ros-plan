use crate::{ident::IdentOwned, key::RelativeKeyOwned};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type PlanSocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(transparent)]
pub struct PlanSocketTable(pub IndexMap<PlanSocketIdent, PlanSocketCfg>);

impl Default for PlanSocketTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PlanSocketCfg {
    #[serde(rename = "pub")]
    Publication(PlanPublicationCfg),
    #[serde(rename = "sub")]
    Subscription(PlanSubscriptionCfg),
    #[serde(rename = "srv")]
    Server(PlanServerCfg),
    #[serde(rename = "cli")]
    Client(PlanClientCfg),
}

impl From<PlanClientCfg> for PlanSocketCfg {
    fn from(v: PlanClientCfg) -> Self {
        Self::Client(v)
    }
}

impl From<PlanServerCfg> for PlanSocketCfg {
    fn from(v: PlanServerCfg) -> Self {
        Self::Server(v)
    }
}

impl From<PlanSubscriptionCfg> for PlanSocketCfg {
    fn from(v: PlanSubscriptionCfg) -> Self {
        Self::Subscription(v)
    }
}

impl From<PlanPublicationCfg> for PlanSocketCfg {
    fn from(v: PlanPublicationCfg) -> Self {
        Self::Publication(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanPublicationCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    // pub qos: Option<QosRequirement>,
    pub src: Vec<RelativeKeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanSubscriptionCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    // pub qos: Option<QosRequirement>,
    pub dst: Vec<RelativeKeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanServerCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    pub listen: RelativeKeyOwned,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanClientCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    pub connect: Vec<RelativeKeyOwned>,
}
