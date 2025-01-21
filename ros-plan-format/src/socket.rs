use crate::{ident::IdentOwned, key::RelativeKeyOwned};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type SocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SocketTable(pub IndexMap<SocketIdent, SocketCfg>);

impl Default for SocketTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SocketCfg {
    Pub(PubSocketCfg),
    Sub(SubSocketCfg),
    Srv(ServerSocketCfg),
    Qry(QuerySocketCfg),
}

impl From<QuerySocketCfg> for SocketCfg {
    fn from(v: QuerySocketCfg) -> Self {
        Self::Qry(v)
    }
}

impl From<ServerSocketCfg> for SocketCfg {
    fn from(v: ServerSocketCfg) -> Self {
        Self::Srv(v)
    }
}

impl From<SubSocketCfg> for SocketCfg {
    fn from(v: SubSocketCfg) -> Self {
        Self::Sub(v)
    }
}

impl From<PubSocketCfg> for SocketCfg {
    fn from(v: PubSocketCfg) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PubSocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    // pub qos: Option<QosRequirement>,
    pub src: Vec<RelativeKeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SubSocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    // pub qos: Option<QosRequirement>,
    pub dst: Vec<RelativeKeyOwned>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServerSocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    pub listen: RelativeKeyOwned,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QuerySocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    pub connect: Vec<RelativeKeyOwned>,
}
