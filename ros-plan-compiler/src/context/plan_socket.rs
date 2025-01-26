use super::node_socket::NodeSocketShared;
use crate::utils::shared_table::{Owned, Shared};
use ros_plan_format::{
    key::{Key, KeyOwned},
    plan_socket::{PlanCliCfg, PlanPubCfg, PlanSrvCfg, PlanSubCfg},
};
use serde::{Deserialize, Serialize};

pub type PlanSocketOwned = Owned<PlanSocketCtx>;
pub type PlanSocketShared = Shared<PlanSocketCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PlanSocketCtx {
    #[serde(rename = "pub")]
    Publication(PlanPubCtx),
    #[serde(rename = "sub")]
    Subscription(PlanSubCtx),
    #[serde(rename = "srv")]
    Server(PlanSrvCtx),
    #[serde(rename = "cli")]
    Client(PlanCliCtx),
}

impl PlanSocketCtx {
    pub fn key(&self) -> &Key {
        match self {
            PlanSocketCtx::Publication(socket) => &socket.key,
            PlanSocketCtx::Subscription(socket) => &socket.key,
            PlanSocketCtx::Server(socket) => &socket.key,
            PlanSocketCtx::Client(socket) => &socket.key,
        }
    }

    pub fn as_publication(&self) -> Option<&PlanPubCtx> {
        if let Self::Publication(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_subscription(&self) -> Option<&PlanSubCtx> {
        if let Self::Subscription(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_server(&self) -> Option<&PlanSrvCtx> {
        if let Self::Server(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_client(&self) -> Option<&PlanCliCtx> {
        if let Self::Client(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PlanCliCtx> for PlanSocketCtx {
    fn from(v: PlanCliCtx) -> Self {
        Self::Client(v)
    }
}

impl From<PlanSrvCtx> for PlanSocketCtx {
    fn from(v: PlanSrvCtx) -> Self {
        Self::Server(v)
    }
}

impl From<PlanSubCtx> for PlanSocketCtx {
    fn from(v: PlanSubCtx) -> Self {
        Self::Subscription(v)
    }
}

impl From<PlanPubCtx> for PlanSocketCtx {
    fn from(v: PlanPubCtx) -> Self {
        Self::Publication(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanPubCtx {
    pub key: KeyOwned,
    pub config: PlanPubCfg,
    pub src: Option<Vec<NodeSocketShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanSubCtx {
    pub key: KeyOwned,
    pub config: PlanSubCfg,
    pub dst: Option<Vec<NodeSocketShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanSrvCtx {
    pub key: KeyOwned,
    pub config: PlanSrvCfg,
    pub listen: Option<NodeSocketShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanCliCtx {
    pub key: KeyOwned,
    pub config: PlanCliCfg,
    pub connect: Option<Vec<NodeSocketShared>>,
}
