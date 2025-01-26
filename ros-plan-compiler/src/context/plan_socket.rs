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
    Pub(PlanPubCtx),

    #[serde(rename = "sub")]
    Sub(PlanSubCtx),

    #[serde(rename = "srv")]
    Srv(PlanSrvCtx),

    #[serde(rename = "cli")]
    Cli(PlanCliCtx),
}

impl PlanSocketCtx {
    pub fn key(&self) -> &Key {
        match self {
            PlanSocketCtx::Pub(socket) => &socket.key,
            PlanSocketCtx::Sub(socket) => &socket.key,
            PlanSocketCtx::Srv(socket) => &socket.key,
            PlanSocketCtx::Cli(socket) => &socket.key,
        }
    }

    pub fn as_pub(&self) -> Option<&PlanPubCtx> {
        if let Self::Pub(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_sub(&self) -> Option<&PlanSubCtx> {
        if let Self::Sub(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_srv(&self) -> Option<&PlanSrvCtx> {
        if let Self::Srv(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_cli(&self) -> Option<&PlanCliCtx> {
        if let Self::Cli(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PlanCliCtx> for PlanSocketCtx {
    fn from(v: PlanCliCtx) -> Self {
        Self::Cli(v)
    }
}

impl From<PlanSrvCtx> for PlanSocketCtx {
    fn from(v: PlanSrvCtx) -> Self {
        Self::Srv(v)
    }
}

impl From<PlanSubCtx> for PlanSocketCtx {
    fn from(v: PlanSubCtx) -> Self {
        Self::Sub(v)
    }
}

impl From<PlanPubCtx> for PlanSocketCtx {
    fn from(v: PlanPubCtx) -> Self {
        Self::Pub(v)
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
