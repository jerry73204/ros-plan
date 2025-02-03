use super::node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared};
use crate::{
    eval::KeyStore,
    utils::shared_table::{Owned, Shared},
};
use ros_plan_format::{
    interface_type::InterfaceTypeOwned,
    key::{Key, KeyOwned},
    qos_requirement::QosRequirement,
};
use serde::{Deserialize, Serialize};

pub type PlanPubOwned = Owned<PlanPubCtx>;
pub type PlanPubShared = Shared<PlanPubCtx>;

pub type PlanSubOwned = Owned<PlanSubCtx>;
pub type PlanSubShared = Shared<PlanSubCtx>;

pub type PlanSrvOwned = Owned<PlanSrvCtx>;
pub type PlanSrvShared = Shared<PlanSrvCtx>;

pub type PlanCliOwned = Owned<PlanCliCtx>;
pub type PlanCliShared = Shared<PlanCliCtx>;

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
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub src_key: Vec<KeyStore>,
    pub src_socket: Option<Vec<NodePubShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanSubCtx {
    pub key: KeyOwned,
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub qos: Option<QosRequirement>,
    pub dst_key: Vec<KeyStore>,
    pub dst_socket: Option<Vec<NodeSubShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanSrvCtx {
    pub key: KeyOwned,
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub listen_key: KeyStore,
    pub listen_socket: Option<NodeSrvShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanCliCtx {
    pub key: KeyOwned,
    #[serde(rename = "type")]
    pub ty: Option<InterfaceTypeOwned>,
    pub connect_key: Vec<KeyStore>,
    pub connect_socket: Option<Vec<NodeCliShared>>,
}
