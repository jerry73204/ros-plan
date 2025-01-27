use super::link::{PubSubLinkShared, ServiceLinkShared};
use crate::utils::shared_table::{Owned, Shared};
use ros_plan_format::{
    key::{Key, KeyOwned},
    node_socket::{NodeCliCfg, NodePubCfg, NodeSrvCfg, NodeSubCfg},
};
use serde::{Deserialize, Serialize};

pub type NodePubOwned = Owned<NodePubCtx>;
pub type NodePubShared = Shared<NodePubCtx>;

pub type NodeSubOwned = Owned<NodeSubCtx>;
pub type NodeSubShared = Shared<NodeSubCtx>;

pub type NodeSrvOwned = Owned<NodeSrvCtx>;
pub type NodeSrvShared = Shared<NodeSrvCtx>;

pub type NodeCliOwned = Owned<NodeCliCtx>;
pub type NodeCliShared = Shared<NodeCliCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeSocketCtx {
    #[serde(rename = "pub")]
    Pub(NodePubCtx),

    #[serde(rename = "sub")]
    Sub(NodeSubCtx),

    #[serde(rename = "srv")]
    Srv(NodeSrvCtx),

    #[serde(rename = "cli")]
    Cli(NodeCliCtx),
}

impl NodeSocketCtx {
    pub fn key(&self) -> &Key {
        match self {
            NodeSocketCtx::Pub(socket) => &socket.key,
            NodeSocketCtx::Sub(socket) => &socket.key,
            NodeSocketCtx::Srv(socket) => &socket.key,
            NodeSocketCtx::Cli(socket) => &socket.key,
        }
    }

    pub fn as_pub(&self) -> Option<&NodePubCtx> {
        if let Self::Pub(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_pub_mut(&mut self) -> Option<&mut NodePubCtx> {
        if let Self::Pub(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_sub(&self) -> Option<&NodeSubCtx> {
        if let Self::Sub(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_sub_mut(&mut self) -> Option<&mut NodeSubCtx> {
        if let Self::Sub(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_srv(&self) -> Option<&NodeSrvCtx> {
        if let Self::Srv(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_srv_mut(&mut self) -> Option<&mut NodeSrvCtx> {
        if let Self::Srv(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_cli(&self) -> Option<&NodeCliCtx> {
        if let Self::Cli(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_cli_mut(&mut self) -> Option<&mut NodeCliCtx> {
        if let Self::Cli(v) = self {
            Some(v)
        } else {
            None
        }
    }

    #[must_use]
    pub fn is_pub(&self) -> bool {
        matches!(self, Self::Pub(..))
    }

    #[must_use]
    pub fn is_sub(&self) -> bool {
        matches!(self, Self::Sub(..))
    }

    #[must_use]
    pub fn is_srv(&self) -> bool {
        matches!(self, Self::Srv(..))
    }

    #[must_use]
    pub fn is_cli(&self) -> bool {
        matches!(self, Self::Cli(..))
    }
}

impl From<NodeCliCtx> for NodeSocketCtx {
    fn from(v: NodeCliCtx) -> Self {
        Self::Cli(v)
    }
}

impl From<NodeSrvCtx> for NodeSocketCtx {
    fn from(v: NodeSrvCtx) -> Self {
        Self::Srv(v)
    }
}

impl From<NodeSubCtx> for NodeSocketCtx {
    fn from(v: NodeSubCtx) -> Self {
        Self::Sub(v)
    }
}

impl From<NodePubCtx> for NodeSocketCtx {
    fn from(v: NodePubCtx) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodePubCtx {
    pub key: KeyOwned,
    pub config: NodePubCfg,
    pub link_to: Option<PubSubLinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSubCtx {
    pub key: KeyOwned,
    pub config: NodeSubCfg,
    pub link_to: Option<PubSubLinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSrvCtx {
    pub key: KeyOwned,
    pub config: NodeSrvCfg,
    pub link_to: Option<ServiceLinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeCliCtx {
    pub key: KeyOwned,
    pub config: NodeCliCfg,
    pub link_to: Option<ServiceLinkShared>,
}
