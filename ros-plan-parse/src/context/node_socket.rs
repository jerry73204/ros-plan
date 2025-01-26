use crate::utils::shared_table::{Owned, Shared};
use ros_plan_format::{
    key::{Key, KeyOwned},
    node_socket::{NodeCliCfg, NodePubCfg, NodeSrvCfg, NodeSubCfg},
};
use serde::{Deserialize, Serialize};

use super::link::LinkShared;

pub type NodeSocketOwned = Owned<NodeSocketCtx>;
pub type NodeSocketShared = Shared<NodeSocketCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeSocketCtx {
    #[serde(rename = "pub")]
    Publication(NodePubCtx),
    #[serde(rename = "sub")]
    Subscription(NodeSubCtx),
    #[serde(rename = "srv")]
    Server(NodeSrvCtx),
    #[serde(rename = "cli")]
    Client(NodeCliCtx),
}

impl NodeSocketCtx {
    pub fn key(&self) -> &Key {
        match self {
            NodeSocketCtx::Publication(socket) => &socket.key,
            NodeSocketCtx::Subscription(socket) => &socket.key,
            NodeSocketCtx::Server(socket) => &socket.key,
            NodeSocketCtx::Client(socket) => &socket.key,
        }
    }

    pub fn as_publication(&self) -> Option<&NodePubCtx> {
        if let Self::Publication(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_publication_mut(&mut self) -> Option<&mut NodePubCtx> {
        if let Self::Publication(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_subscription(&self) -> Option<&NodeSubCtx> {
        if let Self::Subscription(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_subscription_mut(&mut self) -> Option<&mut NodeSubCtx> {
        if let Self::Subscription(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_server(&self) -> Option<&NodeSrvCtx> {
        if let Self::Server(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_server_mut(&mut self) -> Option<&mut NodeSrvCtx> {
        if let Self::Server(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_client(&self) -> Option<&NodeCliCtx> {
        if let Self::Client(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_client_mut(&mut self) -> Option<&mut NodeCliCtx> {
        if let Self::Client(v) = self {
            Some(v)
        } else {
            None
        }
    }

    /// Returns `true` if the node socket context is [`Publication`].
    ///
    /// [`Publication`]: NodeSocketContext::Publication
    #[must_use]
    pub fn is_publication(&self) -> bool {
        matches!(self, Self::Publication(..))
    }

    /// Returns `true` if the node socket context is [`Subscription`].
    ///
    /// [`Subscription`]: NodeSocketContext::Subscription
    #[must_use]
    pub fn is_subscription(&self) -> bool {
        matches!(self, Self::Subscription(..))
    }

    /// Returns `true` if the node socket context is [`Server`].
    ///
    /// [`Server`]: NodeSocketContext::Server
    #[must_use]
    pub fn is_server(&self) -> bool {
        matches!(self, Self::Server(..))
    }

    /// Returns `true` if the node socket context is [`Client`].
    ///
    /// [`Client`]: NodeSocketContext::Client
    #[must_use]
    pub fn is_client(&self) -> bool {
        matches!(self, Self::Client(..))
    }
}

impl From<NodeCliCtx> for NodeSocketCtx {
    fn from(v: NodeCliCtx) -> Self {
        Self::Client(v)
    }
}

impl From<NodeSrvCtx> for NodeSocketCtx {
    fn from(v: NodeSrvCtx) -> Self {
        Self::Server(v)
    }
}

impl From<NodeSubCtx> for NodeSocketCtx {
    fn from(v: NodeSubCtx) -> Self {
        Self::Subscription(v)
    }
}

impl From<NodePubCtx> for NodeSocketCtx {
    fn from(v: NodePubCtx) -> Self {
        Self::Publication(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodePubCtx {
    pub key: KeyOwned,
    pub config: NodePubCfg,
    pub link_to: Option<LinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSubCtx {
    pub key: KeyOwned,
    pub config: NodeSubCfg,
    pub link_to: Option<LinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSrvCtx {
    pub key: KeyOwned,
    pub config: NodeSrvCfg,
    pub link_to: Option<LinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeCliCtx {
    pub key: KeyOwned,
    pub config: NodeCliCfg,
    pub link_to: Option<LinkShared>,
}
