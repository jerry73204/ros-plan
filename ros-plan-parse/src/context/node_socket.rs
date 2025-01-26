use crate::utils::shared_table::{Owned, Shared};
use ros_plan_format::{
    key::{Key, KeyOwned},
    node_socket::{NodeClientCfg, NodePublicationCfg, NodeServerCfg, NodeSubscriptionCfg},
};
use serde::{Deserialize, Serialize};

use super::link::LinkShared;

pub type NodeSocketOwned = Owned<NodeSocketContext>;
pub type NodeSocketShared = Shared<NodeSocketContext>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeSocketContext {
    #[serde(rename = "pub")]
    Publication(NodePublicationContext),
    #[serde(rename = "sub")]
    Subscription(NodeSubscriptionContext),
    #[serde(rename = "srv")]
    Server(NodeServerContext),
    #[serde(rename = "cli")]
    Client(NodeClientContext),
}

impl NodeSocketContext {
    pub fn key(&self) -> &Key {
        match self {
            NodeSocketContext::Publication(socket) => &socket.key,
            NodeSocketContext::Subscription(socket) => &socket.key,
            NodeSocketContext::Server(socket) => &socket.key,
            NodeSocketContext::Client(socket) => &socket.key,
        }
    }

    pub fn as_publication(&self) -> Option<&NodePublicationContext> {
        if let Self::Publication(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_publication_mut(&mut self) -> Option<&mut NodePublicationContext> {
        if let Self::Publication(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_subscription(&self) -> Option<&NodeSubscriptionContext> {
        if let Self::Subscription(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_subscription_mut(&mut self) -> Option<&mut NodeSubscriptionContext> {
        if let Self::Subscription(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_server(&self) -> Option<&NodeServerContext> {
        if let Self::Server(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_server_mut(&mut self) -> Option<&mut NodeServerContext> {
        if let Self::Server(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_client(&self) -> Option<&NodeClientContext> {
        if let Self::Client(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_client_mut(&mut self) -> Option<&mut NodeClientContext> {
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

impl From<NodeClientContext> for NodeSocketContext {
    fn from(v: NodeClientContext) -> Self {
        Self::Client(v)
    }
}

impl From<NodeServerContext> for NodeSocketContext {
    fn from(v: NodeServerContext) -> Self {
        Self::Server(v)
    }
}

impl From<NodeSubscriptionContext> for NodeSocketContext {
    fn from(v: NodeSubscriptionContext) -> Self {
        Self::Subscription(v)
    }
}

impl From<NodePublicationContext> for NodeSocketContext {
    fn from(v: NodePublicationContext) -> Self {
        Self::Publication(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodePublicationContext {
    pub key: KeyOwned,
    pub config: NodePublicationCfg,
    pub link_to: Option<LinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeSubscriptionContext {
    pub key: KeyOwned,
    pub config: NodeSubscriptionCfg,
    pub link_to: Option<LinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeServerContext {
    pub key: KeyOwned,
    pub config: NodeServerCfg,
    pub link_to: Option<LinkShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeClientContext {
    pub key: KeyOwned,
    pub config: NodeClientCfg,
    pub link_to: Option<LinkShared>,
}
