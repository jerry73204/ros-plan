use ros_plan_format::{
    key::{Key, KeyOwned},
    plan_socket::{PlanClientCfg, PlanPublicationCfg, PlanServerCfg, PlanSubscriptionCfg},
};
use serde::{Deserialize, Serialize};

use crate::scope::NodeSocketShared;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PlanSocketContext {
    #[serde(rename = "pub")]
    Publication(PlanPublicationContext),
    #[serde(rename = "sub")]
    Subscription(PlanSubscriptionContext),
    #[serde(rename = "srv")]
    Server(PlanServerContext),
    #[serde(rename = "cli")]
    Client(PlanClientContext),
}

impl PlanSocketContext {
    pub fn key(&self) -> &Key {
        match self {
            PlanSocketContext::Publication(socket) => &socket.key,
            PlanSocketContext::Subscription(socket) => &socket.key,
            PlanSocketContext::Server(socket) => &socket.key,
            PlanSocketContext::Client(socket) => &socket.key,
        }
    }

    pub fn as_publication(&self) -> Option<&PlanPublicationContext> {
        if let Self::Publication(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_subscription(&self) -> Option<&PlanSubscriptionContext> {
        if let Self::Subscription(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_server(&self) -> Option<&PlanServerContext> {
        if let Self::Server(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_client(&self) -> Option<&PlanClientContext> {
        if let Self::Client(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PlanClientContext> for PlanSocketContext {
    fn from(v: PlanClientContext) -> Self {
        Self::Client(v)
    }
}

impl From<PlanServerContext> for PlanSocketContext {
    fn from(v: PlanServerContext) -> Self {
        Self::Server(v)
    }
}

impl From<PlanSubscriptionContext> for PlanSocketContext {
    fn from(v: PlanSubscriptionContext) -> Self {
        Self::Subscription(v)
    }
}

impl From<PlanPublicationContext> for PlanSocketContext {
    fn from(v: PlanPublicationContext) -> Self {
        Self::Publication(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanPublicationContext {
    pub key: KeyOwned,
    pub config: PlanPublicationCfg,
    pub src: Option<Vec<NodeSocketShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanSubscriptionContext {
    pub key: KeyOwned,
    pub config: PlanSubscriptionCfg,
    pub dst: Option<Vec<NodeSocketShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanServerContext {
    pub key: KeyOwned,
    pub config: PlanServerCfg,
    pub listen: Option<NodeSocketShared>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanClientContext {
    pub key: KeyOwned,
    pub config: PlanClientCfg,
    pub connect: Option<Vec<NodeSocketShared>>,
}
