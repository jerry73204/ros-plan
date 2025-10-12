use crate::context::{
    link::{PubSubLinkOwned, PubSubLinkShared, ServiceLinkOwned, ServiceLinkShared},
    node::{NodeOwned, NodeShared},
};

#[derive(Debug)]
pub enum EntityOwned {
    Node(NodeOwned),
    PubSubLink(PubSubLinkOwned),
    ServiceLink(ServiceLinkOwned),
}

impl From<ServiceLinkOwned> for EntityOwned {
    fn from(v: ServiceLinkOwned) -> Self {
        Self::ServiceLink(v)
    }
}

impl From<PubSubLinkOwned> for EntityOwned {
    fn from(v: PubSubLinkOwned) -> Self {
        Self::PubSubLink(v)
    }
}

impl From<NodeOwned> for EntityOwned {
    fn from(v: NodeOwned) -> Self {
        Self::Node(v)
    }
}

#[derive(Debug, Clone)]
pub enum EntityShared {
    Node(NodeShared),
    PubSubLink(PubSubLinkShared),
    ServiceLink(ServiceLinkShared),
}

impl From<ServiceLinkShared> for EntityShared {
    fn from(v: ServiceLinkShared) -> Self {
        Self::ServiceLink(v)
    }
}

impl From<PubSubLinkShared> for EntityShared {
    fn from(v: PubSubLinkShared) -> Self {
        Self::PubSubLink(v)
    }
}

impl From<NodeShared> for EntityShared {
    fn from(v: NodeShared) -> Self {
        Self::Node(v)
    }
}
