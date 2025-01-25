use super::{LinkOwned, LinkShared, NodeOwned, NodeShared};

#[derive(Debug)]
pub enum EntityOwned {
    Node(NodeOwned),
    Link(LinkOwned),
}

impl From<LinkOwned> for EntityOwned {
    fn from(v: LinkOwned) -> Self {
        Self::Link(v)
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
    Link(LinkShared),
}

impl From<LinkShared> for EntityShared {
    fn from(v: LinkShared) -> Self {
        Self::Link(v)
    }
}

impl From<NodeShared> for EntityShared {
    fn from(v: NodeShared) -> Self {
        Self::Node(v)
    }
}
