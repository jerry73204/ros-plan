use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
        node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared},
        plan_socket::{PlanCliShared, PlanPubShared, PlanSrvShared, PlanSubShared},
    },
    scope::ScopeShared,
    Program,
};
use ros_plan_format::key::{Key, KeyKind};

use super::{
    relative_selector::{PlanOrNodeCli, PlanOrNodePub, PlanOrNodeSrv, PlanOrNodeSub},
    AbsoluteSelector, RelativeSelector,
};

pub struct Selector<'a, 'b> {
    program: &'a Program,
    scope: &'b ScopeShared,
}

impl<'a, 'b> Selector<'a, 'b> {
    pub fn new(program: &'a Program, scope: &'b ScopeShared) -> Self {
        Self { program, scope }
    }

    pub fn find_subscope(&self, key: &Key) -> Option<ScopeShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_subscope(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_subscope(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node(&self, key: &Key) -> Option<NodeShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_node(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_node(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_pubsub_link(&self, key: &Key) -> Option<PubSubLinkShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_pubsub_link(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_pubsub_link(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_service_link(&self, key: &Key) -> Option<ServiceLinkShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_service_link(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_service_link(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node_pub(&self, key: &Key) -> Option<NodePubShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_node_pub(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_node_pub(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node_sub(&self, key: &Key) -> Option<NodeSubShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_node_sub(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_node_sub(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node_srv(&self, key: &Key) -> Option<NodeSrvShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_node_srv(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_node_srv(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node_cli(&self, key: &Key) -> Option<NodeCliShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_node_cli(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_node_cli(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_pub(&self, key: &Key) -> Option<PlanPubShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_pub(key)),
            KeyKind::Absolute { .. } => None,
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_sub(&self, key: &Key) -> Option<PlanSubShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_sub(key)),
            KeyKind::Absolute { .. } => None,
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_srv(&self, key: &Key) -> Option<PlanSrvShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_srv(key)),
            KeyKind::Absolute { .. } => None,
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_cli(&self, key: &Key) -> Option<PlanCliShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_cli(key)),
            KeyKind::Absolute { .. } => None,
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_or_node_pub(&self, key: &Key) -> Option<PlanOrNodePub> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_or_node_pub(key)),
            KeyKind::Absolute { .. } => {
                // Only search for node socket if the key is absolute
                let node_socket = AbsoluteSelector::new(self.program).find_node_pub(key)?;
                Some(node_socket.into())
            }
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_or_node_sub(&self, key: &Key) -> Option<PlanOrNodeSub> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_or_node_sub(key)),
            KeyKind::Absolute { .. } => {
                // Only search for node socket if the key is absolute
                let node_socket = AbsoluteSelector::new(self.program).find_node_sub(key)?;
                Some(node_socket.into())
            }
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_or_node_srv(&self, key: &Key) -> Option<PlanOrNodeSrv> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_or_node_srv(key)),
            KeyKind::Absolute { .. } => {
                // Only search for node socket if the key is absolute
                let node_socket = AbsoluteSelector::new(self.program).find_node_srv(key)?;
                Some(node_socket.into())
            }
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_or_node_cli(&self, key: &Key) -> Option<PlanOrNodeCli> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_or_node_cli(key)),
            KeyKind::Absolute { .. } => {
                // Only search for node socket if the key is absolute
                let node_socket = AbsoluteSelector::new(self.program).find_node_cli(key)?;
                Some(node_socket.into())
            }
            KeyKind::Private { .. } => None,
        }
    }
}
