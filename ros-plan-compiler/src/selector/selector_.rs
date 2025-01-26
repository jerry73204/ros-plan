use crate::{
    context::{
        link::LinkShared, node::NodeShared, node_socket::NodeSocketShared,
        plan_socket::PlanSocketShared,
    },
    scope::ScopeShared,
    Program,
};
use ros_plan_format::key::{Key, KeyKind};

use super::{AbsoluteSelector, PlanOrNodeSocket, RelativeSelector};

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

    pub fn find_link(&self, key: &Key) -> Option<LinkShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_link(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_link(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node_socket(&self, key: &Key) -> Option<NodeSocketShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_node_socket(key)),
            KeyKind::Absolute { .. } => AbsoluteSelector::new(self.program).find_node_socket(key),
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_socket(&self, key: &Key) -> Option<PlanSocketShared> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_socket(key)),
            KeyKind::Absolute { .. } => None,
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_plan_or_node_socket(&self, key: &Key) -> Option<PlanOrNodeSocket> {
        match key.to_kind() {
            KeyKind::Relative { key } => self
                .scope
                .with_read(|guard| RelativeSelector::new(&guard).find_plan_or_node_socket(key)),
            KeyKind::Absolute { .. } => {
                // Only search for node socket if the key is absolute
                let node_socket = AbsoluteSelector::new(self.program).find_node_socket(key)?;
                Some(node_socket.into())
            }
            KeyKind::Private { .. } => None,
        }
    }

    pub fn find_node_pub(&self, plan_or_node_socket_key: &Key) -> Option<Vec<NodeSocketShared>> {
        self.find_plan_or_node_socket(plan_or_node_socket_key)?
            .extract_pub_src()
    }

    pub fn find_node_sub(&self, plan_or_node_socket_key: &Key) -> Option<Vec<NodeSocketShared>> {
        self.find_plan_or_node_socket(plan_or_node_socket_key)?
            .extract_sub_dst()
    }

    pub fn find_node_srv(&self, plan_or_node_socket_key: &Key) -> Option<NodeSocketShared> {
        self.find_plan_or_node_socket(plan_or_node_socket_key)?
            .extract_srv_listen()
    }

    pub fn find_node_cli(&self, plan_or_node_socket_key: &Key) -> Option<Vec<NodeSocketShared>> {
        self.find_plan_or_node_socket(plan_or_node_socket_key)?
            .extract_cli_connect()
    }
}
