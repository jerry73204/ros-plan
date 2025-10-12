use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
        node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared},
    },
    scope::{ScopeRef, ScopeRefExt, ScopeShared},
    Program,
};
use ros_plan_format::key::{Key, KeyKind};

pub struct AbsoluteSelector<'a> {
    program: &'a Program,
}

impl<'a> AbsoluteSelector<'a> {
    pub fn new(program: &'a Program) -> Self {
        Self { program }
    }

    pub fn find_subscope(&self, key: &Key) -> Option<ScopeShared> {
        // Set the starting scope according to the key type.
        let (mut curr, mut suffix) = match key.to_kind() {
            KeyKind::Absolute { suffix, .. } => {
                let root: ScopeShared = self.program.root_scope().into();
                (root, suffix)
            }
            _ => return None,
        };

        while let Some(curr_suffix) = suffix.take() {
            let (child, next_suffix) = match curr {
                ScopeShared::Group(shared) => {
                    shared.with_read(|guard| guard.get_subscope(curr_suffix))?
                }
                ScopeShared::Include(shared) => {
                    shared.with_read(|guard| guard.get_subscope(curr_suffix))?
                }
            };

            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }

    pub fn find_node(&self, key: &Key) -> Option<NodeShared> {
        let (scope_key, node_ident) = key.split_parent();
        let scope_key = scope_key?;
        let node_ident = node_ident?;

        let subscope = self.find_subscope(scope_key)?;
        subscope.with_read(|guard| guard.node().get(node_ident).cloned())
    }

    pub fn find_pubsub_link(&self, key: &Key) -> Option<PubSubLinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let scope_key = scope_key?;
        let link_ident = link_ident?;

        let subscope = self.find_subscope(scope_key)?;
        subscope.with_read(|guard| guard.pubsub_link().get(link_ident).cloned())
    }

    pub fn find_service_link(&self, key: &Key) -> Option<ServiceLinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let scope_key = scope_key?;
        let link_ident = link_ident?;

        let subscope = self.find_subscope(scope_key)?;
        subscope.with_read(|guard| guard.service_link().get(link_ident).cloned())
    }

    pub fn find_node_pub(&self, key: &Key) -> Option<NodePubShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let scope = self.find_node(node_key)?;
        scope.with_read(|guard| guard.pub_.get(socket_ident).cloned())
    }

    pub fn find_node_sub(&self, key: &Key) -> Option<NodeSubShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let scope = self.find_node(node_key)?;
        scope.with_read(|guard| guard.sub.get(socket_ident).cloned())
    }

    pub fn find_node_srv(&self, key: &Key) -> Option<NodeSrvShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let scope = self.find_node(node_key)?;
        scope.with_read(|guard| guard.srv.get(socket_ident).cloned())
    }

    pub fn find_node_cli(&self, key: &Key) -> Option<NodeCliShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let scope = self.find_node(node_key)?;
        scope.with_read(|guard| guard.cli.get(socket_ident).cloned())
    }
}
