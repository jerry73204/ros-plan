use super::{ScopeRef, ScopeRefExt, ScopeShared};
use crate::context::{
    link::LinkShared, node::NodeShared, node_socket::NodeSocketShared,
    plan_socket::PlanSocketShared,
};
use ros_plan_format::key::Key;

pub struct LocalSelector<'a, S>
where
    S: ScopeRef,
{
    scope: &'a S,
}

impl<'a, S> LocalSelector<'a, S>
where
    S: ScopeRef,
{
    pub fn new(scope: &'a S) -> Self {
        Self { scope }
    }

    pub fn find_subscope(&self, key: &Key) -> Option<ScopeShared> {
        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.scope.get_subscope(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            let ScopeShared::Group(shared) = curr else {
                return None;
            };
            let owned = shared.upgrade().unwrap();
            let guard = owned.read();

            let (child, next_suffix) = guard.get_subscope(curr_suffix)?;
            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }

    pub fn find_node(&self, key: &Key) -> Option<NodeShared> {
        let (scope_key, node_ident) = key.split_parent();
        let node_ident = node_ident?;

        match scope_key {
            Some(scope_key) => {
                let shared = self.find_subscope(scope_key)?;
                let owned = shared.upgrade().unwrap();
                let guard = owned.read();
                guard.node_map().get(node_ident).cloned()
            }
            None => self.scope.node_map().get(node_ident).cloned(),
        }
    }

    pub fn find_link(&self, key: &Key) -> Option<LinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let link_ident = link_ident?;

        match scope_key {
            Some(scope_key) => {
                let shared = self.find_subscope(scope_key)?;
                let owned = shared.upgrade().unwrap();
                let guard = owned.read();
                guard.link_map().get(link_ident).cloned()
            }
            None => self.scope.link_map().get(link_ident).cloned(),
        }
    }

    pub fn find_node_socket(&self, key: &Key) -> Option<NodeSocketShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let shared = self.find_node(node_key)?;
        let owned = shared.upgrade().unwrap();
        let guard = owned.read();
        guard.socket.get(socket_ident).cloned()
    }

    pub fn find_plan_socket(&self, key: &Key) -> Option<PlanSocketShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let shared = self.find_subscope(scope_key)?;
        let owned = shared.upgrade().unwrap();
        let guard = owned.read();
        let include = guard.as_include()?;
        include.socket_map.get(socket_ident).cloned()
    }
}

pub struct GlobalSelector<'a, S>
where
    S: ScopeRef,
{
    scope: &'a S,
}

impl<'a, S> GlobalSelector<'a, S>
where
    S: ScopeRef,
{
    pub fn new(scope: &'a S) -> Self {
        Self { scope }
    }

    pub fn find_subscope(&self, key: &Key) -> Option<ScopeShared> {
        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.scope.get_subscope(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            let (child, next_suffix) = match curr {
                ScopeShared::Group(shared) => {
                    let owned = shared.upgrade().unwrap();
                    let guard = owned.read();
                    guard.get_subscope(curr_suffix)?
                }
                ScopeShared::Include(shared) => {
                    let owned = shared.upgrade().unwrap();
                    let guard = owned.read();
                    guard.get_subscope(curr_suffix)?
                }
            };

            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }

    pub fn find_node(&self, key: &Key) -> Option<NodeShared> {
        let (scope_key, node_ident) = key.split_parent();
        let node_ident = node_ident?;

        match scope_key {
            Some(scope_key) => {
                let shared = self.find_subscope(scope_key)?;
                let owned = shared.upgrade().unwrap();
                let guard = owned.read();
                guard.node_map().get(node_ident).cloned()
            }
            None => self.scope.node_map().get(node_ident).cloned(),
        }
    }

    pub fn find_link(&self, key: &Key) -> Option<LinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let link_ident = link_ident?;

        match scope_key {
            Some(scope_key) => {
                let shared = self.find_subscope(scope_key)?;
                let owned = shared.upgrade().unwrap();
                let guard = owned.read();
                guard.link_map().get(link_ident).cloned()
            }
            None => self.scope.link_map().get(link_ident).cloned(),
        }
    }

    pub fn find_node_socket(&self, key: &Key) -> Option<NodeSocketShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let shared = self.find_node(node_key)?;
        let owned = shared.upgrade().unwrap();
        let guard = owned.read();
        guard.socket.get(socket_ident).cloned()
    }
}
