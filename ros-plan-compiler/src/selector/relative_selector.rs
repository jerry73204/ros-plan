use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
        node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared},
        plan_socket::{PlanCliShared, PlanPubShared, PlanSrvShared, PlanSubShared},
    },
    scope::{ScopeRef, ScopeRefExt, ScopeShared},
};
use ros_plan_format::key::{Key, KeyKind};

pub struct RelativeSelector<'a, S>
where
    S: ScopeRef,
{
    scope: &'a S,
}

impl<'a, S> RelativeSelector<'a, S>
where
    S: ScopeRef,
{
    pub fn new(scope: &'a S) -> Self {
        Self { scope }
    }

    pub fn find_subscope(&self, key: &Key) -> Option<ScopeShared> {
        // Check whether the input key is relative.
        let KeyKind::Relative { key } = key.to_kind() else {
            return None;
        };

        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.scope.get_subscope(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            let ScopeShared::Group(group) = curr else {
                return None;
            };

            let (child, next_suffix) = group.with_read(|guard| guard.get_subscope(curr_suffix))?;
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
                let subscope = self.find_subscope(scope_key)?;
                subscope.with_read(|guard| guard.node().get(node_ident).cloned())
            }
            None => self.scope.node().get(node_ident).cloned(),
        }
    }

    pub fn find_pubsub_link(&self, key: &Key) -> Option<PubSubLinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let link_ident = link_ident?;

        match scope_key {
            Some(scope_key) => {
                let subscope = self.find_subscope(scope_key)?;
                subscope.with_read(|guard| guard.pubsub_link().get(link_ident).cloned())
            }
            None => self.scope.pubsub_link().get(link_ident).cloned(),
        }
    }

    pub fn find_service_link(&self, key: &Key) -> Option<ServiceLinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let link_ident = link_ident?;

        match scope_key {
            Some(scope_key) => {
                let subscope = self.find_subscope(scope_key)?;
                subscope.with_read(|guard| guard.service_link().get(link_ident).cloned())
            }
            None => self.scope.service_link().get(link_ident).cloned(),
        }
    }

    pub fn find_node_pub(&self, key: &Key) -> Option<NodePubShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let node = self.find_node(node_key)?;
        node.with_read(|guard| guard.pub_.get(socket_ident).cloned())
    }

    pub fn find_node_sub(&self, key: &Key) -> Option<NodeSubShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let node = self.find_node(node_key)?;
        node.with_read(|guard| guard.sub.get(socket_ident).cloned())
    }

    pub fn find_node_srv(&self, key: &Key) -> Option<NodeSrvShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let node = self.find_node(node_key)?;
        node.with_read(|guard| guard.srv.get(socket_ident).cloned())
    }

    pub fn find_node_cli(&self, key: &Key) -> Option<NodeCliShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let node = self.find_node(node_key)?;
        node.with_read(|guard| guard.cli.get(socket_ident).cloned())
    }

    pub fn find_plan_pub(&self, key: &Key) -> Option<PlanPubShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let subscope = self.find_subscope(scope_key)?;
        let include = subscope.as_include()?;
        include.with_read(|guard| guard.pub_.get(socket_ident).cloned())
    }

    pub fn find_plan_sub(&self, key: &Key) -> Option<PlanSubShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let subscope = self.find_subscope(scope_key)?;
        let include = subscope.as_include()?;
        include.with_read(|guard| guard.sub.get(socket_ident).cloned())
    }

    pub fn find_plan_srv(&self, key: &Key) -> Option<PlanSrvShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let subscope = self.find_subscope(scope_key)?;
        let include = subscope.as_include()?;
        include.with_read(|guard| guard.srv.get(socket_ident).cloned())
    }

    pub fn find_plan_cli(&self, key: &Key) -> Option<PlanCliShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let subscope = self.find_subscope(scope_key)?;
        let include = subscope.as_include()?;
        include.with_read(|guard| guard.cli.get(socket_ident).cloned())
    }

    pub fn find_plan_or_node_pub(&self, socket_key: &Key) -> Option<PlanOrNodePub> {
        let output: PlanOrNodePub = {
            if let Some(socket) = self.find_node_pub(socket_key) {
                socket.into()
            } else {
                let socket = self.find_plan_pub(socket_key)?;
                socket.into()
            }
        };
        Some(output)
    }

    pub fn find_plan_or_node_sub(&self, socket_key: &Key) -> Option<PlanOrNodeSub> {
        let output: PlanOrNodeSub = {
            if let Some(socket) = self.find_node_sub(socket_key) {
                socket.into()
            } else {
                let socket = self.find_plan_sub(socket_key)?;
                socket.into()
            }
        };
        Some(output)
    }

    pub fn find_plan_or_node_srv(&self, socket_key: &Key) -> Option<PlanOrNodeSrv> {
        let output: PlanOrNodeSrv = {
            if let Some(socket) = self.find_node_srv(socket_key) {
                socket.into()
            } else {
                let socket = self.find_plan_srv(socket_key)?;
                socket.into()
            }
        };
        Some(output)
    }

    pub fn find_plan_or_node_cli(&self, socket_key: &Key) -> Option<PlanOrNodeCli> {
        let output: PlanOrNodeCli = {
            if let Some(socket) = self.find_node_cli(socket_key) {
                socket.into()
            } else {
                let socket = self.find_plan_cli(socket_key)?;
                socket.into()
            }
        };
        Some(output)
    }
}

#[derive(Debug, Clone)]
pub enum PlanOrNodePub {
    Node(NodePubShared),
    Plan(PlanPubShared),
}

impl PlanOrNodePub {
    pub fn to_node_pub(&self) -> Option<Vec<NodePubShared>> {
        Some(match self {
            PlanOrNodePub::Node(node_pub) => vec![node_pub.clone()],
            PlanOrNodePub::Plan(plan_pub) => plan_pub.with_read(|plan_pub| plan_pub.src.clone())?,
        })
    }
}

impl From<PlanPubShared> for PlanOrNodePub {
    fn from(v: PlanPubShared) -> Self {
        Self::Plan(v)
    }
}

impl From<NodePubShared> for PlanOrNodePub {
    fn from(v: NodePubShared) -> Self {
        Self::Node(v)
    }
}

#[derive(Debug, Clone)]
pub enum PlanOrNodeSub {
    Node(NodeSubShared),
    Plan(PlanSubShared),
}

impl PlanOrNodeSub {
    pub fn to_node_sub(&self) -> Option<Vec<NodeSubShared>> {
        Some(match self {
            PlanOrNodeSub::Node(node_sub) => vec![node_sub.clone()],
            PlanOrNodeSub::Plan(plan_pub) => plan_pub.with_read(|plan_pub| plan_pub.dst.clone())?,
        })
    }
}

impl From<PlanSubShared> for PlanOrNodeSub {
    fn from(v: PlanSubShared) -> Self {
        Self::Plan(v)
    }
}

impl From<NodeSubShared> for PlanOrNodeSub {
    fn from(v: NodeSubShared) -> Self {
        Self::Node(v)
    }
}

#[derive(Debug, Clone)]
pub enum PlanOrNodeSrv {
    Node(NodeSrvShared),
    Plan(PlanSrvShared),
}

impl PlanOrNodeSrv {
    pub fn to_node_srv(&self) -> Option<NodeSrvShared> {
        Some(match self {
            PlanOrNodeSrv::Node(node_srv) => node_srv.clone(),
            PlanOrNodeSrv::Plan(plan_pub) => {
                plan_pub.with_read(|plan_pub| plan_pub.listen.clone())?
            }
        })
    }
}

impl From<PlanSrvShared> for PlanOrNodeSrv {
    fn from(v: PlanSrvShared) -> Self {
        Self::Plan(v)
    }
}

impl From<NodeSrvShared> for PlanOrNodeSrv {
    fn from(v: NodeSrvShared) -> Self {
        Self::Node(v)
    }
}

#[derive(Debug, Clone)]
pub enum PlanOrNodeCli {
    Node(NodeCliShared),
    Plan(PlanCliShared),
}

impl PlanOrNodeCli {
    pub fn to_node_cli(&self) -> Option<Vec<NodeCliShared>> {
        Some(match self {
            PlanOrNodeCli::Node(node_cli) => vec![node_cli.clone()],
            PlanOrNodeCli::Plan(plan_pub) => {
                plan_pub.with_read(|plan_pub| plan_pub.connect.clone())?
            }
        })
    }
}

impl From<PlanCliShared> for PlanOrNodeCli {
    fn from(v: PlanCliShared) -> Self {
        Self::Plan(v)
    }
}

impl From<NodeCliShared> for PlanOrNodeCli {
    fn from(v: NodeCliShared) -> Self {
        Self::Node(v)
    }
}
