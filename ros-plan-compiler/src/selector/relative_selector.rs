use crate::{
    context::{
        link::LinkShared, node::NodeShared, node_socket::NodeSocketShared,
        plan_socket::PlanSocketShared,
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
                subscope.with_read(|guard| guard.node_map().get(node_ident).cloned())
            }
            None => self.scope.node_map().get(node_ident).cloned(),
        }
    }

    pub fn find_link(&self, key: &Key) -> Option<LinkShared> {
        let (scope_key, link_ident) = key.split_parent();
        let link_ident = link_ident?;

        match scope_key {
            Some(scope_key) => {
                let subscope = self.find_subscope(scope_key)?;
                subscope.with_read(|guard| guard.link_map().get(link_ident).cloned())
            }
            None => self.scope.link_map().get(link_ident).cloned(),
        }
    }

    pub fn find_node_socket(&self, key: &Key) -> Option<NodeSocketShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let node = self.find_node(node_key)?;
        node.with_read(|guard| guard.socket.get(socket_ident).cloned())
    }

    pub fn find_plan_socket(&self, key: &Key) -> Option<PlanSocketShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let subscope = self.find_subscope(scope_key)?;
        let include = subscope.as_include()?;
        include.with_read(|guard| guard.socket_map.get(socket_ident).cloned())
    }

    pub fn find_plan_or_node_socket(&self, socket_key: &Key) -> Option<PlanOrNodeSocket> {
        let output: PlanOrNodeSocket = {
            if let Some(socket) = self.find_node_socket(socket_key) {
                socket.into()
            } else {
                let socket = self.find_plan_socket(socket_key)?;
                socket.into()
            }
        };
        Some(output)
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

#[derive(Debug, Clone)]
pub enum PlanOrNodeSocket {
    Node(NodeSocketShared),
    Plan(PlanSocketShared),
}

impl PlanOrNodeSocket {
    pub fn extract_pub_src(&self) -> Option<Vec<NodeSocketShared>> {
        Some(match self {
            PlanOrNodeSocket::Node(socket) => {
                if !socket.with_read(|guard| guard.is_pub()) {
                    return None;
                }
                vec![socket.clone()]
            }
            PlanOrNodeSocket::Plan(socket) => socket.with_read(|guard| {
                let pub_ = guard.as_pub()?;
                let src = pub_.src.clone().unwrap();
                Some(src)
            })?,
        })
    }

    pub fn extract_sub_dst(&self) -> Option<Vec<NodeSocketShared>> {
        Some(match self {
            PlanOrNodeSocket::Node(socket) => {
                if !socket.with_read(|guard| guard.is_sub()) {
                    return None;
                }
                vec![socket.clone()]
            }
            PlanOrNodeSocket::Plan(socket) => socket.with_read(|guard| {
                let sub = guard.as_sub()?;
                let src = sub.dst.clone().unwrap();
                Some(src)
            })?,
        })
    }

    pub fn extract_srv_listen(&self) -> Option<NodeSocketShared> {
        Some(match self {
            PlanOrNodeSocket::Node(socket) => {
                if !socket.with_read(|guard| guard.is_srv()) {
                    return None;
                }
                socket.clone()
            }
            PlanOrNodeSocket::Plan(socket) => socket.with_read(|guard| {
                let srv = guard.as_srv()?;
                let listen = srv.listen.clone().unwrap();
                Some(listen)
            })?,
        })
    }

    pub fn extract_cli_connect(&self) -> Option<Vec<NodeSocketShared>> {
        Some(match self {
            PlanOrNodeSocket::Node(socket) => {
                if !socket.with_read(|guard| guard.is_cli()) {
                    return None;
                }
                vec![socket.clone()]
            }
            PlanOrNodeSocket::Plan(socket) => socket.with_read(|guard| {
                let cli = guard.as_cli()?;
                let connect = cli.connect.clone().unwrap();
                Some(connect)
            })?,
        })
    }
}

impl From<NodeSocketShared> for PlanOrNodeSocket {
    fn from(v: NodeSocketShared) -> Self {
        Self::Node(v)
    }
}

impl From<PlanSocketShared> for PlanOrNodeSocket {
    fn from(v: PlanSocketShared) -> Self {
        Self::Plan(v)
    }
}
