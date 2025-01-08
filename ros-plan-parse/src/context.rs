use crate::{
    error::Error,
    utils::{ArcRwLock, WeakRwLock},
};
use indexmap::IndexMap;
use itertools::Itertools;
use parking_lot::{RwLockReadGuard, RwLockWriteGuard};
use ros_plan_format::{
    eval::ValueOrEval,
    ident::{Ident, IdentOwned},
    key::{Key, KeyOwned},
    link::{Link, LinkIdent},
    node::{Node, NodeIdent},
    parameter::{ArgEntry, ParamName},
    qos::Qos,
    ros_type::RosTypeOwned,
    socket::{PubSocket, QuerySocket, ServerSocket, Socket, SocketIdent, SubSocket},
};
use std::{collections::HashMap, path::PathBuf};

pub type NodeArc = ArcRwLock<NodeContext>;
pub type NodeWeak = WeakRwLock<NodeContext>;
pub type LinkArc = ArcRwLock<LinkContext>;
pub type LinkWeak = WeakRwLock<LinkContext>;
pub type SocketArc = ArcRwLock<SocketContext>;
// pub type SocketWeak = WeakRwLock<SocketContext>;

#[derive(Debug, Clone)]
pub(crate) struct GlobalContextV1 {
    pub namespace: KeyOwned,
    pub root: TrieRef,
    pub node_map: HashMap<KeyOwned, NodeWeak>,
}

#[derive(Debug, Clone)]
pub(crate) struct GlobalContextV2 {
    pub namespace: KeyOwned,
    pub root: TrieRef,
    pub node_map: HashMap<KeyOwned, NodeWeak>,
    pub link_map: HashMap<KeyOwned, LinkWeak>,
}

#[derive(Debug, Clone)]
pub struct Trie {
    pub context: Option<TrieContext>,
    pub children: HashMap<IdentOwned, TrieRef>,
}

impl Default for Trie {
    fn default() -> Self {
        Self {
            children: HashMap::new(),
            context: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrieRef(ArcRwLock<Trie>);

impl TrieRef {
    pub fn get_child(&self, ident: &Ident) -> Option<TrieRef> {
        let guard = self.read();
        let child = guard.children.get(ident)?;
        Some(child.clone())
    }

    pub fn insert(&self, key: &Key, context: TrieContext) -> Result<TrieRef, Error> {
        use std::collections::hash_map::Entry as E;

        // Reject absolute keys.
        if key.is_absolute() {
            return Err(Error::AbsoluteKeyNotAllowed {
                key: key.to_owned(),
            });
        }

        let mut curr: TrieRef = self.clone();

        // Insert subsequent components.
        for (ith, ident) in key.components().enumerate() {
            let mut guard = curr.0.write();

            if ith != 0 && guard.context.is_some() {
                let old_key = key.components().take(ith).join("/");
                return Err(Error::ConflictingKeys {
                    old: old_key.parse().unwrap(),
                    new: key.to_owned(),
                });
            }

            let next = match guard.children.entry(ident.to_owned()) {
                E::Occupied(entry) => entry.get().clone(),
                E::Vacant(entry) => entry.insert(TrieRef::default()).clone(),
            };

            drop(guard);
            curr = next;
        }

        // Check if the destination node already has a context.
        {
            let mut guard = curr.0.write();
            if guard.context.is_some() || !guard.children.is_empty() {
                return Err(Error::ConflictingKeys {
                    old: key.to_owned(),
                    new: key.to_owned(),
                });
            }
            guard.context = Some(context);
        }
        Ok(curr)
    }

    pub(crate) fn read(&self) -> RwLockReadGuard<Trie> {
        self.0.read()
    }

    pub(crate) fn write(&self) -> RwLockWriteGuard<Trie> {
        self.0.write()
    }
}

impl Default for TrieRef {
    fn default() -> Self {
        Self(Trie::default().into())
    }
}

impl From<Trie> for TrieRef {
    fn from(inner: Trie) -> Self {
        Self(inner.into())
    }
}

#[derive(Debug, Clone)]
pub enum TrieContext {
    PlanV1(PlanContextV1),
    PlanV2(PlanContextV2),
    PlanV3(PlanContextV3),
    HerePlanV1(HerePlanContextV1),
    HerePlanV2(HerePlanContextV2),
}

impl From<HerePlanContextV2> for TrieContext {
    fn from(v: HerePlanContextV2) -> Self {
        Self::HerePlanV2(v)
    }
}

impl From<PlanContextV3> for TrieContext {
    fn from(v: PlanContextV3) -> Self {
        Self::PlanV3(v)
    }
}

impl From<HerePlanContextV1> for TrieContext {
    fn from(v: HerePlanContextV1) -> Self {
        Self::HerePlanV1(v)
    }
}

impl From<PlanContextV2> for TrieContext {
    fn from(v: PlanContextV2) -> Self {
        Self::PlanV2(v)
    }
}

impl From<PlanContextV1> for TrieContext {
    fn from(v: PlanContextV1) -> Self {
        Self::PlanV1(v)
    }
}

#[derive(Debug, Clone)]
pub struct PlanContextV1 {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgEntry>,
    pub socket_map: IndexMap<SocketIdent, Socket>,
    pub node_map: HashMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, Link>,
}

#[derive(Debug, Clone)]
pub struct PlanContextV2 {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgEntry>,
    pub socket_map: HashMap<SocketIdent, SocketArc>,
    pub node_map: HashMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, Link>,
}

#[derive(Debug, Clone)]
pub struct PlanContextV3 {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgEntry>,
    pub socket_map: HashMap<SocketIdent, SocketArc>,
    pub node_map: HashMap<NodeIdent, NodeArc>,
    pub link_map: HashMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone)]
pub struct HerePlanContextV1 {
    pub node_map: HashMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, Link>,
}

#[derive(Debug, Clone)]
pub struct HerePlanContextV2 {
    pub node_map: HashMap<NodeIdent, NodeArc>,
    pub link_map: HashMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone)]
pub struct NodeContext {
    pub config: Node,
}

#[derive(Debug, Clone)]
pub enum LinkContext {
    PubSub(PubSubLinkContext),
    Service(ServiceLinkContext),
}

impl From<ServiceLinkContext> for LinkContext {
    fn from(v: ServiceLinkContext) -> Self {
        Self::Service(v)
    }
}

impl From<PubSubLinkContext> for LinkContext {
    fn from(v: PubSubLinkContext) -> Self {
        Self::PubSub(v)
    }
}

#[derive(Debug, Clone)]
pub struct PubSubLinkContext {
    pub ty: RosTypeOwned,
    pub qos: Qos,
    pub src: Vec<NodeTopicUri>,
    pub dst: Vec<NodeTopicUri>,
}

#[derive(Debug, Clone)]
pub struct ServiceLinkContext {
    pub ty: RosTypeOwned,
    pub listen: NodeTopicUri,
    pub connect: Vec<NodeTopicUri>,
}

#[derive(Debug, Clone)]
pub enum SocketContext {
    Pub(PubSocketContext),
    Sub(SubSocketContext),
    Srv(ServerSocketContext),
    Qry(QuerySocketContext),
}

impl From<QuerySocketContext> for SocketContext {
    fn from(v: QuerySocketContext) -> Self {
        Self::Qry(v)
    }
}

impl From<ServerSocketContext> for SocketContext {
    fn from(v: ServerSocketContext) -> Self {
        Self::Srv(v)
    }
}

impl From<SubSocketContext> for SocketContext {
    fn from(v: SubSocketContext) -> Self {
        Self::Sub(v)
    }
}

impl From<PubSocketContext> for SocketContext {
    fn from(v: PubSocketContext) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone)]
pub struct PubSocketContext {
    pub config: PubSocket,
    pub src: Vec<NodeTopicUri>,
}

#[derive(Debug, Clone)]
pub struct SubSocketContext {
    pub config: SubSocket,
    pub dst: Vec<NodeTopicUri>,
}

#[derive(Debug, Clone)]
pub struct ServerSocketContext {
    pub config: ServerSocket,
    pub listen: NodeTopicUri,
}

#[derive(Debug, Clone)]
pub struct QuerySocketContext {
    pub config: QuerySocket,
    pub connect: Vec<NodeTopicUri>,
}

#[derive(Debug, Clone)]
pub struct NodeTopicUri {
    pub node: NodeWeak,
    pub topic: ValueOrEval,
}
