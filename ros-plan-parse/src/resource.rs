use crate::{
    context::{
        link::{LinkArc, LinkWeak},
        node::{NodeArc, NodeWeak},
        socket::SocketArc,
    },
    error::Error,
    utils::ArcRwLock,
};
use indexmap::IndexMap;
use itertools::Itertools;
use parking_lot::{RwLockReadGuard, RwLockWriteGuard};
use ros_plan_format::{
    ident::{Ident, IdentOwned},
    key::{Key, KeyOwned},
    link::LinkIdent,
    node::NodeIdent,
    parameter::{ArgEntry, ParamName},
    socket::SocketIdent,
};
use serde::Serialize;
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize)]
pub struct PlanResource {
    pub namespace: KeyOwned,
    pub root: TrieRef,
    // pub plan_map: HashMap<PathBuf, TrieRef>,
    pub node_map: IndexMap<KeyOwned, NodeWeak>,
    pub link_map: IndexMap<KeyOwned, LinkWeak>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Trie {
    pub context: Option<TrieContext>,
    pub children: IndexMap<IdentOwned, TrieRef>,
}

impl Default for Trie {
    fn default() -> Self {
        Self {
            children: IndexMap::new(),
            context: None,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(transparent)]
pub struct TrieRef(ArcRwLock<Trie>);

impl TrieRef {
    pub fn get_child(&self, ident: &Ident) -> Option<TrieRef> {
        let guard = self.read();
        let child = guard.children.get(ident)?;
        Some(child.clone())
    }

    pub fn insert(&self, key: &Key, context: TrieContext) -> Result<TrieRef, Error> {
        use indexmap::map::Entry as E;

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

#[derive(Debug, Clone, Serialize)]
pub enum TrieContext {
    Plan(PlanContext),
    HerePlan(HerePlanContext),
}

impl From<HerePlanContext> for TrieContext {
    fn from(v: HerePlanContext) -> Self {
        Self::HerePlan(v)
    }
}

impl From<PlanContext> for TrieContext {
    fn from(v: PlanContext) -> Self {
        Self::Plan(v)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct PlanContext {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgEntry>,
    pub socket_map: IndexMap<SocketIdent, SocketArc>,
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone, Serialize)]
pub struct HerePlanContext {
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}
