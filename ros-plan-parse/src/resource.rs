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
use parking_lot::{RwLockReadGuard, RwLockWriteGuard};
use ros_plan_format::{
    eval::ValueOrEval,
    key::{Key, KeyOwned},
    link::LinkIdent,
    node::NodeIdent,
    parameter::{ArgEntry, ParamName},
    socket::SocketIdent,
};
use serde::Serialize;
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize)]
pub struct Resource {
    pub root: ResourceTreeRef,
    pub node_map: IndexMap<KeyOwned, NodeWeak>,
    pub link_map: IndexMap<KeyOwned, LinkWeak>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ResourceTree {
    pub context: Option<PlanResource>,
    pub children: IndexMap<KeyOwned, ResourceTreeRef>,
}

impl Default for ResourceTree {
    fn default() -> Self {
        Self {
            children: IndexMap::new(),
            context: None,
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(transparent)]
pub struct ResourceTreeRef(ArcRwLock<ResourceTree>);

impl ResourceTreeRef {
    pub fn get_child(&self, key: &Key) -> Option<ResourceTreeRef> {
        let guard = self.read();
        let child = guard.children.get(key)?;
        Some(child.clone())
    }

    pub fn insert(&self, key: &Key, context: PlanResource) -> Result<ResourceTreeRef, Error> {
        use indexmap::map::Entry as E;

        // Reject absolute keys.
        if key.is_absolute() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "absolute key is not allowed".to_string(),
            });
        }

        // Reject empty keys.
        if key.is_empty() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "empty key not allowed".to_string(),
            });
        }

        let child = {
            let mut guard = self.write();
            match guard.children.entry(key.to_owned()) {
                E::Vacant(entry) => entry.insert(ResourceTreeRef::default()).clone(),
                E::Occupied(_) => {
                    return Err(Error::ConflictingKeys {
                        old: key.to_owned(),
                        new: key.to_owned(),
                    })
                }
            }
        };

        // Check if the destination node already has a context.
        {
            let mut guard = child.write();
            if guard.context.is_some() || !guard.children.is_empty() {
                return Err(Error::ConflictingKeys {
                    old: key.to_owned(),
                    new: key.to_owned(),
                });
            }
            guard.context = Some(context);
        }
        Ok(child)
    }

    pub(crate) fn read(&self) -> RwLockReadGuard<ResourceTree> {
        self.0.read()
    }

    pub(crate) fn write(&self) -> RwLockWriteGuard<ResourceTree> {
        self.0.write()
    }
}

impl Default for ResourceTreeRef {
    fn default() -> Self {
        Self(ResourceTree::default().into())
    }
}

impl From<ResourceTree> for ResourceTreeRef {
    fn from(inner: ResourceTree) -> Self {
        Self(inner.into())
    }
}

#[derive(Debug, Clone, Serialize)]
pub enum PlanResource {
    PlanFile(IncludeResource),
    HerePlan(HerePlanResource),
}

impl From<IncludeResource> for PlanResource {
    fn from(v: IncludeResource) -> Self {
        Self::PlanFile(v)
    }
}

impl From<HerePlanResource> for PlanResource {
    fn from(v: HerePlanResource) -> Self {
        Self::HerePlan(v)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct IncludeResource {
    pub context: PlanFileResource,
    pub args: IndexMap<ParamName, ValueOrEval>,
    pub when: Option<ValueOrEval>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PlanFileResource {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgEntry>,
    pub var: IndexMap<ParamName, ValueOrEval>,
    pub socket_map: IndexMap<SocketIdent, SocketArc>,
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone, Serialize)]
pub struct HerePlanResource {
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}
