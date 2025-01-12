use crate::{
    context::{
        arg::ArgContext,
        link::{LinkArc, LinkWeak},
        node::{NodeArc, NodeWeak},
        socket::SocketArc,
    },
    eval::EvalSlot,
    tree::{Tree, TreeRef},
};
use indexmap::IndexMap;
use parking_lot::{
    MappedRwLockReadGuard, MappedRwLockWriteGuard, RwLockReadGuard, RwLockWriteGuard,
};
use ros_plan_format::{
    key::KeyOwned, link::LinkIdent, node::NodeIdent, parameter::ParamName, socket::SocketIdent,
};
use serde::Serialize;
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize)]
pub struct Resource {
    pub root: Option<ResourceTreeRef>,
    pub node_map: IndexMap<KeyOwned, NodeWeak>,
    pub link_map: IndexMap<KeyOwned, LinkWeak>,
}

pub type ResourceTree = Tree<Scope>;
pub type ResourceTreeRef = TreeRef<Scope>;

impl ResourceTreeRef {
    pub fn kind(&self) -> ScopeKind {
        let guard = self.read();
        guard.value.kind()
    }

    pub fn as_plan_file(&self) -> Option<MappedRwLockReadGuard<PlanFileScope>> {
        let guard = self.read();
        RwLockReadGuard::try_map(guard, |g| g.value.as_plan_file()).ok()
    }

    pub fn as_plan_file_mut(&self) -> Option<MappedRwLockWriteGuard<PlanFileScope>> {
        let guard = self.write();
        RwLockWriteGuard::try_map(guard, |g| g.value.as_plan_file_mut()).ok()
    }

    pub fn as_group(&self) -> Option<MappedRwLockReadGuard<GroupScope>> {
        let guard = self.read();
        RwLockReadGuard::try_map(guard, |g| g.value.as_group()).ok()
    }

    pub fn as_group_mut(&self) -> Option<MappedRwLockWriteGuard<GroupScope>> {
        let guard = self.write();
        RwLockWriteGuard::try_map(guard, |g| g.value.as_group_mut()).ok()
    }
}

#[derive(Debug, Clone, Serialize)]
pub enum Scope {
    PlanFile(PlanFileScope),
    Group(GroupScope),
}

impl Scope {
    pub fn kind(&self) -> ScopeKind {
        match self {
            Scope::PlanFile(_) => ScopeKind::PlanFile,
            Scope::Group(_) => ScopeKind::Group,
        }
    }

    pub fn as_plan_file(&self) -> Option<&PlanFileScope> {
        if let Self::PlanFile(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_plan_file_mut(&mut self) -> Option<&mut PlanFileScope> {
        if let Self::PlanFile(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group(&self) -> Option<&GroupScope> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group_mut(&mut self) -> Option<&mut GroupScope> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PlanFileScope> for Scope {
    fn from(v: PlanFileScope) -> Self {
        Self::PlanFile(v)
    }
}

impl From<GroupScope> for Scope {
    fn from(v: GroupScope) -> Self {
        Self::Group(v)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ScopeKind {
    PlanFile,
    Group,
}

#[derive(Debug, Clone, Serialize)]
pub struct PlanFileScope {
    pub path: PathBuf,
    pub when: Option<EvalSlot>,
    pub arg_map: IndexMap<ParamName, ArgContext>,
    pub var_map: IndexMap<ParamName, EvalSlot>,
    pub socket_map: IndexMap<SocketIdent, SocketArc>,
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone, Serialize)]
pub struct GroupScope {
    pub when: Option<EvalSlot>,
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}
