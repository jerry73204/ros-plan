use crate::{
    context::{
        arg::ArgContext, expr::ExprContext, link::LinkContext, node::NodeContext,
        socket::SocketContext,
    },
    utils::{
        shared_table::{Owned, Shared},
        tree::{Tree, TreeRef},
    },
};
use indexmap::IndexMap;
use parking_lot::{
    MappedRwLockReadGuard, MappedRwLockWriteGuard, RwLockReadGuard, RwLockWriteGuard,
};
use ros_plan_format::{
    key::Key, link::LinkIdent, node::NodeIdent, parameter::ParamName, socket::SocketIdent,
};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

pub type NodeOwned = Owned<NodeContext>;
pub type NodeShared = Shared<NodeContext>;

pub type LinkOwned = Owned<LinkContext>;
pub type LinkShared = Shared<LinkContext>;

pub type SocketOwned = Owned<SocketContext>;
pub type SocketShared = Shared<SocketContext>;

pub type ScopeTree = Tree<Scope>;
pub type ScopeTreeRef = TreeRef<Scope>;

impl ScopeTreeRef {
    pub fn kind(&self) -> ScopeKind {
        let guard = self.read();
        guard.value.kind()
    }

    pub fn is_plan_file(&self) -> bool {
        let guard = self.read();
        matches!(guard.value, Scope::PlanFile(_))
    }

    pub fn as_plan_file(&self) -> Option<MappedRwLockReadGuard<PlanFileScope>> {
        let guard = self.read();
        RwLockReadGuard::try_map(guard, |g| g.value.as_plan_file()).ok()
    }

    pub fn as_plan_file_mut(&self) -> Option<MappedRwLockWriteGuard<PlanFileScope>> {
        let guard = self.write();
        RwLockWriteGuard::try_map(guard, |g| g.value.as_plan_file_mut()).ok()
    }

    pub fn is_group(&self) -> bool {
        let guard = self.read();
        matches!(guard.value, Scope::Group(_))
    }

    pub fn as_group(&self) -> Option<MappedRwLockReadGuard<GroupScope>> {
        let guard = self.read();
        RwLockReadGuard::try_map(guard, |g| g.value.as_group()).ok()
    }

    pub fn as_group_mut(&self) -> Option<MappedRwLockWriteGuard<GroupScope>> {
        let guard = self.write();
        RwLockWriteGuard::try_map(guard, |g| g.value.as_group_mut()).ok()
    }

    pub fn resolve_key(&self, key: &Key) -> Option<Self> {
        if key.is_absolute() {
            return None;
        }
        if key.is_empty() {
            return Some(self.clone());
        }

        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.get_child(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            if curr.is_plan_file() {
                return None;
            }
            let (child, next_suffix) = curr.get_child(&curr_suffix)?;
            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Scope {
    PlanFile(Box<PlanFileScope>),
    Group(Box<GroupScope>),
}

impl Scope {
    pub fn kind(&self) -> ScopeKind {
        match self {
            Scope::PlanFile(_) => ScopeKind::PlanFile,
            Scope::Group(_) => ScopeKind::Group,
        }
    }

    pub fn is_plan_file(&self) -> bool {
        matches!(self, Self::PlanFile(_))
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

    pub fn is_group(&self) -> bool {
        matches!(self, Self::Group(_))
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
        Self::PlanFile(Box::new(v))
    }
}

impl From<GroupScope> for Scope {
    fn from(v: GroupScope) -> Self {
        Self::Group(Box::new(v))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ScopeKind {
    PlanFile,
    Group,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PlanFileScope {
    pub path: PathBuf,
    pub when: Option<ExprContext>,
    pub arg_map: IndexMap<ParamName, ArgContext>,
    pub var_map: IndexMap<ParamName, ExprContext>,
    pub socket_map: IndexMap<SocketIdent, SocketShared>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct GroupScope {
    pub when: Option<ExprContext>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
}
