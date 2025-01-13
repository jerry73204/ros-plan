use crate::{
    context::{
        arg::ArgContext, expr::ExprContext, link::LinkContext, node::NodeContext,
        socket::SocketContext,
    },
    utils::{
        shared_table::{Owned, Shared, SharedTable},
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
use serde::Serialize;
use std::path::PathBuf;

pub type ScopeTree = Tree<Scope>;
pub type ScopeTreeRef = TreeRef<Scope>;
pub type NodeOwned = Owned<NodeContext>;
pub type NodeShared = Shared<NodeContext>;
pub type LinkOwned = Owned<LinkContext>;
pub type LinkShared = Shared<LinkContext>;
pub type SocketOwned = Owned<SocketContext>;
pub type SocketShared = Shared<SocketContext>;

#[derive(Debug, Serialize)]
pub struct Resource {
    pub(crate) root: Option<ScopeTreeRef>,
    pub(crate) node_tab: SharedTable<NodeContext>,
    pub(crate) link_tab: SharedTable<LinkContext>,
    pub(crate) socket_tab: SharedTable<SocketContext>,
}

impl Resource {
    pub fn find_scope(&self, key: &Key) -> Option<ScopeTreeRef> {
        let suffix = match key.strip_prefix("/".parse().unwrap()) {
            Ok(Some(suffix)) => suffix,
            _ => return None,
        };
        self.root.as_ref()?.find(suffix)
    }
}

impl ScopeTreeRef {
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

#[derive(Debug, Serialize)]
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

#[derive(Debug, Serialize)]
pub struct PlanFileScope {
    pub path: PathBuf,
    pub when: Option<ExprContext>,
    pub arg_map: IndexMap<ParamName, ArgContext>,
    pub var_map: IndexMap<ParamName, ExprContext>,
    pub socket_map: IndexMap<SocketIdent, SocketShared>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
}

#[derive(Debug, Serialize)]
pub struct GroupScope {
    pub when: Option<ExprContext>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
}
