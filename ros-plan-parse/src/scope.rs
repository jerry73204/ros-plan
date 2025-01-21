use crate::{
    context::{
        arg::ArgContext, expr::ExprContext, link::LinkContext, node::NodeContext,
        node_socket::NodeSocketContext, plan_socket::PlanSocketContext,
    },
    error::Error,
    utils::{
        arc_rwlock::ArcRwLock,
        shared_table::{Owned, Shared},
    },
};
use indexmap::IndexMap;
use parking_lot::{
    MappedRwLockReadGuard, MappedRwLockWriteGuard, RwLockReadGuard, RwLockWriteGuard,
};
use ros_plan_format::{
    key::{Key, KeyOwned, StripKeyPrefix},
    link::LinkIdent,
    node::NodeIdent,
    parameter::ParamName,
    plan_socket::PlanSocketIdent,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, ops::Bound, path::PathBuf};

pub type NodeOwned = Owned<NodeContext>;
pub type NodeShared = Shared<NodeContext>;

pub type LinkOwned = Owned<LinkContext>;
pub type LinkShared = Shared<LinkContext>;

pub type PlanSocketOwned = Owned<PlanSocketContext>;
pub type PlanSocketShared = Shared<PlanSocketContext>;

pub type NodeSocketOwned = Owned<NodeSocketContext>;
pub type NodeSocketShared = Shared<NodeSocketContext>;

pub type ScopeTreeRef = ArcRwLock<ScopeTree>;

#[derive(Debug, Serialize, Deserialize)]
pub struct ScopeTree {
    pub value: Scope,
    pub children: BTreeMap<KeyOwned, ScopeTreeRef>,
}

impl ScopeTree {
    pub fn new(value: Scope) -> Self {
        Self {
            value,
            children: BTreeMap::new(),
        }
    }
}

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

    pub fn get_immediate_subscope<'a>(&self, key: &'a Key) -> Option<(Self, Option<&'a Key>)> {
        if !key.is_relative() {
            return None;
        }

        let guard = self.read();
        let children = &guard.children;

        let (prev_key, prev_child) = children
            .range::<Key, _>((Bound::Unbounded, Bound::Included(key)))
            .next_back()?;
        let suffix = match key.strip_prefix(prev_key) {
            StripKeyPrefix::ImproperPrefix => return None,
            StripKeyPrefix::EmptySuffix => None,
            StripKeyPrefix::Suffix(suffix) => Some(suffix),
        };
        Some((prev_child.clone(), suffix))
    }

    pub fn insert_immediate_subscope(&self, key: KeyOwned, value: Scope) -> Result<Self, Error> {
        // Reject non-relative keys.
        if !key.is_relative() {
            return Err(Error::InvalidSubplanName {
                key: key.to_owned(),
                reason: "absolute key is not allowed".to_string(),
            });
        }

        // Return error if the key is a prefix or an extension of an
        // existing key.
        let mut guard = self.write();
        let children = &mut guard.children;

        if let Some((prev_key, _)) = children.range(..key.clone()).next_back() {
            if key.starts_with(prev_key) {
                return Err(Error::ConflictingKeys {
                    offender: prev_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        if let Some((next_key, _)) = children.range(key.clone()..).next() {
            if next_key.starts_with(&key) {
                return Err(Error::ConflictingKeys {
                    offender: next_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        // Create the child
        let child: ScopeTreeRef = ScopeTree {
            children: BTreeMap::new(),
            value,
        }
        .into();
        children.insert(key, child.clone());

        Ok(child)
    }

    pub fn find_subscope_within_scope(&self, key: &Key) -> Option<Self> {
        if !key.is_relative() {
            return None;
        }

        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.get_immediate_subscope(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            if curr.is_plan_file() {
                return None;
            }
            let (child, next_suffix) = curr.get_immediate_subscope(&curr_suffix)?;
            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }

    pub fn find_node_within_scope(&self, key: &Key) -> Option<NodeOwned> {
        let (scope_key, Some(node_name)) = key.split_parent() else {
            return None;
        };
        let scope = match scope_key {
            Some(key) => self.find_subscope_within_scope(key)?,
            None => self.clone(),
        };
        let node = {
            let guard = scope.read();
            let shared = guard.value.node_map().get(node_name)?;
            shared.upgrade().unwrap()
        };
        Some(node)
    }

    pub fn find_node_socket_within_scope(&self, key: &Key) -> Option<NodeSocketOwned> {
        let (Some(node_key), Some(socket_name)) = key.split_parent() else {
            return None;
        };
        let node = self.find_node_within_scope(node_key)?;
        let socket = {
            let guard = node.read();
            let shared = guard.socket.get(socket_name)?;
            shared.upgrade().unwrap()
        };
        Some(socket)
    }

    pub fn find_subplan_socket_within_scope(&self, key: &Key) -> Option<PlanSocketOwned> {
        let (Some(scope_key), Some(socket_name)) = key.split_parent() else {
            return None;
        };
        let subscope = self.find_subscope_within_scope(scope_key)?;
        let plan_file = subscope.as_plan_file()?;
        let shared = plan_file.socket_map.get(socket_name)?;
        let socket = shared.upgrade().unwrap();
        Some(socket)
    }

    pub fn find_subscope_unbounded(&self, key: &Key) -> Option<Self> {
        if !key.is_relative() {
            return None;
        }

        let mut curr = self.clone();
        let mut suffix = Some(key);

        while let Some(curr_suffix) = suffix.take() {
            let (child, next_suffix) = curr.get_immediate_subscope(&curr_suffix)?;
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

    pub fn node_map(&self) -> &IndexMap<NodeIdent, NodeShared> {
        match self {
            Scope::PlanFile(scope) => &scope.node_map,
            Scope::Group(scope) => &scope.node_map,
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
    pub socket_map: IndexMap<PlanSocketIdent, PlanSocketShared>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct GroupScope {
    pub when: Option<ExprContext>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
}
