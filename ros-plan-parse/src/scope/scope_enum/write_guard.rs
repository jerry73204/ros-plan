use crate::{
    context::{link::LinkShared, node::NodeShared},
    scope::{
        GroupScope, GroupScopeShared, KeyKind, PlanScope, PlanScopeShared, ScopeMut, ScopeRef,
    },
};
use indexmap::IndexMap;
use parking_lot::RwLockWriteGuard;
use ros_plan_format::{key::KeyOwned, link::LinkIdent, node::NodeIdent};
use std::collections::BTreeMap;

#[derive(Debug)]
pub enum ScopeWriteGuard<'a> {
    Group(RwLockWriteGuard<'a, GroupScope>),
    Include(RwLockWriteGuard<'a, PlanScope>),
}

impl<'a> ScopeWriteGuard<'a> {
    pub fn as_group(&self) -> Option<&RwLockWriteGuard<'a, GroupScope>> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_group_mut(&mut self) -> Option<&mut RwLockWriteGuard<'a, GroupScope>> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include(&self) -> Option<&RwLockWriteGuard<'a, PlanScope>> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include_mut(&mut self) -> Option<&mut RwLockWriteGuard<'a, PlanScope>> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl<'a> From<RwLockWriteGuard<'a, PlanScope>> for ScopeWriteGuard<'a> {
    fn from(v: RwLockWriteGuard<'a, PlanScope>) -> Self {
        Self::Include(v)
    }
}

impl<'a> From<RwLockWriteGuard<'a, GroupScope>> for ScopeWriteGuard<'a> {
    fn from(v: RwLockWriteGuard<'a, GroupScope>) -> Self {
        Self::Group(v)
    }
}

impl ScopeRef for ScopeWriteGuard<'_> {
    fn node_map(&self) -> &IndexMap<NodeIdent, NodeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.node_map(),
            ScopeWriteGuard::Include(guard) => guard.node_map(),
        }
    }

    fn link_map(&self) -> &IndexMap<LinkIdent, LinkShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.link_map(),
            ScopeWriteGuard::Include(guard) => guard.link_map(),
        }
    }

    fn include_map(&self) -> &IndexMap<KeyOwned, PlanScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.include_map(),
            ScopeWriteGuard::Include(guard) => guard.include_map(),
        }
    }

    fn group_map(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.group_map(),
            ScopeWriteGuard::Include(guard) => guard.group_map(),
        }
    }

    fn key_map(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.key_map(),
            ScopeWriteGuard::Include(guard) => guard.key_map(),
        }
    }
}

impl ScopeMut for ScopeWriteGuard<'_> {
    fn node_map_mut(&mut self) -> &mut IndexMap<NodeIdent, NodeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.node_map_mut(),
            ScopeWriteGuard::Include(guard) => guard.node_map_mut(),
        }
    }

    fn link_map_mut(&mut self) -> &mut IndexMap<LinkIdent, LinkShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.link_map_mut(),
            ScopeWriteGuard::Include(guard) => guard.link_map_mut(),
        }
    }

    fn include_map_mut(&mut self) -> &mut IndexMap<KeyOwned, PlanScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.include_map_mut(),
            ScopeWriteGuard::Include(guard) => guard.include_map_mut(),
        }
    }

    fn group_map_mut(&mut self) -> &mut IndexMap<KeyOwned, GroupScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.group_map_mut(),
            ScopeWriteGuard::Include(guard) => guard.group_map_mut(),
        }
    }

    fn key_map_mut(&mut self) -> &mut BTreeMap<KeyOwned, KeyKind> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.key_map_mut(),
            ScopeWriteGuard::Include(guard) => guard.key_map_mut(),
        }
    }
}
