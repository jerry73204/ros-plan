use crate::{
    context::{link::LinkShared, node::NodeShared},
    scope::{GroupScope, GroupScopeShared, KeyKind, PlanScope, PlanScopeShared, ScopeRef},
};
use indexmap::IndexMap;
use parking_lot::RwLockReadGuard;
use ros_plan_format::{key::KeyOwned, link::LinkIdent, node::NodeIdent};
use std::collections::BTreeMap;

#[derive(Debug)]
pub enum ScopeReadGuard<'a> {
    Group(RwLockReadGuard<'a, GroupScope>),
    Include(RwLockReadGuard<'a, PlanScope>),
}

impl<'a> ScopeReadGuard<'a> {
    pub fn as_group(&self) -> Option<&RwLockReadGuard<'a, GroupScope>> {
        if let Self::Group(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_include(&self) -> Option<&RwLockReadGuard<'a, PlanScope>> {
        if let Self::Include(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl<'a> From<RwLockReadGuard<'a, PlanScope>> for ScopeReadGuard<'a> {
    fn from(v: RwLockReadGuard<'a, PlanScope>) -> Self {
        Self::Include(v)
    }
}

impl<'a> From<RwLockReadGuard<'a, GroupScope>> for ScopeReadGuard<'a> {
    fn from(v: RwLockReadGuard<'a, GroupScope>) -> Self {
        Self::Group(v)
    }
}

impl ScopeRef for ScopeReadGuard<'_> {
    fn node_map(&self) -> &IndexMap<NodeIdent, NodeShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.node_map(),
            ScopeReadGuard::Include(guard) => guard.node_map(),
        }
    }

    fn link_map(&self) -> &IndexMap<LinkIdent, LinkShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.link_map(),
            ScopeReadGuard::Include(guard) => guard.link_map(),
        }
    }

    fn include_map(&self) -> &IndexMap<KeyOwned, PlanScopeShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.include_map(),
            ScopeReadGuard::Include(guard) => guard.include_map(),
        }
    }

    fn group_map(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.group_map(),
            ScopeReadGuard::Include(guard) => guard.group_map(),
        }
    }

    fn key_map(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        match self {
            ScopeReadGuard::Group(guard) => guard.key_map(),
            ScopeReadGuard::Include(guard) => guard.key_map(),
        }
    }
}
