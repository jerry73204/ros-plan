use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
    },
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
    fn node(&self) -> &IndexMap<NodeIdent, NodeShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.node(),
            ScopeReadGuard::Include(guard) => guard.node(),
        }
    }

    fn pubsub_link(&self) -> &IndexMap<LinkIdent, PubSubLinkShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.pubsub_link(),
            ScopeReadGuard::Include(guard) => guard.pubsub_link(),
        }
    }

    fn service_link(&self) -> &IndexMap<LinkIdent, ServiceLinkShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.service_link(),
            ScopeReadGuard::Include(guard) => guard.service_link(),
        }
    }

    fn include(&self) -> &IndexMap<KeyOwned, PlanScopeShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.include(),
            ScopeReadGuard::Include(guard) => guard.include(),
        }
    }

    fn group(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        match self {
            ScopeReadGuard::Group(guard) => guard.group(),
            ScopeReadGuard::Include(guard) => guard.group(),
        }
    }

    fn key(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        match self {
            ScopeReadGuard::Group(guard) => guard.key(),
            ScopeReadGuard::Include(guard) => guard.key(),
        }
    }
}
