use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
    },
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
    fn node(&self) -> &IndexMap<NodeIdent, NodeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.node(),
            ScopeWriteGuard::Include(guard) => guard.node(),
        }
    }

    fn pubsub_link(&self) -> &IndexMap<LinkIdent, PubSubLinkShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.pubsub_link(),
            ScopeWriteGuard::Include(guard) => guard.pubsub_link(),
        }
    }

    fn service_link(&self) -> &IndexMap<LinkIdent, ServiceLinkShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.service_link(),
            ScopeWriteGuard::Include(guard) => guard.service_link(),
        }
    }

    fn include(&self) -> &IndexMap<KeyOwned, PlanScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.include(),
            ScopeWriteGuard::Include(guard) => guard.include(),
        }
    }

    fn group(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.group(),
            ScopeWriteGuard::Include(guard) => guard.group(),
        }
    }

    fn key(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.key(),
            ScopeWriteGuard::Include(guard) => guard.key(),
        }
    }
}

impl ScopeMut for ScopeWriteGuard<'_> {
    fn node_mut(&mut self) -> &mut IndexMap<NodeIdent, NodeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.node_mut(),
            ScopeWriteGuard::Include(guard) => guard.node_mut(),
        }
    }

    fn pubsub_link_mut(&mut self) -> &mut IndexMap<LinkIdent, PubSubLinkShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.pubsub_link_mut(),
            ScopeWriteGuard::Include(guard) => guard.pubsub_link_mut(),
        }
    }

    fn service_link_mut(&mut self) -> &mut IndexMap<LinkIdent, ServiceLinkShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.service_link_mut(),
            ScopeWriteGuard::Include(guard) => guard.service_link_mut(),
        }
    }

    fn include_mut(&mut self) -> &mut IndexMap<KeyOwned, PlanScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.include_mut(),
            ScopeWriteGuard::Include(guard) => guard.include_mut(),
        }
    }

    fn group_mut(&mut self) -> &mut IndexMap<KeyOwned, GroupScopeShared> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.group_mut(),
            ScopeWriteGuard::Include(guard) => guard.group_mut(),
        }
    }

    fn key_mut(&mut self) -> &mut BTreeMap<KeyOwned, KeyKind> {
        match self {
            ScopeWriteGuard::Group(guard) => guard.key_mut(),
            ScopeWriteGuard::Include(guard) => guard.key_mut(),
        }
    }
}
