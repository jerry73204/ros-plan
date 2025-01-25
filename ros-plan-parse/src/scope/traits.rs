use super::{
    EntityShared, GroupScopeShared, LinkShared, NodeShared, NodeSocketShared, PlanFileScopeShared,
    PlanSocketShared, ScopeShared,
};
use crate::error::Error;
use indexmap::IndexMap;
use ros_plan_format::{
    ident::{Ident, IdentOwned},
    key::{Key, KeyOwned, StripKeyPrefix},
    link::LinkIdent,
    node::NodeIdent,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, ops::Bound};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum KeyKind {
    Node,
    Link,
    Group,
    Include,
}

pub trait ScopeRef {
    fn node_map(&self) -> &IndexMap<NodeIdent, NodeShared>;
    fn link_map(&self) -> &IndexMap<LinkIdent, LinkShared>;
    fn include_map(&self) -> &IndexMap<KeyOwned, PlanFileScopeShared>;
    fn group_map(&self) -> &IndexMap<KeyOwned, GroupScopeShared>;
    fn key_map(&self) -> &BTreeMap<KeyOwned, KeyKind>;
}

pub trait ScopeMut: ScopeRef {
    fn node_map_mut(&mut self) -> &mut IndexMap<NodeIdent, NodeShared>;
    fn link_map_mut(&mut self) -> &mut IndexMap<LinkIdent, LinkShared>;
    fn include_map_mut(&mut self) -> &mut IndexMap<KeyOwned, PlanFileScopeShared>;
    fn group_map_mut(&mut self) -> &mut IndexMap<KeyOwned, GroupScopeShared>;
    fn key_map_mut(&mut self) -> &mut BTreeMap<KeyOwned, KeyKind>;
}

pub trait ScopeRefExt: ScopeRef {
    fn subscope_iter(&self) -> Box<dyn Iterator<Item = (&Key, ScopeShared)> + '_> {
        let group_iter = self
            .group_map()
            .iter()
            .map(|(key, group)| (key.as_key(), ScopeShared::from(group.clone())));
        let include_iter = self
            .include_map()
            .iter()
            .map(|(key, include)| (key.as_key(), ScopeShared::from(include.clone())));
        Box::new(group_iter.chain(include_iter))
    }

    fn key_upper_bound(&self, bound: Bound<&Key>) -> Option<(&Key, KeyKind)> {
        let range = (Bound::Unbounded, bound);
        let (sup_key, kind) = self.key_map().range::<Key, _>(range).next_back()?;
        Some((sup_key, *kind))
    }

    fn key_lower_bound(&self, bound: Bound<&Key>) -> Option<(&Key, KeyKind)> {
        let range = (bound, Bound::Unbounded);
        let (inf_key, kind) = self.key_map().range::<Key, _>(range).next_back()?;
        Some((inf_key, *kind))
    }

    fn get_entity(&self, ident: &Ident) -> Option<EntityShared> {
        let kind = self.key_map().get(ident.as_key())?;

        let entity: EntityShared = match kind {
            KeyKind::Node => self.node_map()[ident].clone().into(),
            KeyKind::Link => self.link_map()[ident].clone().into(),
            _ => return None,
        };

        Some(entity)
    }

    fn get_subscope<'a>(&self, key: &'a Key) -> Option<(ScopeShared, Option<&'a Key>)> {
        let (prefix_key, kind) = self.key_upper_bound(Bound::Included(key))?;

        let suffix_key = match key.strip_prefix(prefix_key) {
            StripKeyPrefix::ImproperPrefix => return None,
            StripKeyPrefix::EmptySuffix => None,
            StripKeyPrefix::Suffix(suffix_key) => Some(suffix_key),
        };

        let subscope: ScopeShared = match kind {
            KeyKind::Group => self.group_map()[prefix_key].clone().into(),
            KeyKind::Include => self.include_map()[prefix_key].clone().into(),
            _ => return None,
        };

        Some((subscope, suffix_key))
    }

    fn get_subscope_recursive_unbounded(&self, key: &Key) -> Option<ScopeShared> {
        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.get_subscope(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            let (child, next_suffix) = match curr {
                ScopeShared::Group(shared) => {
                    let owned = shared.upgrade().unwrap();
                    let guard = owned.read();
                    guard.get_subscope(curr_suffix)?
                }
                ScopeShared::Include(shared) => {
                    let owned = shared.upgrade().unwrap();
                    let guard = owned.read();
                    guard.get_subscope(curr_suffix)?
                }
            };

            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }

    fn get_subscope_recursive_bounded(&self, key: &Key) -> Option<ScopeShared> {
        // The first step can start from a plan or a group node.
        let (mut curr, mut suffix) = self.get_subscope(key)?;

        // In later steps, it can only start from a group node.
        while let Some(curr_suffix) = suffix.take() {
            let ScopeShared::Group(shared) = curr else {
                return None;
            };
            let owned = shared.upgrade().unwrap();
            let guard = owned.read();

            let (child, next_suffix) = guard.get_subscope(curr_suffix)?;
            curr = child;
            suffix = next_suffix;
        }

        Some(curr)
    }

    fn get_node_recursive_bounded(&self, key: &Key) -> Option<NodeShared> {
        let (scope_key, node_ident) = key.split_parent();
        let node_ident = node_ident?;

        match scope_key {
            Some(scope_key) => {
                let shared = self.get_subscope_recursive_bounded(scope_key)?;
                let owned = shared.upgrade().unwrap();
                let guard = owned.read();
                guard.node_map().get(node_ident).cloned()
            }
            None => self.node_map().get(node_ident).cloned(),
        }
    }

    fn get_node_socket_recursive_bounded(&self, key: &Key) -> Option<NodeSocketShared> {
        let (node_key, socket_ident) = key.split_parent();
        let node_key = node_key?;
        let socket_ident = socket_ident?;

        let shared = self.get_node_recursive_bounded(node_key)?;
        let owned = shared.upgrade().unwrap();
        let guard = owned.read();
        guard.socket.get(socket_ident).cloned()
    }

    fn get_plan_socket_recursive_bounded(&self, key: &Key) -> Option<PlanSocketShared> {
        let (scope_key, socket_ident) = key.split_parent();
        let scope_key = scope_key?;
        let socket_ident = socket_ident?;

        let shared = self.get_subscope_recursive_bounded(scope_key)?;
        let owned = shared.upgrade().unwrap();
        let guard = owned.read();
        let include = guard.as_include()?;
        include.socket_map.get(socket_ident).cloned()
    }
}

impl<T: ScopeRef> ScopeRefExt for T {}

pub trait ScopeMutExt: ScopeMut {
    fn scope_entry(&mut self, key: KeyOwned) -> Result<ScopeEntry<'_, Self>, Error>
    where
        Self: Sized,
    {
        if let Some((sup_key, _)) = self.key_upper_bound(Bound::Included(&key)) {
            if key.starts_with(sup_key) {
                return Err(Error::ConflictingKeys {
                    offender: sup_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        if let Some((inf_key, _)) = self.key_lower_bound(Bound::Excluded(&key)) {
            if inf_key.starts_with(&key) {
                return Err(Error::ConflictingKeys {
                    offender: inf_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        Ok(ScopeEntry { scope: self, key })
    }

    fn entity_entry(&mut self, ident: IdentOwned) -> Result<EntityEntry<'_, Self>, Error>
    where
        Self: Sized,
    {
        let key = ident.as_key();

        if let Some((sup_key, _)) = self.key_upper_bound(Bound::Included(&key)) {
            if key.starts_with(sup_key) {
                return Err(Error::ConflictingKeys {
                    offender: sup_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        if let Some((inf_key, _)) = self.key_lower_bound(Bound::Excluded(&key)) {
            if inf_key.starts_with(&key) {
                return Err(Error::ConflictingKeys {
                    offender: inf_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        Ok(EntityEntry { scope: self, ident })
    }
}

impl<T: ScopeMut> ScopeMutExt for T {}

pub struct EntityEntry<'a, S: ScopeMut> {
    scope: &'a mut S,
    ident: IdentOwned,
}

impl<S> EntityEntry<'_, S>
where
    S: ScopeMut,
{
    pub fn insert_node(self, node: NodeShared) {
        self.scope
            .key_map_mut()
            .insert(self.ident.clone().into(), KeyKind::Node);
        self.scope.node_map_mut().insert(self.ident, node);
    }

    pub fn insert_link(self, link: LinkShared) {
        self.scope
            .key_map_mut()
            .insert(self.ident.clone().into(), KeyKind::Node);
        self.scope.link_map_mut().insert(self.ident, link);
    }
}

pub struct ScopeEntry<'a, S: ScopeMut> {
    scope: &'a mut S,
    key: KeyOwned,
}

impl<S> ScopeEntry<'_, S>
where
    S: ScopeMut,
{
    pub fn insert_include(self, include: PlanFileScopeShared) {
        self.scope
            .key_map_mut()
            .insert(self.key.clone(), KeyKind::Node);
        self.scope.include_map_mut().insert(self.key, include);
    }

    pub fn insert_group(self, group: GroupScopeShared) {
        self.scope
            .key_map_mut()
            .insert(self.key.clone(), KeyKind::Node);
        self.scope.group_map_mut().insert(self.key, group);
    }
}
