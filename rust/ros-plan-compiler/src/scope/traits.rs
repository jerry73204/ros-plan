use super::{EntityShared, GroupScopeShared, IncludeShared, ScopeShared};
use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
    },
    error::Error,
    selector::RelativeSelector,
};
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
    #[serde(rename = "pubsub_link")]
    PubSubLink,
    ServiceLink,
    Group,
    Include,
}

pub trait ScopeRef {
    fn node(&self) -> &IndexMap<NodeIdent, NodeShared>;
    fn pubsub_link(&self) -> &IndexMap<LinkIdent, PubSubLinkShared>;
    fn service_link(&self) -> &IndexMap<LinkIdent, ServiceLinkShared>;
    fn include(&self) -> &IndexMap<KeyOwned, IncludeShared>;
    fn group(&self) -> &IndexMap<KeyOwned, GroupScopeShared>;
    fn key(&self) -> &BTreeMap<KeyOwned, KeyKind>;
}

pub trait ScopeMut: ScopeRef {
    fn node_mut(&mut self) -> &mut IndexMap<NodeIdent, NodeShared>;
    fn pubsub_link_mut(&mut self) -> &mut IndexMap<LinkIdent, PubSubLinkShared>;
    fn service_link_mut(&mut self) -> &mut IndexMap<LinkIdent, ServiceLinkShared>;
    fn include_mut(&mut self) -> &mut IndexMap<KeyOwned, IncludeShared>;
    fn group_mut(&mut self) -> &mut IndexMap<KeyOwned, GroupScopeShared>;
    fn key_mut(&mut self) -> &mut BTreeMap<KeyOwned, KeyKind>;
}

pub trait ScopeRefExt: ScopeRef {
    // fn subscope_iter(&self) -> Box<dyn Iterator<Item = (&Key, ScopeShared)> + '_> {
    //     let group_iter = self
    //         .group()
    //         .iter()
    //         .map(|(key, group)| (key.as_key(), ScopeShared::from(group.clone())));
    //     let include_iter = self
    //         .include()
    //         .iter()
    //         .map(|(key, include)| (key.as_key(), ScopeShared::from(include.clone())));
    //     Box::new(group_iter.chain(include_iter))
    // }

    fn key_upper_bound(&self, bound: Bound<&Key>) -> Option<(&Key, KeyKind)> {
        let range = (Bound::Unbounded, bound);
        let (sup_key, kind) = self.key().range::<Key, _>(range).next_back()?;
        Some((sup_key, *kind))
    }

    fn key_lower_bound(&self, bound: Bound<&Key>) -> Option<(&Key, KeyKind)> {
        let range = (bound, Bound::Unbounded);
        let (inf_key, kind) = self.key().range::<Key, _>(range).next_back()?;
        Some((inf_key, *kind))
    }

    fn get_entity(&self, ident: &Ident) -> Option<EntityShared> {
        let kind = self.key().get(ident.as_key())?;

        let entity: EntityShared = match kind {
            KeyKind::Node => self.node()[ident].clone().into(),
            KeyKind::PubSubLink => self.pubsub_link()[ident].clone().into(),
            KeyKind::ServiceLink => self.service_link()[ident].clone().into(),
            _ => return None,
        };

        Some(entity)
    }

    fn relative_selector(&self) -> RelativeSelector<'_, Self>
    where
        Self: Sized,
    {
        RelativeSelector::new(self)
    }

    fn get_subscope<'a>(&self, key: &'a Key) -> Option<(ScopeShared, Option<&'a Key>)> {
        let (prefix_key, kind) = self.key_upper_bound(Bound::Included(key))?;

        let suffix_key = match key.strip_prefix(prefix_key) {
            StripKeyPrefix::ImproperPrefix => return None,
            StripKeyPrefix::EmptySuffix => None,
            StripKeyPrefix::Suffix(suffix_key) => Some(suffix_key),
        };

        let subscope: ScopeShared = match kind {
            KeyKind::Group => self.group()[prefix_key].clone().into(),
            KeyKind::Include => {
                let include = &self.include()[prefix_key];
                include.with_read(|guard| guard.plan.clone())?.into()
            }
            _ => return None,
        };

        Some((subscope, suffix_key))
    }

    /// F12: Get subscope with transparency support
    /// Like get_subscope, but for includes checks the transparent flag.
    /// If an include is NOT transparent, returns None (cannot traverse).
    /// If transparent, returns the plan scope and allows traversal.
    fn get_subscope_with_transparency<'a>(
        &self,
        key: &'a Key,
    ) -> Option<(ScopeShared, Option<&'a Key>)> {
        let (prefix_key, kind) = self.key_upper_bound(Bound::Included(key))?;

        let suffix_key = match key.strip_prefix(prefix_key) {
            StripKeyPrefix::ImproperPrefix => return None,
            StripKeyPrefix::EmptySuffix => None,
            StripKeyPrefix::Suffix(suffix_key) => Some(suffix_key),
        };

        let subscope: ScopeShared = match kind {
            KeyKind::Group => self.group()[prefix_key].clone().into(),
            KeyKind::Include => {
                let include = &self.include()[prefix_key];

                // F12: Check transparency flag
                let (is_transparent, plan_opt) = include
                    .with_read(|guard| (guard.transparent.unwrap_or(false), guard.plan.clone()));

                // Only allow traversal if transparent
                if !is_transparent {
                    return None;
                }

                plan_opt?.into()
            }
            _ => return None,
        };

        Some((subscope, suffix_key))
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

        if let Some((sup_key, _)) = self.key_upper_bound(Bound::Included(key)) {
            if key.starts_with(sup_key) {
                return Err(Error::ConflictingKeys {
                    offender: sup_key.to_owned(),
                    inserted: key.to_owned(),
                });
            }
        }

        if let Some((inf_key, _)) = self.key_lower_bound(Bound::Excluded(key)) {
            if inf_key.starts_with(key) {
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
            .key_mut()
            .insert(self.ident.clone().into(), KeyKind::Node);
        self.scope.node_mut().insert(self.ident, node);
    }

    pub fn insert_pubsub_link(self, link: PubSubLinkShared) {
        self.scope
            .key_mut()
            .insert(self.ident.clone().into(), KeyKind::PubSubLink);
        self.scope.pubsub_link_mut().insert(self.ident, link);
    }

    pub fn insert_service_link(self, link: ServiceLinkShared) {
        self.scope
            .key_mut()
            .insert(self.ident.clone().into(), KeyKind::ServiceLink);
        self.scope.service_link_mut().insert(self.ident, link);
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
    pub fn insert_include(self, include: IncludeShared) {
        self.scope
            .key_mut()
            .insert(self.key.clone(), KeyKind::Include);
        self.scope.include_mut().insert(self.key, include);
    }

    pub fn insert_group(self, group: GroupScopeShared) {
        self.scope
            .key_mut()
            .insert(self.key.clone(), KeyKind::Group);
        self.scope.group_mut().insert(self.key, group);
    }
}
