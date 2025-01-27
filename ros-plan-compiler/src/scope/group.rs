use super::{include::IncludeShared, traits::ScopeMut, KeyKind, ScopeRef};
use crate::{
    context::{
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
    },
    eval::BoolStore,
    utils::shared_table::{Owned, Shared},
};
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, link::LinkIdent, node::NodeIdent};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub type GroupScopeOwned = Owned<GroupScope>;
pub type GroupScopeShared = Shared<GroupScope>;

#[derive(Debug, Serialize, Deserialize)]
pub struct GroupScope {
    pub when: Option<BoolStore>,
    pub node: IndexMap<NodeIdent, NodeShared>,
    pub pubsub_link: IndexMap<LinkIdent, PubSubLinkShared>,
    pub service_link: IndexMap<LinkIdent, ServiceLinkShared>,
    pub include: IndexMap<KeyOwned, IncludeShared>,
    pub group: IndexMap<KeyOwned, GroupScopeShared>,
    pub key: BTreeMap<KeyOwned, KeyKind>,
}

impl ScopeMut for GroupScope {
    fn node_mut(&mut self) -> &mut IndexMap<NodeIdent, NodeShared> {
        &mut self.node
    }

    fn pubsub_link_mut(&mut self) -> &mut IndexMap<LinkIdent, PubSubLinkShared> {
        &mut self.pubsub_link
    }

    fn service_link_mut(&mut self) -> &mut IndexMap<LinkIdent, ServiceLinkShared> {
        &mut self.service_link
    }

    fn include_mut(&mut self) -> &mut IndexMap<KeyOwned, IncludeShared> {
        &mut self.include
    }

    fn group_mut(&mut self) -> &mut IndexMap<KeyOwned, GroupScopeShared> {
        &mut self.group
    }

    fn key_mut(&mut self) -> &mut BTreeMap<KeyOwned, KeyKind> {
        &mut self.key
    }
}

impl ScopeRef for GroupScope {
    fn node(&self) -> &IndexMap<NodeIdent, NodeShared> {
        &self.node
    }

    fn pubsub_link(&self) -> &IndexMap<LinkIdent, PubSubLinkShared> {
        &self.pubsub_link
    }

    fn service_link(&self) -> &IndexMap<LinkIdent, ServiceLinkShared> {
        &self.service_link
    }

    fn include(&self) -> &IndexMap<KeyOwned, IncludeShared> {
        &self.include
    }

    fn group(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        &self.group
    }

    fn key(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        &self.key
    }
}
