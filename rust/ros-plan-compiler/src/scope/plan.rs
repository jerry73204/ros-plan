use crate::{
    context::{
        arg::ArgCtx,
        link::{PubSubLinkShared, ServiceLinkShared},
        node::NodeShared,
        plan_socket::{PlanCliShared, PlanPubShared, PlanSrvShared, PlanSubShared},
    },
    eval::ValueStore,
    utils::shared_table::{Owned, Shared},
};
use indexmap::IndexMap;
use ros_plan_format::{
    key::KeyOwned, link::LinkIdent, node::NodeIdent, parameter::ParamName,
    plan_socket::PlanSocketIdent,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::PathBuf};

use super::{include::IncludeShared, traits::ScopeMut, GroupScopeShared, KeyKind, ScopeRef};

pub type PlanScopeOwned = Owned<PlanScope>;
pub type PlanScopeShared = Shared<PlanScope>;

#[derive(Debug, Serialize, Deserialize)]
pub struct PlanScope {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgCtx>,
    pub var: IndexMap<ParamName, ValueStore>,
    #[serde(rename = "pub")]
    pub pub_: IndexMap<PlanSocketIdent, PlanPubShared>,
    pub sub: IndexMap<PlanSocketIdent, PlanSubShared>,
    pub srv: IndexMap<PlanSocketIdent, PlanSrvShared>,
    pub cli: IndexMap<PlanSocketIdent, PlanCliShared>,
    pub node: IndexMap<NodeIdent, NodeShared>,
    pub pubsub_link: IndexMap<LinkIdent, PubSubLinkShared>,
    pub service_link: IndexMap<LinkIdent, ServiceLinkShared>,
    pub include: IndexMap<KeyOwned, IncludeShared>,
    pub group: IndexMap<KeyOwned, GroupScopeShared>,
    pub key: BTreeMap<KeyOwned, KeyKind>,
}

impl ScopeMut for PlanScope {
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

impl ScopeRef for PlanScope {
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
