use super::{plan_file::PlanScopeShared, traits::ScopeMut, KeyKind, ScopeRef};
use crate::{
    context::{expr::ExprCtx, link::LinkShared, node::NodeShared},
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
    pub when: Option<ExprCtx>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
    pub include_map: IndexMap<KeyOwned, PlanScopeShared>,
    pub group_map: IndexMap<KeyOwned, GroupScopeShared>,
    pub key_map: BTreeMap<KeyOwned, KeyKind>,
}

impl ScopeMut for GroupScope {
    fn node_map_mut(&mut self) -> &mut IndexMap<NodeIdent, NodeShared> {
        &mut self.node_map
    }

    fn link_map_mut(&mut self) -> &mut IndexMap<LinkIdent, LinkShared> {
        &mut self.link_map
    }

    fn include_map_mut(&mut self) -> &mut IndexMap<KeyOwned, PlanScopeShared> {
        &mut self.include_map
    }

    fn group_map_mut(&mut self) -> &mut IndexMap<KeyOwned, GroupScopeShared> {
        &mut self.group_map
    }

    fn key_map_mut(&mut self) -> &mut BTreeMap<KeyOwned, KeyKind> {
        &mut self.key_map
    }
}

impl ScopeRef for GroupScope {
    fn node_map(&self) -> &IndexMap<NodeIdent, NodeShared> {
        &self.node_map
    }

    fn link_map(&self) -> &IndexMap<LinkIdent, LinkShared> {
        &self.link_map
    }

    fn include_map(&self) -> &IndexMap<KeyOwned, PlanScopeShared> {
        &self.include_map
    }

    fn group_map(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        &self.group_map
    }

    fn key_map(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        &self.key_map
    }
}
