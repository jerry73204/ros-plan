use super::{
    traits::ScopeMut, GroupScopeShared, KeyKind, LinkShared, NodeShared, PlanFileScopeShared,
    ScopeRef,
};
use crate::context::expr::ExprContext;
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, link::LinkIdent, node::NodeIdent};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Serialize, Deserialize)]
pub struct GroupScope {
    pub when: Option<ExprContext>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
    pub include_map: IndexMap<KeyOwned, PlanFileScopeShared>,
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

    fn include_map_mut(&mut self) -> &mut IndexMap<KeyOwned, PlanFileScopeShared> {
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

    fn include_map(&self) -> &IndexMap<KeyOwned, PlanFileScopeShared> {
        &self.include_map
    }

    fn group_map(&self) -> &IndexMap<KeyOwned, GroupScopeShared> {
        &self.group_map
    }

    fn key_map(&self) -> &BTreeMap<KeyOwned, KeyKind> {
        &self.key_map
    }
}
