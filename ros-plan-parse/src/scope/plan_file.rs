use crate::context::{arg::ArgContext, expr::ExprContext};
use indexmap::IndexMap;
use ros_plan_format::{
    key::KeyOwned, link::LinkIdent, node::NodeIdent, parameter::ParamName,
    plan_socket::PlanSocketIdent,
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::PathBuf};

use super::{
    traits::ScopeMut, GroupScopeShared, KeyKind, LinkShared, NodeShared, PlanFileScopeShared,
    PlanSocketShared, ScopeRef,
};

#[derive(Debug, Serialize, Deserialize)]
pub struct PlanFileScope {
    pub path: PathBuf,
    pub when: Option<ExprContext>,
    pub arg_map: IndexMap<ParamName, ArgContext>,
    pub var_map: IndexMap<ParamName, ExprContext>,
    pub socket_map: IndexMap<PlanSocketIdent, PlanSocketShared>,
    pub node_map: IndexMap<NodeIdent, NodeShared>,
    pub link_map: IndexMap<LinkIdent, LinkShared>,
    pub include_map: IndexMap<KeyOwned, PlanFileScopeShared>,
    pub group_map: IndexMap<KeyOwned, GroupScopeShared>,
    pub key_map: BTreeMap<KeyOwned, KeyKind>,
}

impl ScopeMut for PlanFileScope {
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

impl ScopeRef for PlanFileScope {
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
