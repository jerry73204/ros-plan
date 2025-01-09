use crate::{
    context::{
        arg::ArgContext,
        link::{LinkArc, LinkWeak},
        node::{NodeArc, NodeWeak},
        socket::SocketArc,
    },
    tree::{Tree, TreeRef},
};
use indexmap::IndexMap;
use ros_plan_format::{
    eval::ValueOrEval, key::KeyOwned, link::LinkIdent, node::NodeIdent, parameter::ParamName,
    socket::SocketIdent,
};
use serde::Serialize;
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize)]
pub struct Resource {
    pub root: Option<ResourceTreeRef>,
    pub node_map: IndexMap<KeyOwned, NodeWeak>,
    pub link_map: IndexMap<KeyOwned, LinkWeak>,
}

pub type ResourceTree = Tree<PlanResource>;
pub type ResourceTreeRef = TreeRef<PlanResource>;

#[derive(Debug, Clone, Serialize)]
pub enum PlanResource {
    PlanFile(PlanFileResource),
    HerePlan(GroupResource),
}

impl From<PlanFileResource> for PlanResource {
    fn from(v: PlanFileResource) -> Self {
        Self::PlanFile(v)
    }
}

impl From<GroupResource> for PlanResource {
    fn from(v: GroupResource) -> Self {
        Self::HerePlan(v)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct PlanFileResource {
    pub path: PathBuf,
    pub when: Option<ValueOrEval>,
    pub arg_map: IndexMap<ParamName, ArgContext>,
    pub var_map: IndexMap<ParamName, ValueOrEval>,
    pub socket_map: IndexMap<SocketIdent, SocketArc>,
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone, Serialize)]
pub struct GroupResource {
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}
