use crate::{
    context::{
        link::{LinkArc, LinkWeak},
        node::{NodeArc, NodeWeak},
        socket::SocketArc,
    },
    tree::{Tree, TreeRef},
};
use indexmap::IndexMap;
use ros_plan_format::{
    eval::ValueOrEval,
    key::KeyOwned,
    link::LinkIdent,
    node::NodeIdent,
    parameter::{ArgEntry, ParamName},
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
    PlanFile(IncludeResource),
    HerePlan(HerePlanResource),
}

impl From<IncludeResource> for PlanResource {
    fn from(v: IncludeResource) -> Self {
        Self::PlanFile(v)
    }
}

impl From<HerePlanResource> for PlanResource {
    fn from(v: HerePlanResource) -> Self {
        Self::HerePlan(v)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct IncludeResource {
    pub context: PlanFileResource,
    pub args: IndexMap<ParamName, ValueOrEval>,
    pub when: Option<ValueOrEval>,
}

#[derive(Debug, Clone, Serialize)]
pub struct PlanFileResource {
    pub path: PathBuf,
    pub arg: IndexMap<ParamName, ArgEntry>,
    pub var: IndexMap<ParamName, ValueOrEval>,
    pub socket_map: IndexMap<SocketIdent, SocketArc>,
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}

#[derive(Debug, Clone, Serialize)]
pub struct HerePlanResource {
    pub node_map: IndexMap<NodeIdent, NodeArc>,
    pub link_map: IndexMap<LinkIdent, LinkArc>,
}
