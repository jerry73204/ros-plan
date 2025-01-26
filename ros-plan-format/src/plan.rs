use crate::{
    argument::ArgEntry,
    expr::ValueOrExpr,
    key::RelativeKeyOwned,
    link::{LinkCfg, LinkIdent},
    node::{NodeCfg, NodeIdent},
    parameter::ParamName,
    plan_socket::{PlanSocketCfg, PlanSocketIdent},
    subplan::{GroupCfg, IncludeCfg},
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Plan {
    #[serde(default)]
    pub arg: IndexMap<ParamName, ArgEntry>,

    #[serde(default)]
    pub var: IndexMap<ParamName, ValueOrExpr>,

    #[serde(default)]
    pub socket: IndexMap<PlanSocketIdent, PlanSocketCfg>,

    #[serde(default)]
    pub node: IndexMap<NodeIdent, NodeCfg>,

    #[serde(default)]
    pub link: IndexMap<LinkIdent, LinkCfg>,

    #[serde(default)]
    pub include: IndexMap<RelativeKeyOwned, IncludeCfg>,

    #[serde(default)]
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}
