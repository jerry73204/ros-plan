use crate::{
    expr::{BoolExpr, TextOrExpr, ValueOrExpr},
    key::RelativeKeyOwned,
    link::{LinkCfg, LinkIdent},
    node::{NodeCfg, NodeIdent},
    parameter::ParamName,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IncludeCfg {
    pub when: Option<BoolExpr>,
    pub pkg: Option<TextOrExpr>,
    pub file: Option<TextOrExpr>,
    pub path: Option<PathBuf>,

    #[serde(default)]
    pub arg: IndexMap<ParamName, ValueOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroupCfg {
    pub when: Option<BoolExpr>,

    #[serde(default)]
    pub node: IndexMap<NodeIdent, NodeCfg>,

    #[serde(default)]
    pub link: IndexMap<LinkIdent, LinkCfg>,

    #[serde(default)]
    pub include: IndexMap<RelativeKeyOwned, IncludeCfg>,

    #[serde(default)]
    pub group: IndexMap<RelativeKeyOwned, GroupCfg>,
}
