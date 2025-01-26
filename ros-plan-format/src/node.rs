use crate::{
    expr::{TextOrExpr, ValueOrExpr},
    ident::IdentOwned,
    node_socket::{NodeSocketCfg, NodeSocketIdent},
    parameter::ParamName,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type NodeIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeCfg {
    pub pkg: Option<TextOrExpr>,
    pub exec: Option<TextOrExpr>,
    pub plugin: Option<TextOrExpr>,

    #[serde(default)]
    pub param: IndexMap<ParamName, ValueOrExpr>,

    #[serde(default)]
    pub socket: IndexMap<NodeSocketIdent, NodeSocketCfg>,
}
