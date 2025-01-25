use crate::{
    expr::{TextOrExpr, ValueOrExpr},
    ident::IdentOwned,
    node_socket::NodeSocketTable,
    parameter::ParamName,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type NodeIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(transparent)]
pub struct NodeTable(pub IndexMap<NodeIdent, NodeCfg>);

impl Default for NodeTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeCfg {
    pub pkg: Option<TextOrExpr>,
    pub exec: Option<TextOrExpr>,
    pub plugin: Option<TextOrExpr>,

    #[serde(default)]
    pub param: IndexMap<ParamName, ValueOrExpr>,

    #[serde(default)]
    pub socket: NodeSocketTable,
}
