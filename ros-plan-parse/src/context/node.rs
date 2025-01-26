use super::{expr::TextOrExprCtx, node_socket::NodeSocketShared};
use crate::{
    context::expr::ExprCtx,
    utils::shared_table::{Owned, Shared},
};
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, node_socket::NodeSocketIdent, parameter::ParamName};
use serde::{Deserialize, Serialize};

pub type NodeOwned = Owned<NodeCtx>;
pub type NodeShared = Shared<NodeCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeCtx {
    pub key: KeyOwned,
    pub pkg: Option<TextOrExprCtx>,
    pub exec: Option<TextOrExprCtx>,
    pub plugin: Option<TextOrExprCtx>,
    pub param: IndexMap<ParamName, ExprCtx>,
    pub socket: IndexMap<NodeSocketIdent, NodeSocketShared>,
}
