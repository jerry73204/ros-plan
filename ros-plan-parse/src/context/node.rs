use super::{expr::TextOrExprContext, node_socket::NodeSocketShared};
use crate::{
    context::expr::ExprContext,
    utils::shared_table::{Owned, Shared},
};
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, node_socket::NodeSocketIdent, parameter::ParamName};
use serde::{Deserialize, Serialize};

pub type NodeOwned = Owned<NodeContext>;
pub type NodeShared = Shared<NodeContext>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeContext {
    pub key: KeyOwned,
    pub pkg: Option<TextOrExprContext>,
    pub exec: Option<TextOrExprContext>,
    pub plugin: Option<TextOrExprContext>,
    pub param: IndexMap<ParamName, ExprContext>,
    pub socket: IndexMap<NodeSocketIdent, NodeSocketShared>,
}
