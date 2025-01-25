use crate::{context::expr::ExprContext, scope::NodeSocketShared};
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, node_socket::NodeSocketIdent, parameter::ParamName};
use serde::{Deserialize, Serialize};

use super::expr::TextOrExprContext;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeContext {
    pub key: KeyOwned,
    pub pkg: Option<TextOrExprContext>,
    pub exec: Option<TextOrExprContext>,
    pub plugin: Option<TextOrExprContext>,
    pub param: IndexMap<ParamName, ExprContext>,
    pub socket: IndexMap<NodeSocketIdent, NodeSocketShared>,
}
