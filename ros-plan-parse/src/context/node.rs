use crate::context::expr::ExprContext;
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, node::NodeCfg, parameter::ParamName};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeContext {
    pub key: KeyOwned,
    pub config: NodeCfg,
    pub param: IndexMap<ParamName, ExprContext>,
}
