use crate::context::expr::ExprContext;
use indexmap::IndexMap;
use ros_plan_format::{
    node::{ProcessNodeCfg, RosNodeCfg},
    parameter::ParamName,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeContext {
    Ros(RosNodeContext),
    Proc(ProcessContext),
}

impl From<ProcessContext> for NodeContext {
    fn from(v: ProcessContext) -> Self {
        Self::Proc(v)
    }
}

impl From<RosNodeContext> for NodeContext {
    fn from(v: RosNodeContext) -> Self {
        Self::Ros(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RosNodeContext {
    pub config: RosNodeCfg,
    pub param: IndexMap<ParamName, ExprContext>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessContext {
    pub config: ProcessNodeCfg,
}
