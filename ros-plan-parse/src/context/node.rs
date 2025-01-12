use crate::context::expr::ExprContext;
use indexmap::IndexMap;
use ros_plan_format::{
    node::{ProcessNodeCfg, RosNodeCfg},
    parameter::ParamName,
};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
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

#[derive(Debug, Clone, Serialize)]
pub struct RosNodeContext {
    pub config: RosNodeCfg,
    pub param: IndexMap<ParamName, ExprContext>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ProcessContext {
    pub config: ProcessNodeCfg,
}
