use crate::context::expr::ExprContext;
use indexmap::IndexMap;
use ros_plan_format::{
    key::{Key, KeyOwned},
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

impl NodeContext {
    pub fn key(&self) -> &Key {
        match self {
            NodeContext::Ros(node) => &node.key,
            NodeContext::Proc(node) => &node.key,
        }
    }
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
    pub key: KeyOwned,
    pub config: RosNodeCfg,
    pub param: IndexMap<ParamName, ExprContext>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessContext {
    pub key: KeyOwned,
    pub config: ProcessNodeCfg,
}
