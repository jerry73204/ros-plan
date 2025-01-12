use crate::{
    eval::EvalSlot,
    utils::{ArcRwLock, WeakRwLock},
};
use indexmap::IndexMap;
use ros_plan_format::{
    node::{ProcessNode, RosNode},
    parameter::ParamName,
};
use serde::Serialize;

pub type NodeArc = ArcRwLock<NodeContext>;
pub type NodeWeak = WeakRwLock<NodeContext>;

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
    pub config: RosNode,
    pub param: IndexMap<ParamName, EvalSlot>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ProcessContext {
    pub config: ProcessNode,
}
