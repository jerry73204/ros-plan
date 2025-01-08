use crate::utils::{ArcRwLock, WeakRwLock};
use ros_plan_format::node::Node;
use serde::Serialize;

pub type NodeArc = ArcRwLock<NodeContext>;
pub type NodeWeak = WeakRwLock<NodeContext>;

#[derive(Debug, Clone, Serialize)]
pub struct NodeContext {
    pub config: Node,
}
