use super::node::NodeWeak;
use crate::eval::EvalSlot;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct NodeTopicUri {
    pub node: NodeWeak,
    pub topic: EvalSlot,
}
