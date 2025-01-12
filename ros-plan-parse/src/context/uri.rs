use super::node::NodeWeak;
use crate::context::expr::ExprContext;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct NodeTopicUri {
    pub node: NodeWeak,
    pub topic: ExprContext,
}
