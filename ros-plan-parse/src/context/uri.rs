use crate::{context::expr::ExprContext, resource::NodeShared};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct NodeTopicUri {
    pub node: NodeShared,
    pub topic: ExprContext,
}
