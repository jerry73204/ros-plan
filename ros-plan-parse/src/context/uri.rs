use crate::{context::expr::ExprContext, resource::NodeShared};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeTopicUri {
    pub node: NodeShared,
    pub topic: ExprContext,
}
