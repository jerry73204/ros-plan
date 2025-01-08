use super::node::NodeWeak;
use ros_plan_format::eval::ValueOrEval;
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct NodeTopicUri {
    pub node: NodeWeak,
    pub topic: ValueOrEval,
}
