use crate::scope::NodeShared;
use ros_plan_format::socket::SocketIdent;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeTopicUri {
    pub node: NodeShared,
    pub topic: SocketIdent,
}
