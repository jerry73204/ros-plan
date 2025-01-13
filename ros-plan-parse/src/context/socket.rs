use super::uri::NodeTopicUri;
use ros_plan_format::socket::{PubSocketCfg, QuerySocketCfg, ServerSocketCfg, SubSocketCfg};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SocketContext {
    Pub(PubSocketContext),
    Sub(SubSocketContext),
    Srv(ServerSocketContext),
    Qry(QuerySocketContext),
}

impl From<QuerySocketContext> for SocketContext {
    fn from(v: QuerySocketContext) -> Self {
        Self::Qry(v)
    }
}

impl From<ServerSocketContext> for SocketContext {
    fn from(v: ServerSocketContext) -> Self {
        Self::Srv(v)
    }
}

impl From<SubSocketContext> for SocketContext {
    fn from(v: SubSocketContext) -> Self {
        Self::Sub(v)
    }
}

impl From<PubSocketContext> for SocketContext {
    fn from(v: PubSocketContext) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PubSocketContext {
    pub config: PubSocketCfg,
    pub src: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubSocketContext {
    pub config: SubSocketCfg,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerSocketContext {
    pub config: ServerSocketCfg,
    pub listen: Option<NodeTopicUri>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySocketContext {
    pub config: QuerySocketCfg,
    pub connect: Option<Vec<NodeTopicUri>>,
}
