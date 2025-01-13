use super::uri::NodeTopicUri;
use ros_plan_format::{
    key::{Key, KeyOwned},
    socket::{PubSocketCfg, QuerySocketCfg, ServerSocketCfg, SubSocketCfg},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SocketContext {
    Pub(PubSocketContext),
    Sub(SubSocketContext),
    Srv(ServerSocketContext),
    Qry(QuerySocketContext),
}

impl SocketContext {
    pub fn key(&self) -> &Key {
        match self {
            SocketContext::Pub(socket) => &socket.key,
            SocketContext::Sub(socket) => &socket.key,
            SocketContext::Srv(socket) => &socket.key,
            SocketContext::Qry(socket) => &socket.key,
        }
    }
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
    pub key: KeyOwned,
    pub config: PubSocketCfg,
    pub src: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubSocketContext {
    pub key: KeyOwned,
    pub config: SubSocketCfg,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerSocketContext {
    pub key: KeyOwned,
    pub config: ServerSocketCfg,
    pub listen: Option<NodeTopicUri>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QuerySocketContext {
    pub key: KeyOwned,
    pub config: QuerySocketCfg,
    pub connect: Option<Vec<NodeTopicUri>>,
}
