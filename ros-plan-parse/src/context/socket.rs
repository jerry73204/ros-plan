use super::uri::NodeTopicUri;
use crate::utils::{ArcRwLock, WeakRwLock};
use ros_plan_format::socket::{PubSocket, QuerySocket, ServerSocket, SubSocket};
use serde::Serialize;

pub type SocketArc = ArcRwLock<SocketContext>;
pub type SocketWeak = WeakRwLock<SocketContext>;

#[derive(Debug, Clone, Serialize)]
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

#[derive(Debug, Clone, Serialize)]
pub struct PubSocketContext {
    pub config: PubSocket,
    pub src: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SubSocketContext {
    pub config: SubSocket,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ServerSocketContext {
    pub config: ServerSocket,
    pub listen: Option<NodeTopicUri>,
}

#[derive(Debug, Clone, Serialize)]
pub struct QuerySocketContext {
    pub config: QuerySocket,
    pub connect: Option<Vec<NodeTopicUri>>,
}
