use crate::utils::{ArcRwLock, WeakRwLock};
use ros_plan_format::link::{PubSubLinkCfg, ServiceLinkCfg};
use serde::Serialize;

use super::uri::NodeTopicUri;

pub type LinkArc = ArcRwLock<LinkContext>;
pub type LinkWeak = WeakRwLock<LinkContext>;

#[derive(Debug, Clone, Serialize)]
pub enum LinkContext {
    Pubsub(PubSubLinkContext),
    Service(ServiceLinkContext),
}

impl From<ServiceLinkContext> for LinkContext {
    fn from(v: ServiceLinkContext) -> Self {
        Self::Service(v)
    }
}

impl From<PubSubLinkContext> for LinkContext {
    fn from(v: PubSubLinkContext) -> Self {
        Self::Pubsub(v)
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct PubSubLinkContext {
    pub config: PubSubLinkCfg,
    pub src: Option<Vec<NodeTopicUri>>,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ServiceLinkContext {
    pub config: ServiceLinkCfg,
    pub listen: Option<NodeTopicUri>,
    pub connect: Option<Vec<NodeTopicUri>>,
}
