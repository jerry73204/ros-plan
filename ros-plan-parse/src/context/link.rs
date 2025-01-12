use super::uri::NodeTopicUri;
use ros_plan_format::link::{PubSubLinkCfg, ServiceLinkCfg};
use serde::Serialize;

#[derive(Debug, Serialize)]
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

#[derive(Debug, Serialize)]
pub struct PubSubLinkContext {
    pub config: PubSubLinkCfg,
    pub src: Option<Vec<NodeTopicUri>>,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Serialize)]
pub struct ServiceLinkContext {
    pub config: ServiceLinkCfg,
    pub listen: Option<NodeTopicUri>,
    pub connect: Option<Vec<NodeTopicUri>>,
}
