use super::uri::NodeTopicUri;
use ros_plan_format::link::{PubSubLinkCfg, ServiceLinkCfg};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
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

#[derive(Debug, Serialize, Deserialize)]
pub struct PubSubLinkContext {
    pub config: PubSubLinkCfg,
    pub src: Option<Vec<NodeTopicUri>>,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServiceLinkContext {
    pub config: ServiceLinkCfg,
    pub listen: Option<NodeTopicUri>,
    pub connect: Option<Vec<NodeTopicUri>>,
}
