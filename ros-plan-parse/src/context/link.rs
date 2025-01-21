use super::uri::NodeTopicUri;
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::{PubSubLinkCfg, ServiceLinkCfg},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub enum LinkContext {
    #[serde(rename = "pubsub")]
    PubSub(PubsubLinkContext),
    #[serde(rename = "service")]
    Service(ServiceLinkContext),
}

impl LinkContext {
    pub fn key(&self) -> &Key {
        match self {
            LinkContext::PubSub(link) => &link.key,
            LinkContext::Service(link) => &link.key,
        }
    }
}

impl From<ServiceLinkContext> for LinkContext {
    fn from(v: ServiceLinkContext) -> Self {
        Self::Service(v)
    }
}

impl From<PubsubLinkContext> for LinkContext {
    fn from(v: PubsubLinkContext) -> Self {
        Self::PubSub(v)
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PubsubLinkContext {
    pub key: KeyOwned,
    pub config: PubSubLinkCfg,
    pub src: Option<Vec<NodeTopicUri>>,
    pub dst: Option<Vec<NodeTopicUri>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServiceLinkContext {
    pub key: KeyOwned,
    pub config: ServiceLinkCfg,
    pub listen: Option<NodeTopicUri>,
    pub connect: Option<Vec<NodeTopicUri>>,
}
