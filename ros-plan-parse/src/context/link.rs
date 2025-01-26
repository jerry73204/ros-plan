use crate::utils::shared_table::{Owned, Shared};
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::{PubSubLinkCfg, ServiceLinkCfg},
};
use serde::{Deserialize, Serialize};

use super::node_socket::NodeSocketShared;

pub type LinkOwned = Owned<LinkContext>;
pub type LinkShared = Shared<LinkContext>;

#[derive(Debug, Clone, Serialize, Deserialize)]
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PubsubLinkContext {
    pub key: KeyOwned,
    pub config: PubSubLinkCfg,
    pub src: Option<Vec<NodeSocketShared>>,
    pub dst: Option<Vec<NodeSocketShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceLinkContext {
    pub key: KeyOwned,
    pub config: ServiceLinkCfg,
    pub listen: Option<NodeSocketShared>,
    pub connect: Option<Vec<NodeSocketShared>>,
}
