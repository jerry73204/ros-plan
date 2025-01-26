use super::node_socket::NodeSocketShared;
use crate::utils::shared_table::{Owned, Shared};
use ros_plan_format::{
    key::{Key, KeyOwned},
    link::{PubSubLinkCfg, ServiceLinkCfg},
};
use serde::{Deserialize, Serialize};

pub type LinkOwned = Owned<LinkCtx>;
pub type LinkShared = Shared<LinkCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LinkCtx {
    #[serde(rename = "pubsub")]
    PubSub(PubsubLinkCtx),
    #[serde(rename = "service")]
    Service(ServiceLinkCtx),
}

impl LinkCtx {
    pub fn key(&self) -> &Key {
        match self {
            LinkCtx::PubSub(link) => &link.key,
            LinkCtx::Service(link) => &link.key,
        }
    }
}

impl From<ServiceLinkCtx> for LinkCtx {
    fn from(v: ServiceLinkCtx) -> Self {
        Self::Service(v)
    }
}

impl From<PubsubLinkCtx> for LinkCtx {
    fn from(v: PubsubLinkCtx) -> Self {
        Self::PubSub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PubsubLinkCtx {
    pub key: KeyOwned,
    pub config: PubSubLinkCfg,
    pub src: Option<Vec<NodeSocketShared>>,
    pub dst: Option<Vec<NodeSocketShared>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceLinkCtx {
    pub key: KeyOwned,
    pub config: ServiceLinkCfg,
    pub listen: Option<NodeSocketShared>,
    pub connect: Option<Vec<NodeSocketShared>>,
}
