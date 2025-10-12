use super::node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared};
use crate::{
    eval::{BoolStore, KeyStore, TextStore},
    utils::shared_table::{Owned, Shared},
};
use ros_plan_format::{
    interface_type::InterfaceTypeOwned,
    key::{Key, KeyOwned},
    qos::Qos,
};
use serde::{Deserialize, Serialize};

pub type PubSubLinkOwned = Owned<PubSubLinkCtx>;
pub type PubSubLinkShared = Shared<PubSubLinkCtx>;

pub type ServiceLinkOwned = Owned<ServiceLinkCtx>;
pub type ServiceLinkShared = Shared<ServiceLinkCtx>;

#[derive(Debug, Clone)]
pub enum LinkCtx {
    PubSub(PubSubLinkCtx),
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

impl From<PubSubLinkCtx> for LinkCtx {
    fn from(v: PubSubLinkCtx) -> Self {
        Self::PubSub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PubSubLinkCtx {
    pub key: KeyOwned,
    pub ty: InterfaceTypeOwned,
    pub qos: Qos,
    pub when: Option<BoolStore>,
    pub topic: Option<TextStore>,
    pub src_key: Vec<KeyStore>,
    pub dst_key: Vec<KeyStore>,
    pub src_socket: Option<Vec<NodePubShared>>,
    pub dst_socket: Option<Vec<NodeSubShared>>,
    pub derived_topic: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceLinkCtx {
    pub key: KeyOwned,
    pub ty: InterfaceTypeOwned,
    pub when: Option<BoolStore>,
    pub listen_key: KeyStore,
    pub connect_key: Vec<KeyStore>,
    pub listen_socket: Option<NodeSrvShared>,
    pub connect_socket: Option<Vec<NodeCliShared>>,
}
