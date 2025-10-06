use super::node_socket::{NodeCliShared, NodePubShared, NodeSrvShared, NodeSubShared};
use crate::{
    eval::{TextStore, ValueStore},
    utils::shared_table::{Owned, Shared},
};
use indexmap::IndexMap;
use ros_plan_format::{key::KeyOwned, node_socket::NodeSocketIdent, parameter::ParamName};
use serde::{Deserialize, Serialize};

pub type NodeOwned = Owned<NodeCtx>;
pub type NodeShared = Shared<NodeCtx>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeCtx {
    pub key: KeyOwned,
    pub namespace: Option<String>,
    pub pkg: Option<TextStore>,
    pub exec: Option<TextStore>,
    pub plugin: Option<TextStore>,
    pub param: IndexMap<ParamName, ValueStore>,
    #[serde(rename = "pub")]
    pub pub_: IndexMap<NodeSocketIdent, NodePubShared>,
    pub sub: IndexMap<NodeSocketIdent, NodeSubShared>,
    pub srv: IndexMap<NodeSocketIdent, NodeSrvShared>,
    pub cli: IndexMap<NodeSocketIdent, NodeCliShared>,
}
