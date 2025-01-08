use crate::{
    eval::ValueOrEval,
    link::LinkTable,
    node::NodeTable,
    parameter::{ArgEntry, ParamName},
    socket::SocketTable,
    subplan::SubplanTable,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Plan {
    #[serde(default)]
    pub arg: IndexMap<ParamName, ArgEntry>,

    #[serde(default)]
    pub var: IndexMap<ParamName, ValueOrEval>,

    #[serde(default)]
    pub socket: SocketTable,

    #[serde(default)]
    pub node: NodeTable,

    #[serde(default)]
    pub link: LinkTable,

    #[serde(default, flatten)]
    pub subplan: SubplanTable,
}
