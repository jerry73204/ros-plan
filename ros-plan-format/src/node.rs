use crate::{
    error::InvalidNodeDeclaration, expr::ValueOrExpr, ident::IdentOwned, parameter::ParamName,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type NodeIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedNodeTable", into = "SerializedNodeTable")]
pub struct NodeTable(pub IndexMap<NodeIdent, Node>);

impl Default for NodeTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

impl TryFrom<SerializedNodeTable> for NodeTable {
    type Error = InvalidNodeDeclaration;

    fn try_from(table: SerializedNodeTable) -> Result<Self, Self::Error> {
        macro_rules! bail {
            ($ident:expr) => {
                return Err(InvalidNodeDeclaration::RepeatedDefinition {
                    name: $ident.to_string(),
                });
            };
        }

        let SerializedNodeTable { ros, proc } = table;
        let mut map: IndexMap<IdentOwned, Node> = IndexMap::new();

        for (ident, node) in ros {
            let prev = map.insert(ident.clone(), node.into());
            if prev.is_some() {
                bail!(ident);
            }
        }
        for (ident, node) in proc {
            let prev = map.insert(ident.clone(), node.into());
            if prev.is_some() {
                bail!(ident);
            }
        }

        Ok(Self(map))
    }
}

impl From<NodeTable> for SerializedNodeTable {
    fn from(table: NodeTable) -> Self {
        let mut ros = IndexMap::new();
        let mut proc = IndexMap::new();

        for (ident, node) in table.0 {
            match node {
                Node::Ros(node) => {
                    ros.insert(ident, node);
                }
                Node::Proc(node) => {
                    proc.insert(ident, node);
                }
            }
        }

        Self { ros, proc }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerializedNodeTable {
    #[serde(default)]
    pub ros: IndexMap<NodeIdent, RosNode>,

    #[serde(default)]
    pub proc: IndexMap<NodeIdent, ProcessNode>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Node {
    Ros(RosNode),
    Proc(ProcessNode),
}

impl From<ProcessNode> for Node {
    fn from(v: ProcessNode) -> Self {
        Self::Proc(v)
    }
}

impl From<RosNode> for Node {
    fn from(v: RosNode) -> Self {
        Self::Ros(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RosNode {
    pub pkg: String,
    pub exec: Option<String>,
    pub plugin: Option<String>,

    #[serde(default)]
    pub param: IndexMap<ParamName, ValueOrExpr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProcessNode {
    pub command: Vec<String>,
}
