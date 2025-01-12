use crate::{error::InvalidSocketDeclaration, ident::IdentOwned, link::TopicUri};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type SocketIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedSocketTable", into = "SerializedSocketTable")]
pub struct SocketTable(pub IndexMap<SocketIdent, SocketCfg>);

impl Default for SocketTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

impl TryFrom<SerializedSocketTable> for SocketTable {
    type Error = InvalidSocketDeclaration;

    fn try_from(table: SerializedSocketTable) -> Result<Self, Self::Error> {
        macro_rules! bail {
            ($ident:expr) => {
                return Err(InvalidSocketDeclaration::RepeatedDefinition {
                    name: $ident.to_string(),
                });
            };
        }

        let SerializedSocketTable {
            pub_,
            sub,
            srv,
            qry,
        } = table;
        let mut map: IndexMap<IdentOwned, SocketCfg> = IndexMap::new();

        for (ident, socket) in pub_ {
            let prev = map.insert(ident.clone(), socket.into());
            if prev.is_some() {
                bail!(ident);
            }
        }
        for (ident, socket) in sub {
            let prev = map.insert(ident.clone(), socket.into());
            if prev.is_some() {
                bail!(ident);
            }
        }
        for (ident, socket) in srv {
            let prev = map.insert(ident.clone(), socket.into());
            if prev.is_some() {
                bail!(ident);
            }
        }
        for (ident, socket) in qry {
            let prev = map.insert(ident.clone(), socket.into());
            if prev.is_some() {
                bail!(ident);
            }
        }

        Ok(Self(map))
    }
}

impl From<SocketTable> for SerializedSocketTable {
    fn from(table: SocketTable) -> Self {
        let mut pub_ = IndexMap::new();
        let mut sub = IndexMap::new();
        let mut srv = IndexMap::new();
        let mut qry = IndexMap::new();

        for (ident, socket) in table.0 {
            match socket {
                SocketCfg::Pub(socket) => {
                    pub_.insert(ident, socket);
                }
                SocketCfg::Sub(socket) => {
                    sub.insert(ident, socket);
                }
                SocketCfg::Srv(socket) => {
                    srv.insert(ident, socket);
                }
                SocketCfg::Qry(socket) => {
                    qry.insert(ident, socket);
                }
            }
        }

        Self {
            pub_,
            sub,
            srv,
            qry,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerializedSocketTable {
    #[serde(default, rename = "pub")]
    pub pub_: IndexMap<SocketIdent, PubSocketCfg>,

    #[serde(default)]
    pub sub: IndexMap<SocketIdent, SubSocketCfg>,

    #[serde(default)]
    pub srv: IndexMap<SocketIdent, ServerSocketCfg>,

    #[serde(default)]
    pub qry: IndexMap<SocketIdent, QuerySocketCfg>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SocketCfg {
    Pub(PubSocketCfg),
    Sub(SubSocketCfg),
    Srv(ServerSocketCfg),
    Qry(QuerySocketCfg),
}

impl From<QuerySocketCfg> for SocketCfg {
    fn from(v: QuerySocketCfg) -> Self {
        Self::Qry(v)
    }
}

impl From<ServerSocketCfg> for SocketCfg {
    fn from(v: ServerSocketCfg) -> Self {
        Self::Srv(v)
    }
}

impl From<SubSocketCfg> for SocketCfg {
    fn from(v: SubSocketCfg) -> Self {
        Self::Sub(v)
    }
}

impl From<PubSocketCfg> for SocketCfg {
    fn from(v: PubSocketCfg) -> Self {
        Self::Pub(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PubSocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    // pub qos: Option<QosRequirement>,
    pub src: Vec<TopicUri>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SubSocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    // pub qos: Option<QosRequirement>,
    pub dst: Vec<TopicUri>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServerSocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    pub listen: TopicUri,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QuerySocketCfg {
    // #[serde(rename = "type")]
    // pub ty: RosTypeOwned,
    pub connect: Vec<TopicUri>,
}
