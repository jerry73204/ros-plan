use crate::{
    error::InvalidLinkDeclaration, expr::ValueOrExpr, ident::IdentOwned, key::NonEmptyKeyOwned,
    qos::Qos, ros_type::RosTypeOwned,
};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

pub type LinkIdent = IdentOwned;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedLinkTable", into = "SerializedLinkTable")]
pub struct LinkTable(pub IndexMap<LinkIdent, LinkCfg>);

impl Default for LinkTable {
    fn default() -> Self {
        Self(IndexMap::new())
    }
}

impl TryFrom<SerializedLinkTable> for LinkTable {
    type Error = InvalidLinkDeclaration;

    fn try_from(table: SerializedLinkTable) -> Result<Self, Self::Error> {
        macro_rules! bail {
            ($ident:expr) => {
                return Err(InvalidLinkDeclaration::RepeatedDefinition {
                    name: $ident.to_string(),
                });
            };
        }

        let SerializedLinkTable { pubsub, service } = table;
        let mut map: IndexMap<IdentOwned, LinkCfg> = IndexMap::new();

        for (ident, link) in pubsub {
            let prev = map.insert(ident.clone(), link.into());
            if prev.is_some() {
                bail!(ident);
            }
        }
        for (ident, link) in service {
            let prev = map.insert(ident.clone(), link.into());
            if prev.is_some() {
                bail!(ident);
            }
        }

        Ok(Self(map))
    }
}

impl From<LinkTable> for SerializedLinkTable {
    fn from(table: LinkTable) -> Self {
        let mut pubsub = IndexMap::new();
        let mut service = IndexMap::new();

        for (ident, link) in table.0 {
            match link {
                LinkCfg::Pubsub(link) => {
                    pubsub.insert(ident, link);
                }
                LinkCfg::Service(link) => {
                    service.insert(ident, link);
                }
            }
        }

        Self { pubsub, service }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerializedLinkTable {
    #[serde(default)]
    pub pubsub: IndexMap<LinkIdent, PubSubLinkCfg>,

    #[serde(default)]
    pub service: IndexMap<LinkIdent, ServiceLinkCfg>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LinkCfg {
    #[serde(rename = "pubsub")]
    Pubsub(PubSubLinkCfg),
    #[serde(rename = "service")]
    Service(ServiceLinkCfg),
}

impl From<PubSubLinkCfg> for LinkCfg {
    fn from(v: PubSubLinkCfg) -> Self {
        Self::Pubsub(v)
    }
}

impl From<ServiceLinkCfg> for LinkCfg {
    fn from(v: ServiceLinkCfg) -> Self {
        Self::Service(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PubSubLinkCfg {
    #[serde(rename = "type")]
    pub ty: RosTypeOwned,

    #[serde(default)]
    pub qos: Qos,

    pub src: Vec<TopicUri>,
    pub dst: Vec<TopicUri>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServiceLinkCfg {
    #[serde(rename = "type")]
    pub ty: RosTypeOwned,
    pub listen: TopicUri,
    pub connect: Vec<TopicUri>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicUri {
    pub node: NonEmptyKeyOwned,
    pub topic: ValueOrExpr,
}
