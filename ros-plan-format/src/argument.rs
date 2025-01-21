use crate::{
    error::InvalidParameterDeclaration,
    expr::{Value, ValueType},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedArgEntry", into = "SerializedArgEntry")]
pub struct ArgEntry {
    pub slot: ArgCfg,
    pub help: Option<String>,
}

#[derive(Debug, Clone)]
pub enum ArgCfg {
    Required { ty: ValueType },
    Optional { default: Value },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SerializedArgEntry {
    pub help: Option<String>,
    #[serde(rename = "type")]
    pub ty: Option<ValueType>,
    pub default: Option<Value>,
}

impl TryFrom<SerializedArgEntry> for ArgEntry {
    type Error = InvalidParameterDeclaration;

    fn try_from(entry: SerializedArgEntry) -> Result<Self, Self::Error> {
        let SerializedArgEntry {
            ty: expect,
            default,
            help,
        } = entry;

        let slot = match (expect, default) {
            (None, None) => unreachable!(),
            (None, Some(default)) => ArgCfg::Optional { default },
            (Some(ty), None) => ArgCfg::Required { ty },
            (Some(_), Some(_)) => {
                return Err(InvalidParameterDeclaration::InvalidArgumentDefinition)
            }
        };
        let entry = ArgEntry { slot, help };

        Ok(entry)
    }
}

impl From<ArgEntry> for SerializedArgEntry {
    fn from(entry: ArgEntry) -> Self {
        let ArgEntry { slot, help } = entry;

        match slot {
            ArgCfg::Required { ty } => Self {
                ty: Some(ty),
                default: None,
                help,
            },
            ArgCfg::Optional { default } => Self {
                ty: None,
                default: Some(default),
                help,
            },
        }
    }
}
