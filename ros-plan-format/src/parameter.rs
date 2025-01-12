use crate::{
    error::{InvalidParameterDeclaration, ParseParamDefError},
    expr::{Value, ValueType},
};
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::{
    fmt::{self, Display},
    str::FromStr,
    sync::LazyLock,
};

pub static RE_PARAM_NAME: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new("^[[:alpha:]]([[:alnum:]]|-|_)*$").expect("invalid regex for parameter name")
});

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedArgEntry", into = "SerializedArgEntry")]
pub struct ArgEntry {
    pub slot: ArgSlot,
    pub help: Option<String>,
}

#[derive(Debug, Clone)]
pub enum ArgSlot {
    Required { ty: ValueType },
    Optional { default: Value },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SerializedArgEntry {
    pub help: Option<String>,
    pub require: Option<ValueType>,
    pub default: Option<Value>,
}

impl TryFrom<SerializedArgEntry> for ArgEntry {
    type Error = InvalidParameterDeclaration;

    fn try_from(entry: SerializedArgEntry) -> Result<Self, Self::Error> {
        let SerializedArgEntry {
            require: expect,
            default,
            help,
        } = entry;

        let slot = match (expect, default) {
            (None, None) => unreachable!(),
            (None, Some(default)) => ArgSlot::Optional { default },
            (Some(ty), None) => ArgSlot::Required { ty },
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
            ArgSlot::Required { ty } => Self {
                require: Some(ty),
                default: None,
                help,
            },
            ArgSlot::Optional { default } => Self {
                require: None,
                default: Some(default),
                help,
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ParamName(String);

impl FromStr for ParamName {
    type Err = ParseParamDefError;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        let name = name.to_string();

        if RE_PARAM_NAME.is_match(&name) {
            Ok(Self(name))
        } else {
            Err(ParseParamDefError::InvalidName { name })
        }
    }
}

impl AsRef<str> for ParamName {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

impl Display for ParamName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}
