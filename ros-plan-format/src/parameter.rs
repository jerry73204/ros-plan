use crate::{
    error::{InvalidParameterDeclaration, ParseParamDefError},
    eval::{Value, ValueType},
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
pub enum ArgEntry {
    Required {
        ty: ValueType,
        help: Option<String>,
    },
    Optional {
        default: Value,
        help: Option<String>,
    },
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

        let entry = match (expect, default) {
            (None, None) => unreachable!(),
            (None, Some(default)) => ArgEntry::Optional { default, help },
            (Some(ty), None) => ArgEntry::Required { ty, help },
            (Some(_), Some(_)) => todo!(),
        };

        Ok(entry)
    }
}

impl From<ArgEntry> for SerializedArgEntry {
    fn from(entry: ArgEntry) -> Self {
        match entry {
            ArgEntry::Required { ty, help } => Self {
                require: Some(ty),
                default: None,
                help,
            },
            ArgEntry::Optional { default, help } => Self {
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
