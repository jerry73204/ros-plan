use crate::error::ParseParamDefError;
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

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ParamName(String);

impl ParamName {
    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

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
