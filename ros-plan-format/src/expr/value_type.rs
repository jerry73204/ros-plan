use serde::{Deserialize, Serialize};
use serde_yaml::value::Tag;

use crate::error::DeserializationError;

#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Hash,
    strum::Display,
    strum::EnumString,
    Serialize,
    Deserialize,
)]
pub enum ValueType {
    #[serde(rename = "bool")]
    #[strum(serialize = "bool")]
    Bool,

    #[serde(rename = "i64")]
    #[strum(serialize = "i64")]
    I64,

    #[serde(rename = "f64")]
    #[strum(serialize = "f64")]
    F64,

    #[serde(rename = "str")]
    #[strum(serialize = "str")]
    String,

    #[serde(rename = "bool_list")]
    #[strum(serialize = "bool_list")]
    BoolList,

    #[serde(rename = "binary")]
    #[strum(serialize = "binary")]
    Binary,

    #[serde(rename = "i64_list")]
    #[strum(serialize = "i64_list")]
    I64List,

    #[serde(rename = "f64_list")]
    #[strum(serialize = "f64_list")]
    F64List,

    #[serde(rename = "str_list")]
    #[strum(serialize = "str_list")]
    StringList,
}

impl ValueType {
    pub fn from_yaml_tag(tag: &Tag) -> Result<Self, DeserializationError> {
        tag.to_string()
            .strip_prefix("!")
            .unwrap()
            .parse()
            .map_err(|_| DeserializationError::InvalidTypeTag)
    }
}
