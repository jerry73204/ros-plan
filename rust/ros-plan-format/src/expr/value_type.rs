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

    #[serde(rename = "key")]
    #[strum(serialize = "key")]
    Key,

    #[serde(rename = "path")]
    #[strum(serialize = "path")]
    Path,

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_type_display() {
        assert_eq!(ValueType::Bool.to_string(), "bool");
        assert_eq!(ValueType::I64.to_string(), "i64");
        assert_eq!(ValueType::F64.to_string(), "f64");
        assert_eq!(ValueType::String.to_string(), "str");
        assert_eq!(ValueType::Key.to_string(), "key");
        assert_eq!(ValueType::Path.to_string(), "path");
        assert_eq!(ValueType::BoolList.to_string(), "bool_list");
        assert_eq!(ValueType::Binary.to_string(), "binary");
        assert_eq!(ValueType::I64List.to_string(), "i64_list");
        assert_eq!(ValueType::F64List.to_string(), "f64_list");
        assert_eq!(ValueType::StringList.to_string(), "str_list");
    }

    #[test]
    fn value_type_from_string() {
        assert_eq!("bool".parse::<ValueType>().unwrap(), ValueType::Bool);
        assert_eq!("i64".parse::<ValueType>().unwrap(), ValueType::I64);
        assert_eq!("f64".parse::<ValueType>().unwrap(), ValueType::F64);
        assert_eq!("str".parse::<ValueType>().unwrap(), ValueType::String);
        assert_eq!("key".parse::<ValueType>().unwrap(), ValueType::Key);
        assert_eq!("path".parse::<ValueType>().unwrap(), ValueType::Path);
        assert_eq!(
            "bool_list".parse::<ValueType>().unwrap(),
            ValueType::BoolList
        );
        assert_eq!("binary".parse::<ValueType>().unwrap(), ValueType::Binary);
        assert_eq!("i64_list".parse::<ValueType>().unwrap(), ValueType::I64List);
        assert_eq!("f64_list".parse::<ValueType>().unwrap(), ValueType::F64List);
        assert_eq!(
            "str_list".parse::<ValueType>().unwrap(),
            ValueType::StringList
        );
    }

    #[test]
    fn value_type_from_yaml_tag() {
        let tag = Tag::new("bool");
        assert_eq!(ValueType::from_yaml_tag(&tag).unwrap(), ValueType::Bool);

        let tag = Tag::new("i64");
        assert_eq!(ValueType::from_yaml_tag(&tag).unwrap(), ValueType::I64);

        let tag = Tag::new("str");
        assert_eq!(ValueType::from_yaml_tag(&tag).unwrap(), ValueType::String);

        let tag = Tag::new("key");
        assert_eq!(ValueType::from_yaml_tag(&tag).unwrap(), ValueType::Key);
    }

    #[test]
    fn value_type_from_invalid_tag() {
        let tag = Tag::new("invalid_type");
        assert!(ValueType::from_yaml_tag(&tag).is_err());
    }

    #[test]
    fn value_type_equality() {
        assert_eq!(ValueType::Bool, ValueType::Bool);
        assert_ne!(ValueType::Bool, ValueType::I64);
        assert_ne!(ValueType::I64, ValueType::F64);
    }

    #[test]
    fn value_type_serialize() {
        let yaml = serde_yaml::to_string(&ValueType::Bool).unwrap();
        assert_eq!(yaml.trim(), "bool");

        let yaml = serde_yaml::to_string(&ValueType::I64).unwrap();
        assert_eq!(yaml.trim(), "i64");

        let yaml = serde_yaml::to_string(&ValueType::String).unwrap();
        assert_eq!(yaml.trim(), "str");
    }

    #[test]
    fn value_type_deserialize() {
        let result: Result<ValueType, _> = serde_yaml::from_str("bool");
        assert_eq!(result.unwrap(), ValueType::Bool);

        let result: Result<ValueType, _> = serde_yaml::from_str("i64");
        assert_eq!(result.unwrap(), ValueType::I64);

        let result: Result<ValueType, _> = serde_yaml::from_str("str");
        assert_eq!(result.unwrap(), ValueType::String);
    }
}
