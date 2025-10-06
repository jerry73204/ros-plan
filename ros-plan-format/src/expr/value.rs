use super::{Base64String, ValueType};
use crate::{
    error::ParseYamlError,
    key::{Key, KeyOwned},
};
use base64::prelude::{BASE64_STANDARD, *};
use serde::{Deserialize, Serialize};
use std::{
    fmt::{self, Debug, Display},
    path::{Path, PathBuf},
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Value {
    #[serde(rename = "bool")]
    Bool(bool),

    #[serde(rename = "i64")]
    I64(i64),

    #[serde(rename = "f64")]
    F64(f64),

    #[serde(rename = "str")]
    String(String),

    #[serde(rename = "key")]
    Key(KeyOwned),

    #[serde(rename = "path")]
    Path(PathBuf),

    #[serde(rename = "bool_list")]
    BoolList(Vec<bool>),

    #[serde(rename = "i64_list")]
    I64List(Vec<i64>),

    #[serde(rename = "f64_list")]
    F64List(Vec<f64>),

    #[serde(rename = "str_list")]
    StringList(Vec<String>),

    #[serde(rename = "binary")]
    Binary(Base64String),
}

impl Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(val) => Debug::fmt(val, f),
            Value::I64(val) => Debug::fmt(val, f),
            Value::F64(val) => Debug::fmt(val, f),
            Value::String(val) => Debug::fmt(val, f),
            Value::Key(val) => Debug::fmt(val, f),
            Value::Path(val) => Debug::fmt(val, f),
            Value::BoolList(vec) => Debug::fmt(vec, f),
            Value::I64List(vec) => Debug::fmt(vec, f),
            Value::F64List(vec) => Debug::fmt(vec, f),
            Value::StringList(vec) => Debug::fmt(vec, f),
            Value::Binary(bytes) => Debug::fmt(bytes, f),
        }
    }
}

impl From<Vec<String>> for Value {
    fn from(v: Vec<String>) -> Self {
        Self::StringList(v)
    }
}

impl From<Vec<f64>> for Value {
    fn from(v: Vec<f64>) -> Self {
        Self::F64List(v)
    }
}

impl From<Vec<i64>> for Value {
    fn from(v: Vec<i64>) -> Self {
        Self::I64List(v)
    }
}

impl From<Vec<u8>> for Value {
    fn from(v: Vec<u8>) -> Self {
        Self::Binary(Base64String(v))
    }
}

impl From<Vec<bool>> for Value {
    fn from(v: Vec<bool>) -> Self {
        Self::BoolList(v)
    }
}

impl From<String> for Value {
    fn from(v: String) -> Self {
        Self::String(v)
    }
}

impl From<f64> for Value {
    fn from(v: f64) -> Self {
        Self::F64(v)
    }
}

impl From<i64> for Value {
    fn from(v: i64) -> Self {
        Self::I64(v)
    }
}

impl From<bool> for Value {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}

impl From<KeyOwned> for Value {
    fn from(v: KeyOwned) -> Self {
        Self::Key(v)
    }
}

impl From<PathBuf> for Value {
    fn from(v: PathBuf) -> Self {
        Self::Path(v)
    }
}

impl Value {
    pub fn from_yaml_value(
        ty_hint: Option<ValueType>,
        yaml_value: serde_yaml::Value,
    ) -> Result<Self, ParseYamlError> {
        macro_rules! assert_type {
            ($expect:ident) => {
                if !matches!(ty_hint, None | Some(ValueType::$expect)) {
                    todo!()
                }
            };
        }

        let value: Self = match yaml_value {
            serde_yaml::Value::Bool(val) => {
                assert_type!(Bool);
                val.into()
            }
            serde_yaml::Value::Number(val) => {
                if let Some(val) = val.as_i64() {
                    assert_type!(I64);
                    val.into()
                } else {
                    assert_type!(F64);
                    let val = val.as_f64().unwrap();
                    val.into()
                }
            }
            serde_yaml::Value::String(text) => match ty_hint {
                None | Some(ValueType::String) => Self::String(text),
                Some(ValueType::Binary) => {
                    let Ok(bytes) = BASE64_STANDARD.decode(&text) else {
                        todo!()
                    };
                    Self::Binary(bytes.into())
                }
                _ => todo!(),
            },
            serde_yaml::Value::Sequence(vec) => {
                if let Some(first) = vec.first() {
                    match first {
                        serde_yaml::Value::Bool(_) => {
                            let vec: Option<Vec<bool>> =
                                vec.into_iter().map(|val| val.as_bool()).collect();
                            let Some(vec) = vec else { todo!() };
                            Self::BoolList(vec)
                        }
                        serde_yaml::Value::Number(first) => {
                            if first.is_i64() {
                                let vec: Option<Vec<i64>> =
                                    vec.into_iter().map(|val| val.as_i64()).collect();
                                let Some(vec) = vec else { todo!() };
                                Self::I64List(vec)
                            } else {
                                assert!(first.is_f64());
                                let vec: Option<Vec<f64>> =
                                    vec.into_iter().map(|val| val.as_f64()).collect();
                                let Some(vec) = vec else { todo!() };
                                Self::F64List(vec)
                            }
                        }
                        serde_yaml::Value::String(_) => {
                            let vec: Option<Vec<String>> = vec
                                .into_iter()
                                .map(|val| Some(val.as_str()?.to_string()))
                                .collect();
                            let Some(vec) = vec else { todo!() };
                            Self::StringList(vec)
                        }
                        _ => todo!(),
                    }
                } else {
                    let Some(ty) = ty_hint else {
                        todo!();
                    };

                    match ty {
                        ValueType::BoolList => Self::BoolList(vec![]),
                        ValueType::I64List => Self::I64List(vec![]),
                        ValueType::F64List => Self::F64List(vec![]),
                        ValueType::StringList => Self::StringList(vec![]),
                        _ => todo!(),
                    }
                }
            }
            _ => todo!(),
        };
        Ok(value)
    }

    pub fn ty(&self) -> ValueType {
        match self {
            Value::Bool(_) => ValueType::Bool,
            Value::I64(_) => ValueType::I64,
            Value::F64(_) => ValueType::F64,
            Value::String(_) => ValueType::String,
            Value::Key(_) => ValueType::Key,
            Value::Path(_) => ValueType::Path,
            Value::BoolList(_) => ValueType::BoolList,
            Value::Binary { .. } => ValueType::Binary,
            Value::I64List(_) => ValueType::I64List,
            Value::F64List(_) => ValueType::F64List,
            Value::StringList(_) => ValueType::StringList,
        }
    }

    pub fn to_bool(&self) -> Option<bool> {
        if let Self::Bool(v) = self {
            Some(*v)
        } else {
            None
        }
    }

    pub fn to_i64(&self) -> Option<i64> {
        if let Self::I64(v) = self {
            Some(*v)
        } else {
            None
        }
    }

    pub fn to_f64(&self) -> Option<f64> {
        if let Self::F64(v) = self {
            Some(*v)
        } else {
            None
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        if let Self::String(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_key(&self) -> Option<&Key> {
        if let Self::Key(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_path(&self) -> Option<&Path> {
        if let Self::Path(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_bytes(&self) -> Option<&[u8]> {
        if let Self::Binary(bytes) = self {
            Some(bytes.as_slice())
        } else {
            None
        }
    }

    pub fn as_bool_slice(&self) -> Option<&[bool]> {
        if let Self::BoolList(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_i64_slice(&self) -> Option<&[i64]> {
        if let Self::I64List(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_f64_slice(&self) -> Option<&[f64]> {
        if let Self::F64List(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_string_slice(&self) -> Option<&[String]> {
        if let Self::StringList(v) = self {
            Some(v)
        } else {
            None
        }
    }

    #[must_use]
    pub fn is_bool(&self) -> bool {
        matches!(self, Self::Bool(..))
    }

    #[must_use]
    pub fn is_i64(&self) -> bool {
        matches!(self, Self::I64(..))
    }

    #[must_use]
    pub fn is_f64(&self) -> bool {
        matches!(self, Self::F64(..))
    }

    #[must_use]
    pub fn is_string(&self) -> bool {
        matches!(self, Self::String(..))
    }

    #[must_use]
    pub fn is_key(&self) -> bool {
        matches!(self, Self::Key(..))
    }

    #[must_use]
    pub fn is_path(&self) -> bool {
        matches!(self, Self::Path(..))
    }

    #[must_use]
    pub fn is_bool_list(&self) -> bool {
        matches!(self, Self::BoolList(..))
    }

    #[must_use]
    pub fn is_i64_list(&self) -> bool {
        matches!(self, Self::I64List(..))
    }

    #[must_use]
    pub fn is_f64_list(&self) -> bool {
        matches!(self, Self::F64List(..))
    }

    #[must_use]
    pub fn is_string_list(&self) -> bool {
        matches!(self, Self::StringList(..))
    }

    #[must_use]
    pub fn is_bytes(&self) -> bool {
        matches!(self, Self::Binary { .. })
    }

    pub fn try_into_string(self) -> Result<String, Self> {
        if let Self::String(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_key(self) -> Result<KeyOwned, Self> {
        if let Self::Key(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_path(self) -> Result<PathBuf, Self> {
        if let Self::Path(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_bool_vec(self) -> Result<Vec<bool>, Self> {
        if let Self::BoolList(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_i64_vec(self) -> Result<Vec<i64>, Self> {
        if let Self::I64List(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_f64_vec(self) -> Result<Vec<f64>, Self> {
        if let Self::F64List(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_string_vec(self) -> Result<Vec<String>, Self> {
        if let Self::StringList(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_bytes(self) -> Result<Vec<u8>, Self> {
        if let Self::Binary(v) = self {
            Ok(v.0)
        } else {
            Err(self)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_type_checking() {
        let val_bool = Value::Bool(true);
        assert!(val_bool.is_bool());
        assert!(!val_bool.is_i64());
        assert_eq!(val_bool.ty(), ValueType::Bool);

        let val_i64 = Value::I64(42);
        assert!(val_i64.is_i64());
        assert!(!val_i64.is_bool());
        assert_eq!(val_i64.ty(), ValueType::I64);

        let val_f64 = Value::F64(2.5);
        assert!(val_f64.is_f64());
        assert!(!val_f64.is_i64());
        assert_eq!(val_f64.ty(), ValueType::F64);

        let val_string = Value::String("test".to_string());
        assert!(val_string.is_string());
        assert!(!val_string.is_key());
        assert_eq!(val_string.ty(), ValueType::String);
    }

    #[test]
    fn value_conversions_success() {
        let val_bool = Value::Bool(true);
        assert_eq!(val_bool.to_bool(), Some(true));

        let val_i64 = Value::I64(42);
        assert_eq!(val_i64.to_i64(), Some(42));

        let val_f64 = Value::F64(2.5);
        assert_eq!(val_f64.to_f64(), Some(2.5));

        let val_string = Value::String("hello".to_string());
        assert_eq!(val_string.as_str(), Some("hello"));
    }

    #[test]
    fn value_conversions_failure() {
        let val_bool = Value::Bool(true);
        assert_eq!(val_bool.to_i64(), None);
        assert_eq!(val_bool.to_f64(), None);
        assert_eq!(val_bool.as_str(), None);

        let val_i64 = Value::I64(42);
        assert_eq!(val_i64.to_bool(), None);
        assert_eq!(val_i64.to_f64(), None);
    }

    #[test]
    fn value_list_checking() {
        let bool_list = Value::BoolList(vec![true, false]);
        assert!(bool_list.is_bool_list());
        assert!(!bool_list.is_i64_list());
        assert_eq!(bool_list.ty(), ValueType::BoolList);

        let i64_list = Value::I64List(vec![1, 2, 3]);
        assert!(i64_list.is_i64_list());
        assert!(!i64_list.is_f64_list());
        assert_eq!(i64_list.ty(), ValueType::I64List);

        let f64_list = Value::F64List(vec![1.0, 2.0, 3.0]);
        assert!(f64_list.is_f64_list());
        assert!(!f64_list.is_string_list());
        assert_eq!(f64_list.ty(), ValueType::F64List);

        let string_list = Value::StringList(vec!["a".to_string(), "b".to_string()]);
        assert!(string_list.is_string_list());
        assert!(!string_list.is_bool_list());
        assert_eq!(string_list.ty(), ValueType::StringList);
    }

    #[test]
    fn value_list_conversions() {
        let bool_list = Value::BoolList(vec![true, false, true]);
        assert_eq!(bool_list.as_bool_slice(), Some(&[true, false, true][..]));

        let i64_list = Value::I64List(vec![1, 2, 3]);
        assert_eq!(i64_list.as_i64_slice(), Some(&[1, 2, 3][..]));

        let f64_list = Value::F64List(vec![1.5, 2.5]);
        assert_eq!(f64_list.as_f64_slice(), Some(&[1.5, 2.5][..]));

        let string_list = Value::StringList(vec!["x".to_string(), "y".to_string()]);
        let expected = ["x".to_string(), "y".to_string()];
        assert_eq!(string_list.as_string_slice(), Some(&expected[..]));
    }

    #[test]
    fn value_from_primitives() {
        let from_bool: Value = true.into();
        assert!(from_bool.is_bool());
        assert_eq!(from_bool.to_bool(), Some(true));

        let from_i64: Value = 42i64.into();
        assert!(from_i64.is_i64());
        assert_eq!(from_i64.to_i64(), Some(42));

        let from_f64: Value = 2.5f64.into();
        assert!(from_f64.is_f64());
        assert_eq!(from_f64.to_f64(), Some(2.5));

        let from_string: Value = "test".to_string().into();
        assert!(from_string.is_string());
        assert_eq!(from_string.as_str(), Some("test"));
    }

    #[test]
    fn value_from_vecs() {
        let from_bool_vec: Value = vec![true, false].into();
        assert!(from_bool_vec.is_bool_list());

        let from_i64_vec: Value = vec![1i64, 2, 3].into();
        assert!(from_i64_vec.is_i64_list());

        let from_f64_vec: Value = vec![1.0f64, 2.0].into();
        assert!(from_f64_vec.is_f64_list());

        let from_string_vec: Value = vec!["a".to_string(), "b".to_string()].into();
        assert!(from_string_vec.is_string_list());

        let from_bytes: Value = vec![0u8, 1, 2].into();
        assert!(from_bytes.is_bytes());
    }

    #[test]
    fn value_try_into_success() {
        let val_string = Value::String("hello".to_string());
        let result = val_string.try_into_string();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), "hello".to_string());

        let val_i64_list = Value::I64List(vec![1, 2, 3]);
        let result = val_i64_list.try_into_i64_vec();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec![1, 2, 3]);

        let val_bool_list = Value::BoolList(vec![true, false]);
        let result = val_bool_list.try_into_bool_vec();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), vec![true, false]);
    }

    #[test]
    fn value_try_into_failure() {
        let val_bool = Value::Bool(true);
        assert!(val_bool.clone().try_into_string().is_err());
        assert!(val_bool.try_into_i64_vec().is_err());

        let val_i64 = Value::I64(42);
        assert!(val_i64.try_into_bool_vec().is_err());
    }

    #[test]
    fn value_deserialize_bool() {
        let yaml = "!bool true";
        let result: Result<Value, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value = result.unwrap();
        assert_eq!(value.to_bool(), Some(true));
    }

    #[test]
    fn value_deserialize_i64() {
        let yaml = "!i64 42";
        let result: Result<Value, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value = result.unwrap();
        assert_eq!(value.to_i64(), Some(42));
    }

    #[test]
    fn value_deserialize_f64() {
        let yaml = "!f64 2.5";
        let result: Result<Value, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value = result.unwrap();
        assert_eq!(value.to_f64(), Some(2.5));
    }

    #[test]
    fn value_deserialize_string() {
        let yaml = "!str \"hello world\"";
        let result: Result<Value, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value = result.unwrap();
        assert_eq!(value.as_str(), Some("hello world"));
    }

    #[test]
    fn value_deserialize_i64_list() {
        let yaml = "!i64_list [1, 2, 3]";
        let result: Result<Value, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value = result.unwrap();
        assert_eq!(value.as_i64_slice(), Some(&[1, 2, 3][..]));
    }

    #[test]
    fn value_deserialize_string_list() {
        let yaml = "!str_list [\"a\", \"b\", \"c\"]";
        let result: Result<Value, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value = result.unwrap();
        let expected = ["a".to_string(), "b".to_string(), "c".to_string()];
        assert_eq!(value.as_string_slice(), Some(&expected[..]));
    }

    #[test]
    fn value_display_primitives() {
        assert_eq!(Value::Bool(true).to_string(), "true");
        assert_eq!(Value::I64(42).to_string(), "42");
        assert_eq!(Value::F64(2.5).to_string(), "2.5");
        assert_eq!(Value::String("test".to_string()).to_string(), "\"test\"");
    }

    #[test]
    fn value_display_lists() {
        let bool_list = Value::BoolList(vec![true, false]);
        assert_eq!(bool_list.to_string(), "[true, false]");

        let i64_list = Value::I64List(vec![1, 2, 3]);
        assert_eq!(i64_list.to_string(), "[1, 2, 3]");
    }

    #[test]
    fn value_key_type() {
        let key: KeyOwned = "node/topic".parse().unwrap();
        let val_key: Value = key.into();
        assert!(val_key.is_key());
        assert_eq!(val_key.ty(), ValueType::Key);
        assert!(val_key.as_key().is_some());
    }

    #[test]
    fn value_path_type() {
        let path = PathBuf::from("/tmp/test");
        let val_path: Value = path.into();
        assert!(val_path.is_path());
        assert_eq!(val_path.ty(), ValueType::Path);
        assert!(val_path.as_path().is_some());
    }

    #[test]
    fn value_binary_type() {
        let bytes = vec![0u8, 1, 2, 3];
        let val_binary: Value = bytes.clone().into();
        assert!(val_binary.is_bytes());
        assert_eq!(val_binary.ty(), ValueType::Binary);
        assert_eq!(val_binary.as_bytes(), Some(&bytes[..]));
    }
}
