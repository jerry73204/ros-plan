use super::{Base64String, ValueType};
use crate::{
    error::ParseYamlError,
    key::{Key, KeyOwned},
};
use base64::prelude::{BASE64_STANDARD, *};
use serde::{Deserialize, Serialize};
use std::fmt::{self, Debug, Display};

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
