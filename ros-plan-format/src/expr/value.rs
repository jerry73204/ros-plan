use super::{Base64String, ValueType};
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

impl Value {
    pub fn ty(&self) -> ValueType {
        match self {
            Value::Bool(_) => ValueType::Bool,
            Value::I64(_) => ValueType::I64,
            Value::F64(_) => ValueType::F64,
            Value::String(_) => ValueType::String,
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

    /// Returns `true` if the value is [`Bool`].
    ///
    /// [`Bool`]: Value::Bool
    #[must_use]
    pub fn is_bool(&self) -> bool {
        matches!(self, Self::Bool(..))
    }

    /// Returns `true` if the value is [`Integer`].
    ///
    /// [`Integer`]: Value::Integer
    #[must_use]
    pub fn is_integer(&self) -> bool {
        matches!(self, Self::I64(..))
    }

    /// Returns `true` if the value is [`Double`].
    ///
    /// [`Double`]: Value::Double
    #[must_use]
    pub fn is_double(&self) -> bool {
        matches!(self, Self::F64(..))
    }

    /// Returns `true` if the value is [`String`].
    ///
    /// [`String`]: Value::String
    #[must_use]
    pub fn is_string(&self) -> bool {
        matches!(self, Self::String(..))
    }

    /// Returns `true` if the value is [`BoolArray`].
    ///
    /// [`BoolArray`]: Value::BoolArray
    #[must_use]
    pub fn is_bool_array(&self) -> bool {
        matches!(self, Self::BoolList(..))
    }

    /// Returns `true` if the value is [`IntegerArray`].
    ///
    /// [`IntegerArray`]: Value::IntegerArray
    #[must_use]
    pub fn is_integer_array(&self) -> bool {
        matches!(self, Self::I64List(..))
    }

    /// Returns `true` if the value is [`DoubleArray`].
    ///
    /// [`DoubleArray`]: Value::DoubleArray
    #[must_use]
    pub fn is_double_array(&self) -> bool {
        matches!(self, Self::F64List(..))
    }

    /// Returns `true` if the value is [`StringArray`].
    ///
    /// [`StringArray`]: Value::StringArray
    #[must_use]
    pub fn is_string_array(&self) -> bool {
        matches!(self, Self::StringList(..))
    }

    /// Returns `true` if the value is [`ByteArray`].
    ///
    /// [`ByteArray`]: Value::ByteArray
    #[must_use]
    pub fn is_byte_array(&self) -> bool {
        matches!(self, Self::Binary { .. })
    }
}
