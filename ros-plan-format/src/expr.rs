use crate::error::InvalidByteArrayData;
use base64::prelude::*;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt::{self, Debug, Display};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ValueOrExpr {
    Value(Value),
    Expr { eval: Expr },
}

impl ValueOrExpr {
    pub fn into_value(self) -> Option<Value> {
        match self {
            ValueOrExpr::Value(value) => Some(value),
            ValueOrExpr::Expr { .. } => None,
        }
    }

    pub fn as_value(&self) -> Option<&Value> {
        match self {
            ValueOrExpr::Value(value) => Some(value),
            ValueOrExpr::Expr { .. } => None,
        }
    }

    pub fn as_eval(&self) -> Option<&Expr> {
        match self {
            ValueOrExpr::Expr { eval } => Some(eval),
            ValueOrExpr::Value(_) => None,
        }
    }

    pub fn to_bool(&self) -> Option<bool> {
        self.as_value()?.to_bool()
    }

    pub fn to_i64(&self) -> Option<i64> {
        self.as_value()?.to_i64()
    }

    pub fn to_f64(&self) -> Option<f64> {
        self.as_value()?.to_f64()
    }

    pub fn as_str(&self) -> Option<&str> {
        self.as_value()?.as_str()
    }

    pub fn as_bytes(&self) -> Option<&[u8]> {
        self.as_value()?.as_bytes()
    }

    pub fn as_bool_slice(&self) -> Option<&[bool]> {
        self.as_value()?.as_bool_slice()
    }

    pub fn as_i64_slice(&self) -> Option<&[i64]> {
        self.as_value()?.as_i64_slice()
    }

    pub fn as_f64_slice(&self) -> Option<&[f64]> {
        self.as_value()?.as_f64_slice()
    }

    pub fn as_string_slice(&self) -> Option<&[String]> {
        self.as_value()?.as_string_slice()
    }
}

impl From<Expr> for ValueOrExpr {
    fn from(eval: Expr) -> Self {
        Self::Expr { eval }
    }
}

impl From<Value> for ValueOrExpr {
    fn from(v: Value) -> Self {
        Self::Value(v)
    }
}

impl Display for ValueOrExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValueOrExpr::Value(value) => Display::fmt(value, f),
            ValueOrExpr::Expr { eval } => Display::fmt(eval, f),
        }
    }
}

#[derive(Clone)]
pub struct Expr(String);

impl Expr {
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl AsRef<str> for Expr {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

impl Serialize for Expr {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let code = &self.0;
        format!("{{{code}}}").serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Expr {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let text = String::deserialize(deserializer)?;
        // if text.contains('\n') {
        //     return Err(D::Error::custom("the code cannot contain a line break"));
        // }

        // let Some(suffix) = text.strip_prefix("{") else {
        //     return Err(D::Error::custom(
        //         "the code must be wrapped within \"{ code }\"",
        //     ));
        // };
        // let Some(code) = suffix.strip_suffix("}") else {
        //     return Err(D::Error::custom(
        //         "the code must be wrapped within \"{ code }\"",
        //     ));
        // };

        Ok(Self(text.to_string()))
    }
}

impl Debug for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Debug::fmt(&self.0, f)
    }
}

impl Display for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(&self.0, f)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Value {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
    ByteArray { bytes: ByteArrayData },
}

impl Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Bool(val) => Debug::fmt(val, f),
            Value::Integer(val) => Debug::fmt(val, f),
            Value::Double(val) => Debug::fmt(val, f),
            Value::String(val) => Debug::fmt(val, f),
            Value::BoolArray(vec) => Debug::fmt(vec, f),
            Value::IntegerArray(vec) => Debug::fmt(vec, f),
            Value::DoubleArray(vec) => Debug::fmt(vec, f),
            Value::StringArray(vec) => Debug::fmt(vec, f),
            Value::ByteArray { bytes } => Debug::fmt(bytes, f),
        }
    }
}

impl From<Vec<String>> for Value {
    fn from(v: Vec<String>) -> Self {
        Self::StringArray(v)
    }
}

impl From<Vec<f64>> for Value {
    fn from(v: Vec<f64>) -> Self {
        Self::DoubleArray(v)
    }
}

impl From<Vec<i64>> for Value {
    fn from(v: Vec<i64>) -> Self {
        Self::IntegerArray(v)
    }
}

impl From<Vec<u8>> for Value {
    fn from(v: Vec<u8>) -> Self {
        Self::ByteArray {
            bytes: ByteArrayData(v),
        }
    }
}

impl From<Vec<bool>> for Value {
    fn from(v: Vec<bool>) -> Self {
        Self::BoolArray(v)
    }
}

impl From<String> for Value {
    fn from(v: String) -> Self {
        Self::String(v)
    }
}

impl From<f64> for Value {
    fn from(v: f64) -> Self {
        Self::Double(v)
    }
}

impl From<i64> for Value {
    fn from(v: i64) -> Self {
        Self::Integer(v)
    }
}

impl From<bool> for Value {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedByteArrayData", into = "SerializedByteArrayData")]
pub struct ByteArrayData(pub Vec<u8>);

impl ByteArrayData {
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }
}

impl From<Vec<u8>> for ByteArrayData {
    fn from(value: Vec<u8>) -> Self {
        Self(value)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
enum SerializedByteArrayData {
    Array(Vec<u8>),
    Text(String),
}

impl TryFrom<SerializedByteArrayData> for ByteArrayData {
    type Error = InvalidByteArrayData;

    fn try_from(data: SerializedByteArrayData) -> Result<Self, Self::Error> {
        let data = match data {
            SerializedByteArrayData::Array(data) => data,
            SerializedByteArrayData::Text(text) => {
                let Some((prefix, suffix)) = text.split_once(':') else {
                    return Err(InvalidByteArrayData {
                        reason: "the byte array data must starts with \
                                     \"hex:\" or \"base64:\""
                            .to_string(),
                    });
                };

                match prefix {
                    "hex" => hex::decode(suffix).map_err(|err| InvalidByteArrayData {
                        reason: format!("bad hex data: {err}"),
                    })?,
                    "base64" => {
                        BASE64_STANDARD
                            .decode(suffix)
                            .map_err(|err| InvalidByteArrayData {
                                reason: format!("bad base64 data: {err}"),
                            })?
                    }
                    _ => {
                        return Err(InvalidByteArrayData {
                            reason: "the byte array data must starts with \
                                     \"hex:\" or \"base64:\""
                                .to_string(),
                        })
                    }
                }
            }
        };
        Ok(data.into())
    }
}

impl From<ByteArrayData> for SerializedByteArrayData {
    fn from(value: ByteArrayData) -> Self {
        Self::Array(value.0)
    }
}

impl Value {
    pub fn ty(&self) -> ValueType {
        match self {
            Value::Bool(_) => ValueType::Bool,
            Value::Integer(_) => ValueType::Integer,
            Value::Double(_) => ValueType::Double,
            Value::String(_) => ValueType::String,
            Value::BoolArray(_) => ValueType::BoolArray,
            Value::ByteArray { .. } => ValueType::ByteArray,
            Value::IntegerArray(_) => ValueType::IntegerArray,
            Value::DoubleArray(_) => ValueType::DoubleArray,
            Value::StringArray(_) => ValueType::StringArray,
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
        if let Self::Integer(v) = self {
            Some(*v)
        } else {
            None
        }
    }

    pub fn to_f64(&self) -> Option<f64> {
        if let Self::Double(v) = self {
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
        if let Self::ByteArray { bytes } = self {
            Some(bytes.as_slice())
        } else {
            None
        }
    }

    pub fn as_bool_slice(&self) -> Option<&[bool]> {
        if let Self::BoolArray(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_i64_slice(&self) -> Option<&[i64]> {
        if let Self::IntegerArray(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_f64_slice(&self) -> Option<&[f64]> {
        if let Self::DoubleArray(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_string_slice(&self) -> Option<&[String]> {
        if let Self::StringArray(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

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
#[serde(rename_all = "snake_case")]
pub enum ValueType {
    #[strum(serialize = "bool")]
    Bool,

    #[serde(rename = "i64")]
    #[strum(serialize = "i64")]
    Integer,

    #[serde(rename = "f64")]
    #[strum(serialize = "f64")]
    Double,

    #[strum(serialize = "string")]
    String,

    #[strum(serialize = "bool_array")]
    BoolArray,

    #[strum(serialize = "byte_array")]
    ByteArray,

    #[serde(rename = "i64_array")]
    #[strum(serialize = "i64_array")]
    IntegerArray,

    #[serde(rename = "f64_array")]
    #[strum(serialize = "f64_array")]
    DoubleArray,

    #[strum(serialize = "string_array")]
    StringArray,
}
