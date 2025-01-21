use base64::prelude::*;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};
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
        code.serialize(serializer)
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
    Binary(Binary),
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
        Self::Binary(Binary(v))
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

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Binary(pub Vec<u8>);

impl Binary {
    pub fn as_slice(&self) -> &[u8] {
        &self.0
    }
}

impl From<Vec<u8>> for Binary {
    fn from(value: Vec<u8>) -> Self {
        Self(value)
    }
}

impl From<Binary> for Vec<u8> {
    fn from(value: Binary) -> Self {
        value.0
    }
}

impl Serialize for Binary {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        BASE64_STANDARD.encode(&self.0).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Binary {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let base64_text = String::deserialize(deserializer)?;
        let binary = BASE64_STANDARD
            .decode(&base64_text)
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(Self(binary))
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
