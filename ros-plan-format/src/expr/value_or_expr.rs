use super::{Expr, KeyOrExpr, TextOrExpr, Value, ValueType};
use crate::{error::DeserializationError, key::Key};
use serde::{Deserialize, Serialize};
use serde_yaml::value::{Tag, TaggedValue};
use std::fmt::{self, Debug, Display};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "TaggedValue", into = "TaggedValue")]
pub enum ValueOrExpr {
    Value(Value),
    Expr { ty: ValueType, expr: Expr },
}

impl ValueOrExpr {
    pub fn ty(&self) -> ValueType {
        match self {
            ValueOrExpr::Value(value) => value.ty(),
            ValueOrExpr::Expr { ty, .. } => *ty,
        }
    }

    pub fn into_value(self) -> Option<Value> {
        match self {
            ValueOrExpr::Value(value) => Some(value),
            _ => None,
        }
    }

    pub fn as_value(&self) -> Option<&Value> {
        match self {
            ValueOrExpr::Value(value) => Some(value),
            ValueOrExpr::Expr { .. } => None,
        }
    }

    pub fn as_expr(&self) -> Option<(ValueType, &Expr)> {
        match self {
            ValueOrExpr::Expr { ty, expr } => Some((*ty, expr)),
            _ => None,
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

    pub fn as_key(&self) -> Option<&Key> {
        self.as_value()?.as_key()
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

impl From<Value> for ValueOrExpr {
    fn from(v: Value) -> Self {
        Self::Value(v)
    }
}

impl Display for ValueOrExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValueOrExpr::Value(value) => Display::fmt(value, f),
            ValueOrExpr::Expr { expr, .. } => Display::fmt(expr, f),
        }
    }
}

impl TryFrom<TaggedValue> for ValueOrExpr {
    type Error = DeserializationError;

    fn try_from(tagged_value: TaggedValue) -> Result<Self, Self::Error> {
        let TaggedValue {
            tag,
            value: inner_value,
        } = tagged_value;

        let value_or_expr: ValueOrExpr = if tag == Tag::new("str") {
            let serde_yaml::Value::String(text) = inner_value else {
                todo!();
            };

            let text_or_expr: TextOrExpr = text
                .parse()
                .map_err(|_err| DeserializationError::ExpectTextOrExpr)?;

            match text_or_expr {
                TextOrExpr::Text(text) => Value::String(text).into(),
                TextOrExpr::Expr(expr) => {
                    let ty = ValueType::from_yaml_tag(&tag)?;
                    ValueOrExpr::Expr { ty, expr }
                }
            }
        } else if tag == Tag::new("key") {
            let serde_yaml::Value::String(text) = inner_value else {
                todo!();
            };

            let key_or_expr: KeyOrExpr = text
                .parse()
                .map_err(|_err| DeserializationError::ExpectTextOrExpr)?;

            match key_or_expr {
                KeyOrExpr::Key(key) => Value::Key(key).into(),
                KeyOrExpr::Expr(expr) => {
                    let ty = ValueType::from_yaml_tag(&tag)?;
                    ValueOrExpr::Expr { ty, expr }
                }
            }
        } else if let serde_yaml::Value::String(text) = inner_value {
            let ty = ValueType::from_yaml_tag(&tag)?;
            let expr: Expr = text
                .parse()
                .map_err(|_err| DeserializationError::ExpectExpr)?;
            ValueOrExpr::Expr { ty, expr }
        } else {
            let outer_value = serde_yaml::Value::Tagged(Box::new(TaggedValue {
                tag,
                value: inner_value,
            }));
            let value: Value = serde_yaml::from_value(outer_value)
                .map_err(|error| DeserializationError::ExpectValueWithType { error })?;
            value.into()
        };

        Ok(value_or_expr)
    }
}

impl From<ValueOrExpr> for TaggedValue {
    fn from(value: ValueOrExpr) -> Self {
        match value {
            ValueOrExpr::Value(value) => {
                let serde_yaml::Value::Tagged(tagged_value) = serde_yaml::to_value(value).unwrap()
                else {
                    unreachable!()
                };
                *tagged_value
            }
            ValueOrExpr::Expr { ty, expr } => TaggedValue {
                tag: Tag::new(ty.to_string()),
                value: serde_yaml::to_value(expr).unwrap(),
            },
        }
    }
}
