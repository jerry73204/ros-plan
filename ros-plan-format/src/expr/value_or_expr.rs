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

impl From<bool> for ValueOrExpr {
    fn from(value: bool) -> Self {
        Value::from(value).into()
    }
}

impl From<i64> for ValueOrExpr {
    fn from(value: i64) -> Self {
        Value::from(value).into()
    }
}

impl From<f64> for ValueOrExpr {
    fn from(value: f64) -> Self {
        Value::from(value).into()
    }
}

impl From<String> for ValueOrExpr {
    fn from(value: String) -> Self {
        Value::from(value).into()
    }
}

impl From<Vec<bool>> for ValueOrExpr {
    fn from(vec: Vec<bool>) -> Self {
        Value::from(vec).into()
    }
}

impl From<Vec<i64>> for ValueOrExpr {
    fn from(vec: Vec<i64>) -> Self {
        Value::from(vec).into()
    }
}

impl From<Vec<f64>> for ValueOrExpr {
    fn from(vec: Vec<f64>) -> Self {
        Value::from(vec).into()
    }
}

impl From<Vec<String>> for ValueOrExpr {
    fn from(vec: Vec<String>) -> Self {
        Value::from(vec).into()
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_i64_value() {
        let yaml = "!i64 42";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert_eq!(value_or_expr.to_i64(), Some(42));
        assert_eq!(value_or_expr.ty(), ValueType::I64);
    }

    #[test]
    fn parse_i64_expr() {
        let yaml = "!i64 $ 1 + 2 $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert!(value_or_expr.as_expr().is_some());
        let (ty, expr) = value_or_expr.as_expr().unwrap();
        assert_eq!(ty, ValueType::I64);
        assert_eq!(expr.as_str(), " 1 + 2 ");
    }

    #[test]
    fn parse_f64_value() {
        let yaml = "!f64 2.5";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert_eq!(value_or_expr.to_f64(), Some(2.5));
    }

    #[test]
    fn parse_f64_expr() {
        let yaml = "!f64 $ rate / 2.0 $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert!(value_or_expr.as_expr().is_some());
    }

    #[test]
    fn parse_bool_value() {
        let yaml = "!bool true";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert_eq!(value_or_expr.to_bool(), Some(true));
    }

    #[test]
    fn parse_bool_expr() {
        let yaml = "!bool $ enable_feature $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert!(value_or_expr.as_expr().is_some());
    }

    #[test]
    fn parse_str_value() {
        let yaml = "!str \"hello world\"";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert_eq!(value_or_expr.as_str(), Some("hello world"));
    }

    #[test]
    fn parse_str_expr() {
        let yaml = "!str $ \"/robot/\" .. robot_id $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert!(value_or_expr.as_expr().is_some());
    }

    #[test]
    fn parse_i64_list_value() {
        let yaml = "!i64_list [1, 2, 3]";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert_eq!(value_or_expr.as_i64_slice(), Some(&[1, 2, 3][..]));
    }

    #[test]
    fn parse_i64_list_expr() {
        let yaml = "!i64_list $ {1, 2, 3} $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        assert!(value_or_expr.as_expr().is_some());
    }

    #[test]
    fn parse_str_list_value() {
        let yaml = "!str_list [\"a\", \"b\", \"c\"]";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let value_or_expr = result.unwrap();
        let expected = ["a".to_string(), "b".to_string(), "c".to_string()];
        assert_eq!(value_or_expr.as_string_slice(), Some(&expected[..]));
    }

    #[test]
    fn value_or_expr_type() {
        let yaml_i64 = "!i64 $ count $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml_i64);
        assert_eq!(result.unwrap().ty(), ValueType::I64);

        let yaml_str = "!str $ name $";
        let result: Result<ValueOrExpr, _> = serde_yaml::from_str(yaml_str);
        assert_eq!(result.unwrap().ty(), ValueType::String);
    }

    #[test]
    fn from_primitives() {
        let from_bool: ValueOrExpr = true.into();
        assert_eq!(from_bool.to_bool(), Some(true));

        let from_i64: ValueOrExpr = 42i64.into();
        assert_eq!(from_i64.to_i64(), Some(42));

        let from_f64: ValueOrExpr = 2.5f64.into();
        assert_eq!(from_f64.to_f64(), Some(2.5));

        let from_string: ValueOrExpr = "hello".to_string().into();
        assert_eq!(from_string.as_str(), Some("hello"));
    }
}
