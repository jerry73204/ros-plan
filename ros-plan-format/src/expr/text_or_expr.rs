use std::{
    fmt::{self, Display},
    str::FromStr,
};

use super::Expr;
use crate::error::ParseExpressionError;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};

#[derive(Debug, Clone)]
pub enum TextOrExpr {
    Text(String),
    Expr(Expr),
}

impl From<Expr> for TextOrExpr {
    fn from(v: Expr) -> Self {
        Self::Expr(v)
    }
}

impl From<String> for TextOrExpr {
    fn from(v: String) -> Self {
        Self::Text(v)
    }
}

impl Display for TextOrExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TextOrExpr::Text(text) => {
                if text.starts_with("$") {
                    write!(f, "\\{text}")
                } else {
                    write!(f, "{text}")
                }
            }
            TextOrExpr::Expr(expr) => {
                write!(f, "{expr}")
            }
        }
    }
}

impl FromStr for TextOrExpr {
    type Err = ParseExpressionError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let text_or_expr: TextOrExpr = if let Some(escaped) = text.strip_prefix("\\") {
            escaped.to_string().into()
        } else if text.starts_with("$") {
            let expr: Expr = text.parse()?;
            expr.into()
        } else {
            text.to_string().into()
        };

        Ok(text_or_expr)
    }
}

impl Serialize for TextOrExpr {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_string().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for TextOrExpr {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let string = String::deserialize(deserializer)?;
        let text_or_expr: TextOrExpr = string
            .parse()
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(text_or_expr)
    }
}
