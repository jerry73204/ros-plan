use std::{
    fmt::{self, Display},
    str::FromStr,
};

use super::Expr;
use crate::{error::ParseExpressionError, key::KeyOwned};
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};

#[derive(Debug, Clone)]
pub enum KeyOrExpr {
    Key(KeyOwned),
    Expr(Expr),
}

impl From<Expr> for KeyOrExpr {
    fn from(v: Expr) -> Self {
        Self::Expr(v)
    }
}

impl From<KeyOwned> for KeyOrExpr {
    fn from(v: KeyOwned) -> Self {
        Self::Key(v)
    }
}

impl Display for KeyOrExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            KeyOrExpr::Key(text) => {
                write!(f, "{text}")
            }
            KeyOrExpr::Expr(expr) => {
                write!(f, "{expr}")
            }
        }
    }
}

impl FromStr for KeyOrExpr {
    type Err = ParseExpressionError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let key_or_expr: KeyOrExpr = if text.starts_with("$") {
            let expr: Expr = text.parse()?;
            expr.into()
        } else {
            let key: KeyOwned = text.parse().map_err(|_| todo!())?;
            key.into()
        };

        Ok(key_or_expr)
    }
}

impl Serialize for KeyOrExpr {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_string().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for KeyOrExpr {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let string = String::deserialize(deserializer)?;
        let text_or_expr: KeyOrExpr = string
            .parse()
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(text_or_expr)
    }
}
