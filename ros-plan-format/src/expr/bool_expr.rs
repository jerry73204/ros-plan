use super::Expr;
use crate::error::ParseExpressionError;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use std::{
    fmt::{self, Debug, Display},
    str::FromStr,
};

#[derive(Debug, Clone)]
#[repr(transparent)]
pub struct BoolExpr(Expr);

impl BoolExpr {
    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }

    pub fn as_expr(&self) -> &Expr {
        &self.0
    }
}

impl AsRef<str> for BoolExpr {
    fn as_ref(&self) -> &str {
        self.0.as_str()
    }
}

impl Display for BoolExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        Display::fmt(&self.0, f)
    }
}

impl FromStr for BoolExpr {
    type Err = ParseExpressionError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let expr: Expr = text.parse()?;
        Ok(Self(expr))
    }
}

impl Serialize for BoolExpr {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_string().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for BoolExpr {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let string = String::deserialize(deserializer)?;
        let expr: BoolExpr = string
            .parse()
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(expr)
    }
}
