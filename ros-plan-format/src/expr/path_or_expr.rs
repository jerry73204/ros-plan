use std::{
    fmt::{self, Display},
    path::PathBuf,
    str::FromStr,
};

use super::Expr;
use crate::error::ParseExpressionError;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};

#[derive(Debug, Clone)]
pub enum PathOrExpr {
    Path(PathBuf),
    Expr(Expr),
}

impl From<Expr> for PathOrExpr {
    fn from(v: Expr) -> Self {
        Self::Expr(v)
    }
}

impl From<PathBuf> for PathOrExpr {
    fn from(v: PathBuf) -> Self {
        Self::Path(v)
    }
}

impl Display for PathOrExpr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PathOrExpr::Path(path) => {
                if path.starts_with("$") {
                    write!(f, "\\{}", path.display())
                } else {
                    write!(f, "{}", path.display())
                }
            }
            PathOrExpr::Expr(expr) => {
                write!(f, "{expr}")
            }
        }
    }
}

impl FromStr for PathOrExpr {
    type Err = ParseExpressionError;

    fn from_str(path: &str) -> Result<Self, Self::Err> {
        let path_or_expr: PathOrExpr = if let Some(escaped) = path.strip_prefix("\\") {
            PathBuf::from(escaped).into()
        } else if path.starts_with("$") {
            let expr: Expr = path.parse()?;
            expr.into()
        } else {
            PathBuf::from(path).into()
        };

        Ok(path_or_expr)
    }
}

impl Serialize for PathOrExpr {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_string().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for PathOrExpr {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let string = String::deserialize(deserializer)?;
        let text_or_expr: PathOrExpr = string
            .parse()
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(text_or_expr)
    }
}
