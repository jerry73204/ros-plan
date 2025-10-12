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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_key() {
        let text = "node_name/topic";
        let result: Result<KeyOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let key_or_expr = result.unwrap();
        assert!(matches!(key_or_expr, KeyOrExpr::Key(_)));
    }

    #[test]
    fn parse_key_with_namespace() {
        let text = "namespace/node/topic";
        let result: Result<KeyOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let key_or_expr = result.unwrap();
        assert!(matches!(key_or_expr, KeyOrExpr::Key(_)));
    }

    #[test]
    fn parse_expr() {
        let text = "$ robot_ns .. \"/cmd_vel\" $";
        let result: Result<KeyOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let key_or_expr = result.unwrap();
        assert!(matches!(key_or_expr, KeyOrExpr::Expr(_)));
    }

    #[test]
    fn parse_expr_multi_line() {
        let text = "$$$\nreturn build_topic()\n$$$\n";
        let result: Result<KeyOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let key_or_expr = result.unwrap();
        assert!(matches!(key_or_expr, KeyOrExpr::Expr(_)));
    }

    #[test]
    fn display_key() {
        let key: KeyOwned = "node/topic".parse().unwrap();
        let key_or_expr = KeyOrExpr::Key(key);
        let formatted = key_or_expr.to_string();
        assert_eq!(formatted, "node/topic");
    }

    #[test]
    fn display_expr() {
        let expr = Expr::from_code("topic_name");
        let key_or_expr = KeyOrExpr::Expr(expr);
        let formatted = key_or_expr.to_string();
        assert_eq!(formatted, "$topic_name$");
    }

    #[test]
    fn from_key() {
        let key: KeyOwned = "test/key".parse().unwrap();
        let key_or_expr: KeyOrExpr = key.into();
        assert!(matches!(key_or_expr, KeyOrExpr::Key(_)));
    }

    #[test]
    fn from_expr() {
        let expr = Expr::from_code("code");
        let key_or_expr: KeyOrExpr = expr.into();
        assert!(matches!(key_or_expr, KeyOrExpr::Expr(_)));
    }

    #[test]
    fn roundtrip_key() {
        let original = "node/socket";
        let key_or_expr: KeyOrExpr = original.parse().unwrap();
        let serialized = key_or_expr.to_string();
        let reparsed: KeyOrExpr = serialized.parse().unwrap();
        assert!(matches!(reparsed, KeyOrExpr::Key(_)));
    }

    #[test]
    fn roundtrip_expr() {
        let original = "$ key_expr $";
        let key_or_expr: KeyOrExpr = original.parse().unwrap();
        let serialized = key_or_expr.to_string();
        let reparsed: KeyOrExpr = serialized.parse().unwrap();
        assert!(matches!(reparsed, KeyOrExpr::Expr(_)));
    }

    #[test]
    fn deserialize_key_from_yaml() {
        let yaml = "robot/odom";
        let result: Result<KeyOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), KeyOrExpr::Key(_)));
    }

    #[test]
    fn deserialize_expr_from_yaml() {
        let yaml = "$ topic $";
        let result: Result<KeyOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), KeyOrExpr::Expr(_)));
    }
}
