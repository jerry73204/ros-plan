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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_plain_text() {
        let text = "hello world";
        let result: Result<TextOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let text_or_expr = result.unwrap();
        assert!(matches!(text_or_expr, TextOrExpr::Text(_)));
    }

    #[test]
    fn parse_expr() {
        let text = "$ robot_name .. \"/cmd_vel\" $";
        let result: Result<TextOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let text_or_expr = result.unwrap();
        assert!(matches!(text_or_expr, TextOrExpr::Expr(_)));
    }

    #[test]
    fn parse_escaped_dollar() {
        let text = "\\$not_an_expr";
        let result: Result<TextOrExpr, _> = text.parse();
        assert!(result.is_ok());
        let text_or_expr = result.unwrap();
        if let TextOrExpr::Text(s) = text_or_expr {
            assert_eq!(s, "$not_an_expr");
        } else {
            panic!("Expected Text variant");
        }
    }

    #[test]
    fn display_plain_text() {
        let text_or_expr = TextOrExpr::Text("hello".to_string());
        let formatted = text_or_expr.to_string();
        assert_eq!(formatted, "hello");
    }

    #[test]
    fn display_text_starting_with_dollar() {
        let text_or_expr = TextOrExpr::Text("$special".to_string());
        let formatted = text_or_expr.to_string();
        assert_eq!(formatted, "\\$special");
    }

    #[test]
    fn display_expr() {
        let expr = Expr::from_code("a + b");
        let text_or_expr = TextOrExpr::Expr(expr);
        let formatted = text_or_expr.to_string();
        assert_eq!(formatted, "$a + b$");
    }

    #[test]
    fn roundtrip_plain_text() {
        let original = "hello world";
        let text_or_expr: TextOrExpr = original.parse().unwrap();
        let serialized = text_or_expr.to_string();
        let reparsed: TextOrExpr = serialized.parse().unwrap();
        assert!(matches!(reparsed, TextOrExpr::Text(_)));
    }

    #[test]
    fn roundtrip_expr() {
        let original = "$ x + 1 $";
        let text_or_expr: TextOrExpr = original.parse().unwrap();
        let serialized = text_or_expr.to_string();
        let reparsed: TextOrExpr = serialized.parse().unwrap();
        assert!(matches!(reparsed, TextOrExpr::Expr(_)));
    }

    #[test]
    fn from_string() {
        let text_or_expr: TextOrExpr = "test".to_string().into();
        assert!(matches!(text_or_expr, TextOrExpr::Text(_)));
    }

    #[test]
    fn from_expr() {
        let expr = Expr::from_code("code");
        let text_or_expr: TextOrExpr = expr.into();
        assert!(matches!(text_or_expr, TextOrExpr::Expr(_)));
    }

    #[test]
    fn deserialize_from_yaml() {
        let yaml = "$ value $";
        let result: Result<TextOrExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        assert!(matches!(result.unwrap(), TextOrExpr::Expr(_)));
    }
}
