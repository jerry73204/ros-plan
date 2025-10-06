use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use std::{
    fmt::{self, Debug, Display},
    str::FromStr,
};

use crate::error::ParseExpressionError;

#[derive(Debug, Clone)]
#[repr(transparent)]
pub struct Expr(String);

impl Expr {
    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn from_code(code: &str) -> Self {
        Self(code.to_string())
    }
}

impl AsRef<str> for Expr {
    fn as_ref(&self) -> &str {
        &self.0
    }
}

impl Display for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let code = &self.0;

        let text = if code.contains("\n") {
            format!("$$$\n{code}$$$\n")
        } else {
            format!("${code}$")
        };

        Display::fmt(&text, f)
    }
}

impl FromStr for Expr {
    type Err = ParseExpressionError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        macro_rules! bail {
            ($reason:expr) => {
                return Err(ParseExpressionError {
                    reason: $reason.to_string(),
                });
            };
        }

        let code = if let Some(text) = text.strip_prefix("$$$\n") {
            if let Some(text) = text.strip_suffix("$$$\n") {
                if text.ends_with("\n") || text.is_empty() {
                    text
                } else {
                    bail!("multi-line expression must ends with a line break");
                }
            } else {
                bail!("imbalanced $$$");
            }
        } else if let Some(text) = text.strip_prefix("$") {
            if let Some(text) = text.strip_suffix("$") {
                if !text.contains('\n') {
                    text
                } else {
                    bail!("multi-line expression must be wrapped within a pair of $$$");
                }
            } else {
                bail!("imbalanced $");
            }
        } else {
            bail!("expression must be wrapped within a pair of $$$ or $");
        };

        Ok(Expr(code.to_string()))
    }
}

impl Serialize for Expr {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_string().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Expr {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let string = String::deserialize(deserializer)?;
        let expr: Expr = string
            .parse()
            .map_err(|err| D::Error::custom(format!("{err}")))?;
        Ok(expr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_single_line_expr() {
        let text = "$ a + b $";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_ok());
        let expr = result.unwrap();
        assert_eq!(expr.as_str(), " a + b ");
    }

    #[test]
    fn parse_single_line_expr_no_spaces() {
        let text = "$a+b$";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_ok());
        let expr = result.unwrap();
        assert_eq!(expr.as_str(), "a+b");
    }

    #[test]
    fn parse_single_line_expr_complex() {
        let text = "$ robot_name .. \"/cmd_vel\" $";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_ok());
        let expr = result.unwrap();
        assert_eq!(expr.as_str(), " robot_name .. \"/cmd_vel\" ");
    }

    #[test]
    fn parse_multi_line_expr() {
        let text = "$$$\nlocal x = 1\nreturn x\n$$$\n";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_ok());
        let expr = result.unwrap();
        assert_eq!(expr.as_str(), "local x = 1\nreturn x\n");
    }

    #[test]
    fn parse_multi_line_expr_empty() {
        let text = "$$$\n$$$\n";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_ok());
        let expr = result.unwrap();
        assert_eq!(expr.as_str(), "");
    }

    #[test]
    fn parse_multi_line_expr_single_line() {
        let text = "$$$\nreturn 42\n$$$\n";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_ok());
        let expr = result.unwrap();
        assert_eq!(expr.as_str(), "return 42\n");
    }

    #[test]
    fn reject_single_line_with_newline() {
        let text = "$ a\nb $";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.reason.contains("multi-line"));
    }

    #[test]
    fn reject_multi_line_without_trailing_newline() {
        let text = "$$$\nreturn 42$$$\n";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.reason.contains("line break"));
    }

    #[test]
    fn reject_imbalanced_single_dollar() {
        let text = "$ a + b";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.reason.contains("imbalanced"));
    }

    #[test]
    fn reject_imbalanced_triple_dollar() {
        let text = "$$$\nreturn 42\n";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.reason.contains("imbalanced"));
    }

    #[test]
    fn reject_no_delimiters() {
        let text = "just text";
        let result: Result<Expr, _> = text.parse();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.reason.contains("wrapped"));
    }

    #[test]
    fn roundtrip_single_line() {
        let original = "$ a + b $";
        let expr: Expr = original.parse().unwrap();
        let serialized = expr.to_string();
        let reparsed: Expr = serialized.parse().unwrap();
        assert_eq!(expr.as_str(), reparsed.as_str());
    }

    #[test]
    fn roundtrip_multi_line() {
        let original = "$$$\nlocal x = 1\nreturn x\n$$$\n";
        let expr: Expr = original.parse().unwrap();
        let serialized = expr.to_string();
        let reparsed: Expr = serialized.parse().unwrap();
        assert_eq!(expr.as_str(), reparsed.as_str());
    }

    #[test]
    fn display_format_single_line() {
        let expr = Expr::from_code("a + b");
        let formatted = expr.to_string();
        assert_eq!(formatted, "$a + b$");
    }

    #[test]
    fn display_format_multi_line() {
        let expr = Expr::from_code("local x = 1\nreturn x\n");
        let formatted = expr.to_string();
        assert_eq!(formatted, "$$$\nlocal x = 1\nreturn x\n$$$\n");
    }
}
