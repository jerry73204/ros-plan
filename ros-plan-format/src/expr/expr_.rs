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
