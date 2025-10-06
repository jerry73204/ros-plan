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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_simple_bool_expr() {
        let text = "$ enable_feature $";
        let result: Result<BoolExpr, _> = text.parse();
        assert!(result.is_ok());
        let bool_expr = result.unwrap();
        assert_eq!(bool_expr.as_str(), " enable_feature ");
    }

    #[test]
    fn parse_bool_expr_comparison() {
        let text = "$ count > 0 $";
        let result: Result<BoolExpr, _> = text.parse();
        assert!(result.is_ok());
        let bool_expr = result.unwrap();
        assert_eq!(bool_expr.as_str(), " count > 0 ");
    }

    #[test]
    fn parse_bool_expr_logical_and() {
        let text = "$ flag1 and flag2 $";
        let result: Result<BoolExpr, _> = text.parse();
        assert!(result.is_ok());
        let bool_expr = result.unwrap();
        assert_eq!(bool_expr.as_str(), " flag1 and flag2 ");
    }

    #[test]
    fn parse_bool_expr_multi_line() {
        let text = "$$$\nlocal x = get_value()\nreturn x > 10\n$$$\n";
        let result: Result<BoolExpr, _> = text.parse();
        assert!(result.is_ok());
        let bool_expr = result.unwrap();
        assert_eq!(bool_expr.as_str(), "local x = get_value()\nreturn x > 10\n");
    }

    #[test]
    fn reject_invalid_expr() {
        let text = "not an expression";
        let result: Result<BoolExpr, _> = text.parse();
        assert!(result.is_err());
    }

    #[test]
    fn as_expr_method() {
        let bool_expr: BoolExpr = "$ flag $".parse().unwrap();
        let expr = bool_expr.as_expr();
        assert_eq!(expr.as_str(), " flag ");
    }

    #[test]
    fn display_format() {
        let bool_expr: BoolExpr = "$ enabled $".parse().unwrap();
        let formatted = bool_expr.to_string();
        assert_eq!(formatted, "$ enabled $");
    }

    #[test]
    fn roundtrip() {
        let original = "$ x and y $";
        let bool_expr: BoolExpr = original.parse().unwrap();
        let serialized = bool_expr.to_string();
        let reparsed: BoolExpr = serialized.parse().unwrap();
        assert_eq!(bool_expr.as_str(), reparsed.as_str());
    }

    #[test]
    fn deserialize_from_yaml() {
        let yaml = "$ condition $";
        let result: Result<BoolExpr, _> = serde_yaml::from_str(yaml);
        assert!(result.is_ok());
        let bool_expr = result.unwrap();
        assert_eq!(bool_expr.as_str(), " condition ");
    }
}
