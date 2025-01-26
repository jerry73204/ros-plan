use ros_plan_format::expr::{TextOrExpr, Value, ValueOrExpr, ValueType};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextOrExprCtx {
    pub default: TextOrExpr,
    #[serde(rename = "override")]
    pub override_: Option<String>,
    pub result: Option<String>,
}

impl TextOrExprCtx {
    pub fn new(default: TextOrExpr) -> Self {
        Self {
            default,
            override_: None,
            result: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExprCtx {
    pub default: ValueOrExpr,
    #[serde(rename = "override")]
    pub override_: Option<Value>,
    pub result: Option<Value>,
}

impl ExprCtx {
    pub fn new(default: ValueOrExpr) -> Self {
        Self {
            default,
            override_: None,
            result: None,
        }
    }

    pub fn ty(&self) -> Option<ValueType> {
        Some(self.result.as_ref()?.ty())
    }
}
