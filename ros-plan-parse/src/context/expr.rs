use ros_plan_format::expr::{Value, ValueOrExpr, ValueType};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExprContext {
    pub default: ValueOrExpr,
    #[serde(rename = "override")]
    pub override_: Option<Value>,
    #[serde(skip)]
    pub result: Option<Value>,
}

impl ExprContext {
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
