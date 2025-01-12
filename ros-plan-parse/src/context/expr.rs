use ros_plan_format::expr::{Value, ValueOrExpr, ValueType};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct ExprContext {
    pub default: ValueOrExpr,
    pub override_: Option<Value>,
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
