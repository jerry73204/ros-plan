use ros_plan_format::eval::{Value, ValueOrEval, ValueType};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct ArgContext {
    pub ty: ValueType,
    pub help: Option<String>,
    pub default: Option<Value>,
    pub assign: Option<ValueOrEval>,
}
