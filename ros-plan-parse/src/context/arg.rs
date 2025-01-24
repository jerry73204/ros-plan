use ros_plan_format::{
    argument::ArgEntry,
    expr::{ValueOrExpr, ValueType},
};
use serde::{Deserialize, Serialize};

use super::expr::ExprContext;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArgContext {
    pub ty: ValueType,
    pub help: Option<String>,
    pub default: Option<ExprContext>,
    pub assign: Option<ExprContext>,
}

impl ArgContext {
    pub fn new(spec: ArgEntry, assign: Option<ValueOrExpr>) -> Self {
        let ArgEntry { help, ty, default } = spec;

        Self {
            ty,
            help,
            default: default.map(ExprContext::new),
            assign: assign.map(ExprContext::new),
        }
    }
}
