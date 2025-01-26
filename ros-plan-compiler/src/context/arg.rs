use super::expr::ExprCtx;
use ros_plan_format::{
    argument::ArgEntry,
    expr::{ValueOrExpr, ValueType},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArgCtx {
    pub ty: ValueType,
    pub help: Option<String>,
    pub default: Option<ExprCtx>,
    pub assign: Option<ExprCtx>,
}

impl ArgCtx {
    pub fn new(spec: ArgEntry, assign: Option<ValueOrExpr>) -> Self {
        let ArgEntry { help, ty, default } = spec;

        Self {
            ty,
            help,
            default: default.map(ExprCtx::new),
            assign: assign.map(ExprCtx::new),
        }
    }
}
