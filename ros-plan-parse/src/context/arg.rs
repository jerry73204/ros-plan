use ros_plan_format::{
    expr::{Value, ValueOrExpr, ValueType},
    parameter::{ArgEntry, ArgSlot},
};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct ArgContext {
    pub ty: ValueType,
    pub help: Option<String>,
    pub default: Option<Value>,
    pub assign: Option<ValueOrExpr>,
    pub override_: Option<ValueOrExpr>,
    pub result: Option<Value>,
}

impl ArgContext {
    pub fn new(spec: ArgEntry, assign: Option<ValueOrExpr>) -> Self {
        let ArgEntry { slot, help } = spec;
        let (ty, default_in_spec) = match slot {
            ArgSlot::Required { ty } => (ty, None),
            ArgSlot::Optional { default } => (default.ty(), Some(default)),
        };

        Self {
            ty,
            help,
            default: default_in_spec,
            assign,
            override_: None,
            result: None,
        }
    }
}
