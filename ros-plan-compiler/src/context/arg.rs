use crate::eval::ValueStore;
use ros_plan_format::{
    argument::ArgEntry,
    expr::{ValueOrExpr, ValueType},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArgCtx {
    pub ty: ValueType,
    pub help: Option<String>,
    pub default: Option<ValueStore>,
    pub assign: Option<ValueStore>,
}

impl ArgCtx {
    pub fn new(spec: ArgEntry, assign: Option<ValueOrExpr>) -> Self {
        let ArgEntry { help, ty, default } = spec;

        Self {
            ty,
            help,
            default: default.map(ValueStore::new),
            assign: assign.map(ValueStore::new),
        }
    }
}
