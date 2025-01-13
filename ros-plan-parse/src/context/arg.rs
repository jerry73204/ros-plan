use ros_plan_format::{
    argument::{ArgCfg, ArgEntry},
    expr::{Value, ValueOrExpr, ValueType},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArgContext {
    pub ty: ValueType,
    pub help: Option<String>,
    pub default: Option<Value>,
    pub assign: Option<ValueOrExpr>,
    #[serde(rename = "override")]
    pub override_: Option<ValueOrExpr>,
    #[serde(skip)]
    pub result: Option<Value>,
}

impl ArgContext {
    pub fn new(spec: ArgEntry, assign: Option<ValueOrExpr>) -> Self {
        let ArgEntry { slot, help } = spec;
        let (ty, default_in_spec) = match slot {
            ArgCfg::Required { ty } => (ty, None),
            ArgCfg::Optional { default } => (default.ty(), Some(default)),
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
