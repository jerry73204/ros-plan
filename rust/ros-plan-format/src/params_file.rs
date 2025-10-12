use crate::{expr::Value, parameter::ParamName};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ParamsList(pub IndexMap<ParamName, Value>);
