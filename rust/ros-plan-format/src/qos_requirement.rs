use crate::qos::{DurabilityPolicy, HistoryPolicy, QosPreset, ReliabilityPolicy};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QosRequirement {
    Preset(QosPreset),
    Profile(QosRequirementProfile),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QosRequirementProfile {
    pub depth: Option<usize>,
    pub reliability: Option<ReliabilityPolicy>,
    pub durability: Option<DurabilityPolicy>,
    pub history: Option<HistoryPolicy>,
}
