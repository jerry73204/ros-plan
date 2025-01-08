use crate::qos::QosPreset;
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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReliabilityPolicy {
    BestEffort,
    Reliable,
    SystemDefault,
    Unknown,
}
