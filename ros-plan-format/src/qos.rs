use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Qos {
    Preset(QosPreset),
    Profile(QosProfile),
}

impl Default for Qos {
    fn default() -> Self {
        Self::Preset(QosPreset::Default)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum QosPreset {
    Default,
    ServicesDefault,
    SystemDefault,
    SensorData,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct QosProfile {
    pub depth: usize,
    pub reliability: ReliabilityPolicy,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ReliabilityPolicy {
    BestEffort,
    Reliable,
    SystemDefault,
    Unknown,
}
