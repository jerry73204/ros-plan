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
    #[serde(default = "default_durability")]
    pub durability: DurabilityPolicy,
    #[serde(default = "default_history")]
    pub history: HistoryPolicy,
}

fn default_durability() -> DurabilityPolicy {
    DurabilityPolicy::Volatile
}

fn default_history() -> HistoryPolicy {
    HistoryPolicy::KeepLast
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum ReliabilityPolicy {
    BestEffort,
    Reliable,
    SystemDefault,
    Unknown,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum DurabilityPolicy {
    Volatile,
    TransientLocal,
    SystemDefault,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum HistoryPolicy {
    KeepLast,
    KeepAll,
    SystemDefault,
}
