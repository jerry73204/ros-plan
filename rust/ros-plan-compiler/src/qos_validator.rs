use ros_plan_format::{
    qos::{DurabilityPolicy, HistoryPolicy, Qos, QosPreset, QosProfile, ReliabilityPolicy},
    qos_requirement::{QosRequirement, QosRequirementProfile},
};

/// Error result from QoS validation: (policy_name, required_value, offered_value)
pub type QosValidationError = (String, String, String);

/// Validates that an offered QoS profile satisfies a requirement
pub fn satisfies_requirement(
    offered: &Qos,
    required: &QosRequirement,
) -> Result<(), QosValidationError> {
    let offered_profile = expand_qos_to_profile(offered);
    let required_profile = expand_requirement_to_profile(required);

    // Validate reliability
    if let Some(req_reliability) = &required_profile.reliability {
        if !reliability_satisfies(&offered_profile.reliability, req_reliability) {
            return Err((
                "reliability".to_string(),
                format!("{:?}", req_reliability),
                format!("{:?}", offered_profile.reliability),
            ));
        }
    }

    // Validate durability
    if let Some(req_durability) = &required_profile.durability {
        if !durability_satisfies(&offered_profile.durability, req_durability) {
            return Err((
                "durability".to_string(),
                format!("{:?}", req_durability),
                format!("{:?}", offered_profile.durability),
            ));
        }
    }

    // Validate depth
    if let Some(req_depth) = required_profile.depth {
        if offered_profile.depth < req_depth {
            return Err((
                "depth".to_string(),
                format!("{}", req_depth),
                format!("{}", offered_profile.depth),
            ));
        }
    }

    // History validation: KeepAll satisfies any requirement, KeepLast only satisfies KeepLast
    if let Some(req_history) = &required_profile.history {
        if !history_satisfies(&offered_profile.history, req_history) {
            return Err((
                "history".to_string(),
                format!("{:?}", req_history),
                format!("{:?}", offered_profile.history),
            ));
        }
    }

    Ok(())
}

/// Derives a minimal QoS profile that satisfies all requirements
/// Falls back to ROS 2 defaults when no requirements are specified
pub fn derive_minimal_qos(requirements: &[&QosRequirement]) -> Qos {
    if requirements.is_empty() {
        // No requirements - use ROS 2 defaults
        return Qos::Profile(QosProfile {
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            history: HistoryPolicy::KeepLast,
            depth: 10,
        });
    }

    let req_profiles: Vec<_> = requirements
        .iter()
        .map(|r| expand_requirement_to_profile(r))
        .collect();

    // Find strictest reliability (Reliable > BestEffort)
    let reliability = req_profiles
        .iter()
        .filter_map(|p| p.reliability.as_ref())
        .max_by_key(|r| reliability_strictness(r))
        .cloned()
        .unwrap_or(ReliabilityPolicy::Reliable); // Default to Reliable

    // Find strictest durability (TransientLocal > Volatile)
    let durability = req_profiles
        .iter()
        .filter_map(|p| p.durability.as_ref())
        .max_by_key(|d| durability_strictness(d))
        .cloned()
        .unwrap_or(DurabilityPolicy::Volatile); // Default to Volatile

    // Find strictest history (KeepAll > KeepLast, but default to KeepLast to avoid unbounded growth)
    let history = req_profiles
        .iter()
        .filter_map(|p| p.history.as_ref())
        .max_by_key(|h| history_strictness(h))
        .cloned()
        .unwrap_or(HistoryPolicy::KeepLast); // Default to KeepLast

    // Find maximum depth
    let depth = req_profiles
        .iter()
        .filter_map(|p| p.depth)
        .max()
        .unwrap_or(10); // Default to 10

    Qos::Profile(QosProfile {
        reliability,
        durability,
        history,
        depth,
    })
}

/// Expands a QoS (preset or profile) to a full profile
fn expand_qos_to_profile(qos: &Qos) -> QosProfile {
    match qos {
        Qos::Preset(preset) => expand_preset(preset),
        Qos::Profile(profile) => profile.clone(),
    }
}

/// Expands a QoS requirement to a profile with optional fields
fn expand_requirement_to_profile(req: &QosRequirement) -> QosRequirementProfile {
    match req {
        QosRequirement::Preset(preset) => {
            let profile = expand_preset(preset);
            QosRequirementProfile {
                depth: Some(profile.depth),
                reliability: Some(profile.reliability),
                durability: Some(profile.durability),
                history: Some(profile.history),
            }
        }
        QosRequirement::Profile(profile) => profile.clone(),
    }
}

/// Expands a QoS preset to a full profile
/// Based on ROS 2 QoS presets: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
fn expand_preset(preset: &QosPreset) -> QosProfile {
    match preset {
        QosPreset::Default => QosProfile {
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            history: HistoryPolicy::KeepLast,
            depth: 10,
        },
        QosPreset::ServicesDefault => QosProfile {
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            history: HistoryPolicy::KeepLast,
            depth: 10,
        },
        QosPreset::SensorData => QosProfile {
            reliability: ReliabilityPolicy::BestEffort,
            durability: DurabilityPolicy::Volatile,
            history: HistoryPolicy::KeepLast,
            depth: 5,
        },
        QosPreset::SystemDefault => QosProfile {
            reliability: ReliabilityPolicy::SystemDefault,
            durability: DurabilityPolicy::SystemDefault,
            history: HistoryPolicy::SystemDefault,
            depth: 10,
        },
    }
}

/// Checks if offered reliability satisfies required reliability
/// Rules: Reliable → Any, BestEffort → BestEffort, SystemDefault → Any
fn reliability_satisfies(offered: &ReliabilityPolicy, required: &ReliabilityPolicy) -> bool {
    use ReliabilityPolicy::*;
    match (offered, required) {
        // SystemDefault and Unknown are always compatible
        (SystemDefault, _) | (_, SystemDefault) => true,
        (Unknown, _) | (_, Unknown) => true,
        // Reliable satisfies any requirement
        (Reliable, _) => true,
        // BestEffort only satisfies BestEffort
        (BestEffort, BestEffort) => true,
        (BestEffort, Reliable) => false,
    }
}

/// Checks if offered durability satisfies required durability
/// Rules: TransientLocal → Any, Volatile → Volatile, SystemDefault → Any
fn durability_satisfies(offered: &DurabilityPolicy, required: &DurabilityPolicy) -> bool {
    use DurabilityPolicy::*;
    match (offered, required) {
        // SystemDefault is always compatible
        (SystemDefault, _) | (_, SystemDefault) => true,
        // TransientLocal satisfies any requirement
        (TransientLocal, _) => true,
        // Volatile only satisfies Volatile
        (Volatile, Volatile) => true,
        (Volatile, TransientLocal) => false,
    }
}

/// Checks if offered history satisfies required history
/// Rules: KeepAll → Any, KeepLast → KeepLast, SystemDefault → Any
fn history_satisfies(offered: &HistoryPolicy, required: &HistoryPolicy) -> bool {
    use HistoryPolicy::*;
    match (offered, required) {
        // SystemDefault is always compatible
        (SystemDefault, _) | (_, SystemDefault) => true,
        // KeepAll satisfies any requirement
        (KeepAll, _) => true,
        // KeepLast only satisfies KeepLast
        (KeepLast, KeepLast) => true,
        (KeepLast, KeepAll) => false,
    }
}

/// Returns strictness level for reliability (higher = stricter)
fn reliability_strictness(r: &ReliabilityPolicy) -> u8 {
    match r {
        ReliabilityPolicy::Reliable => 2,
        ReliabilityPolicy::BestEffort => 1,
        ReliabilityPolicy::SystemDefault => 0,
        ReliabilityPolicy::Unknown => 0,
    }
}

/// Returns strictness level for durability (higher = stricter)
fn durability_strictness(d: &DurabilityPolicy) -> u8 {
    match d {
        DurabilityPolicy::TransientLocal => 2,
        DurabilityPolicy::Volatile => 1,
        DurabilityPolicy::SystemDefault => 0,
    }
}

/// Returns strictness level for history (higher = stricter)
fn history_strictness(h: &HistoryPolicy) -> u8 {
    match h {
        HistoryPolicy::KeepAll => 2,
        HistoryPolicy::KeepLast => 1,
        HistoryPolicy::SystemDefault => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reliability_compatible_reliable_to_any() {
        // Reliable publisher → Reliable subscriber ✓
        assert!(reliability_satisfies(
            &ReliabilityPolicy::Reliable,
            &ReliabilityPolicy::Reliable
        ));
        // Reliable publisher → BestEffort subscriber ✓
        assert!(reliability_satisfies(
            &ReliabilityPolicy::Reliable,
            &ReliabilityPolicy::BestEffort
        ));
    }

    #[test]
    fn test_reliability_compatible_best_effort_to_best_effort() {
        // BestEffort publisher → BestEffort subscriber ✓
        assert!(reliability_satisfies(
            &ReliabilityPolicy::BestEffort,
            &ReliabilityPolicy::BestEffort
        ));
    }

    #[test]
    fn test_reliability_incompatible_best_effort_to_reliable() {
        // BestEffort publisher → Reliable subscriber ✗
        assert!(!reliability_satisfies(
            &ReliabilityPolicy::BestEffort,
            &ReliabilityPolicy::Reliable
        ));
    }

    #[test]
    fn test_durability_compatible_transient_to_any() {
        // TransientLocal → TransientLocal ✓
        assert!(durability_satisfies(
            &DurabilityPolicy::TransientLocal,
            &DurabilityPolicy::TransientLocal
        ));
        // TransientLocal → Volatile ✓
        assert!(durability_satisfies(
            &DurabilityPolicy::TransientLocal,
            &DurabilityPolicy::Volatile
        ));
    }

    #[test]
    fn test_durability_compatible_volatile_to_volatile() {
        // Volatile → Volatile ✓
        assert!(durability_satisfies(
            &DurabilityPolicy::Volatile,
            &DurabilityPolicy::Volatile
        ));
    }

    #[test]
    fn test_durability_incompatible_volatile_to_transient() {
        // Volatile → TransientLocal ✗
        assert!(!durability_satisfies(
            &DurabilityPolicy::Volatile,
            &DurabilityPolicy::TransientLocal
        ));
    }

    #[test]
    fn test_depth_requirement_satisfaction() {
        let offered = Qos::Profile(QosProfile {
            reliability: ReliabilityPolicy::Reliable,
            durability: DurabilityPolicy::Volatile,
            history: HistoryPolicy::KeepLast,
            depth: 20,
        });

        // Depth 20 satisfies requirement ≥10 ✓
        let required = QosRequirement::Profile(QosRequirementProfile {
            depth: Some(10),
            reliability: None,
            durability: None,
            history: None,
        });
        assert!(satisfies_requirement(&offered, &required).is_ok());

        // Depth 20 fails requirement ≥30 ✗
        let required_high = QosRequirement::Profile(QosRequirementProfile {
            depth: Some(30),
            reliability: None,
            durability: None,
            history: None,
        });
        assert!(satisfies_requirement(&offered, &required_high).is_err());
    }

    #[test]
    fn test_derive_qos_from_multiple_requirements() {
        let req1 = QosRequirement::Profile(QosRequirementProfile {
            reliability: Some(ReliabilityPolicy::BestEffort),
            depth: Some(5),
            durability: None,
            history: None,
        });
        let req2 = QosRequirement::Profile(QosRequirementProfile {
            reliability: Some(ReliabilityPolicy::Reliable),
            depth: Some(15),
            durability: Some(DurabilityPolicy::TransientLocal),
            history: None,
        });

        let derived = derive_minimal_qos(&[&req1, &req2]);

        if let Qos::Profile(profile) = derived {
            // Should use strictest: Reliable, TransientLocal, depth=15
            assert_eq!(profile.reliability, ReliabilityPolicy::Reliable);
            assert_eq!(profile.durability, DurabilityPolicy::TransientLocal);
            assert_eq!(profile.depth, 15);
        } else {
            panic!("Expected Profile, got Preset");
        }
    }

    #[test]
    fn test_derive_qos_with_no_requirements_uses_defaults() {
        let derived = derive_minimal_qos(&[]);

        if let Qos::Profile(profile) = derived {
            // Should use ROS 2 defaults: Reliable, Volatile, KeepLast, depth=10
            assert_eq!(profile.reliability, ReliabilityPolicy::Reliable);
            assert_eq!(profile.durability, DurabilityPolicy::Volatile);
            assert_eq!(profile.history, HistoryPolicy::KeepLast);
            assert_eq!(profile.depth, 10);
        } else {
            panic!("Expected Profile, got Preset");
        }
    }

    #[test]
    fn test_system_default_always_compatible() {
        // SystemDefault reliability is compatible with anything
        assert!(reliability_satisfies(
            &ReliabilityPolicy::SystemDefault,
            &ReliabilityPolicy::Reliable
        ));
        assert!(reliability_satisfies(
            &ReliabilityPolicy::Reliable,
            &ReliabilityPolicy::SystemDefault
        ));

        // SystemDefault durability is compatible with anything
        assert!(durability_satisfies(
            &DurabilityPolicy::SystemDefault,
            &DurabilityPolicy::TransientLocal
        ));
        assert!(durability_satisfies(
            &DurabilityPolicy::Volatile,
            &DurabilityPolicy::SystemDefault
        ));
    }
}
