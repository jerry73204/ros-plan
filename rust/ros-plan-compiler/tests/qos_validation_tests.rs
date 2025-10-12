// F18 Integration Tests
// Tests for QoS requirement satisfaction validation

use indexmap::IndexMap;
use ros_plan_compiler::Compiler;
use std::path::PathBuf;

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("qos")
        .join(name)
}

fn error_fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("errors")
        .join(name)
}

#[test]
fn test_explicit_qos_satisfies_socket_requirements() {
    // F18: Link has explicit QoS that meets socket requirements → Success
    let path = fixture_path("explicit_satisfies.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with explicit QoS satisfying requirements should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_explicit_qos_fails_reliability_requirement() {
    // F18: Link QoS is BestEffort but socket requires Reliable → Error
    let path = error_fixture_path("qos_reliability_mismatch.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Plan with insufficient reliability should fail compilation"
    );

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("reliability"),
        "Error message should mention reliability: {}",
        err_msg
    );
}

#[test]
fn test_explicit_qos_fails_durability_requirement() {
    // F18: Link QoS is Volatile but socket requires TransientLocal → Error
    let path = error_fixture_path("qos_durability_mismatch.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Plan with insufficient durability should fail compilation"
    );

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("durability"),
        "Error message should mention durability: {}",
        err_msg
    );
}

#[test]
fn test_explicit_qos_fails_depth_requirement() {
    // F18: Link depth=10 but socket requires depth≥20 → Error
    let path = error_fixture_path("qos_depth_insufficient.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Plan with insufficient depth should fail compilation"
    );

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("depth"),
        "Error message should mention depth: {}",
        err_msg
    );
}

#[test]
fn test_derived_qos_from_single_socket_requirement() {
    // F18: No explicit link QoS, derives from single socket requirement
    let path = fixture_path("derived_from_requirements.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with derived QoS from requirements should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_derived_qos_from_multiple_socket_requirements() {
    // F18: No explicit link QoS, derives strictest from multiple sockets
    let path = fixture_path("mixed_requirements.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with derived QoS from multiple requirements should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_derived_qos_defaults_when_no_requirements() {
    // F18: No explicit link QoS, no socket requirements → Uses ROS 2 defaults
    let path = fixture_path("defaults_no_requirements.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with no QoS requirements should compile successfully with defaults: {:?}",
        result.err()
    );
}

#[test]
fn test_qos_preset_expansion_and_validation() {
    // F18: Link uses preset (e.g., SensorData), validates against requirements
    let path = fixture_path("preset_validation.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with QoS preset should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_mixed_pub_sub_requirements() {
    // F18: Multiple publishers and subscribers with different requirements
    let path = fixture_path("mixed_requirements.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with mixed pub/sub requirements should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_qos_error_message_quality() {
    // F18: Verify error messages include helpful information
    let path = error_fixture_path("qos_reliability_mismatch.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(result.is_err(), "Should fail compilation");

    let err = result.unwrap_err();
    let err_msg = err.to_string();

    // Check that error message contains useful information
    assert!(
        err_msg.contains("QoS"),
        "Error should mention QoS: {}",
        err_msg
    );
    assert!(
        err_msg.contains("reliability"),
        "Error should mention specific policy: {}",
        err_msg
    );
    assert!(
        err_msg.contains("data_link") || err_msg.contains("link"),
        "Error should mention link name: {}",
        err_msg
    );
}
