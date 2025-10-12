// Phase 3 Integration Tests
// Tests for F3, F7, F10, F15, F30

use indexmap::IndexMap;
use ros_plan_compiler::{Compiler, Error};
use std::path::PathBuf;

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

#[test]
fn test_single_source_with_explicit_topic() {
    // F3: Plan socket topic field parsing and usage
    let path = fixture_path("link_explicit_topic.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Single source with explicit topic should compile: {:?}",
        result.err()
    );

    let program = result.unwrap();
    let yaml = serde_yaml::to_string(&program).unwrap();

    // Verify the topic is set correctly
    assert!(
        yaml.contains("/camera/image_raw"),
        "Compiled program should contain the explicit topic"
    );
}

#[test]
fn test_multi_source_with_explicit_topic_succeeds() {
    // F7/F15: Multi-source with topic should succeed
    let path = fixture_path("link_multi_source_valid.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Multi-source with explicit topic should succeed: {:?}",
        result.err()
    );

    let program = result.unwrap();
    let yaml = serde_yaml::to_string(&program).unwrap();

    // Verify the shared topic is used
    assert!(
        yaml.contains("/tf"),
        "Compiled program should use the shared /tf topic"
    );
}

#[test]
fn test_multi_source_without_topic_fails() {
    // F7/F15: Multi-source without topic should fail with helpful error
    let path = fixture_path("link_multi_source_error.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Multi-source without topic should fail validation"
    );

    let err = result.unwrap_err();
    let err_msg = format!("{}", err);

    // Verify it's the right error type
    match err {
        Error::MultipleSourcesRequireExplicitTopic { link, source_count } => {
            assert_eq!(source_count, 2, "Should report 2 sources");
            assert!(
                link.as_str().contains("sensor_fusion"),
                "Error should reference the link name"
            );
        }
        _ => panic!(
            "Expected MultipleSourcesRequireExplicitTopic error, got: {:?}",
            err
        ),
    }

    // Verify error message is helpful
    assert!(
        err_msg.contains("sensor_fusion"),
        "Error should mention link name"
    );
    assert!(err_msg.contains("2 sources"), "Error should mention count");
    assert!(
        err_msg.contains("topic"),
        "Error should suggest adding topic field"
    );
}

#[test]
fn test_plan_socket_topic_resolution() {
    // F10: Plan socket topic should be used when link doesn't specify one
    let path = fixture_path("plan_socket_topic_resolution.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan socket topic resolution should work: {:?}",
        result.err()
    );

    let program = result.unwrap();
    let yaml = serde_yaml::to_string(&program).unwrap();

    // Verify the plan socket's topic is used
    assert!(
        yaml.contains("/sensors/camera/image"),
        "Compiled program should use plan socket's topic"
    );
}

#[test]
fn test_error_message_quality() {
    // F15: Verify error messages are clear and actionable
    let path = fixture_path("link_multi_source_error.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(result.is_err());

    let err_msg = format!("{}", result.unwrap_err());

    // Check for helpful error message components
    assert!(
        err_msg.contains("sensor_fusion"),
        "Should include link name"
    );
    assert!(
        err_msg.contains("sources"),
        "Should mention multiple sources"
    );
    assert!(err_msg.contains("topic"), "Should suggest topic attribute");

    // Check for suggestion/example in error message
    assert!(
        err_msg.contains("Suggestion") || err_msg.contains("Add"),
        "Should include helpful suggestion"
    );
}
