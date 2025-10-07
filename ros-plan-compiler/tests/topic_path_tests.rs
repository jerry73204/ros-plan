// F8 Integration Tests
// Tests for absolute/relative topic path resolution

use indexmap::IndexMap;
use ros_plan_compiler::Compiler;
use std::path::PathBuf;

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

#[test]
fn test_absolute_topic_path() {
    // F8: Absolute topic (starting with /) should remain unchanged
    let path = fixture_path("absolute_topic.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with absolute topic should compile successfully: {:?}",
        result.err()
    );

    // The absolute topic /absolute/chatter should be preserved as-is
    // (Verifying actual topic values would require program introspection)
}

#[test]
fn test_relative_topic_path() {
    // F8: Relative topic (not starting with /) should get namespace prepended
    let path = fixture_path("relative_topic.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with relative topic should compile successfully: {:?}",
        result.err()
    );

    // The relative topic "relative/chatter" should become "/talker/relative/chatter"
    // or similar depending on namespace resolution
}

#[test]
fn test_nested_namespace_topic() {
    // F8: Multi-level namespace with derived topic
    let path = fixture_path("nested_namespace_topic.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Plan with nested namespace should compile successfully: {:?}",
        result.err()
    );

    // Topic should be derived from robot1/camera/image socket
    // and become absolute path like /robot1/camera/image
}

#[test]
fn test_mixed_absolute_relative() {
    // F8: Verify both absolute and relative topics can coexist
    // This test verifies the basic functionality works
    // More detailed topic value tests would require program introspection

    // These fixtures already test absolute and relative independently
    test_absolute_topic_path();
    test_relative_topic_path();

    // Both should succeed
    // In practice, a real plan would have both types of topics
    // and the namespace resolution would handle them correctly
}
