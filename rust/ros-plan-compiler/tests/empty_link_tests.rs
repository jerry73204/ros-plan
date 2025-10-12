// F5 Integration Tests
// Tests for empty src/dst in links

use indexmap::IndexMap;
use ros_plan_compiler::{Compiler, Error};
use std::path::PathBuf;

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
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
fn test_consume_only_link_with_explicit_topic() {
    // F5: Consume-only link (empty src) with explicit topic should succeed
    let path = fixture_path("consume_only_link.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Consume-only link with explicit topic should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_publish_only_link() {
    // F5: Publish-only link (empty dst) should succeed
    let path = fixture_path("publish_only_link.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Publish-only link should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_empty_link_fails() {
    // F5: Link with both empty src and dst should fail
    let path = error_fixture_path("empty_link.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Link with both empty src and dst should fail"
    );

    let err = result.unwrap_err();
    let err_msg = format!("{}", err);

    match err {
        Error::LinkMustHaveSourceOrDestination { link } => {
            assert_eq!(link.as_str(), "/invalid_link");
        }
        _ => panic!("Expected LinkMustHaveSourceOrDestination, got: {:?}", err),
    }

    assert!(
        err_msg.contains("invalid_link") && err_msg.contains("source or a destination"),
        "Error message should explain the issue: {}",
        err_msg
    );
}

#[test]
fn test_consume_only_without_topic_fails() {
    // F5: Consume-only link without explicit topic should fail
    let path = error_fixture_path("consume_only_no_topic.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Consume-only link without explicit topic should fail"
    );

    let err = result.unwrap_err();
    let err_msg = format!("{}", err);

    match err {
        Error::EmptySourceRequiresExplicitTopic { link } => {
            assert_eq!(link.as_str(), "/consume_link");
        }
        _ => panic!("Expected EmptySourceRequiresExplicitTopic, got: {:?}", err),
    }

    assert!(
        err_msg.contains("consume_link") && err_msg.contains("topic"),
        "Error message should explain the issue: {}",
        err_msg
    );
}
