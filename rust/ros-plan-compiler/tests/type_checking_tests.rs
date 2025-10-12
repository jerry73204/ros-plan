// F17 Integration Tests
// Tests for type compatibility checking

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
fn test_type_compatible() {
    // F17: Link with compatible socket types should succeed
    let path = fixture_path("type_compatible.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Link with compatible types should compile successfully: {:?}",
        result.err()
    );
}

#[test]
fn test_type_unspecified_compatible() {
    // F17: Socket without type should be compatible with any link type
    let path = fixture_path("type_unspecified_compatible.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_ok(),
        "Socket without type should be compatible: {:?}",
        result.err()
    );
}

#[test]
fn test_type_mismatch_pubsub() {
    // F17: PubSub link with mismatched socket type should fail
    let path = error_fixture_path("type_mismatch_pubsub.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(result.is_err(), "Link with mismatched types should fail");

    let err = result.unwrap_err();
    let err_msg = format!("{}", err);

    match err {
        Error::LinkSocketTypeMismatch {
            link,
            link_type,
            socket,
            socket_type,
        } => {
            assert!(link.contains("chatter"));
            assert_eq!(link_type, "std_msgs/msg/String");
            assert!(socket.contains("talker") && socket.contains("output"));
            assert_eq!(socket_type, "std_msgs/msg/Int32");
        }
        _ => panic!("Expected LinkSocketTypeMismatch, got: {:?}", err),
    }

    assert!(
        err_msg.contains("type mismatch") && err_msg.contains("Int32"),
        "Error message should explain the type mismatch: {}",
        err_msg
    );
}

#[test]
fn test_type_mismatch_service() {
    // F17: Service link with mismatched socket type should fail
    let path = error_fixture_path("type_mismatch_service.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Service link with mismatched types should fail"
    );

    let err = result.unwrap_err();
    let err_msg = format!("{}", err);

    match err {
        Error::LinkSocketTypeMismatch {
            link,
            link_type,
            socket,
            socket_type,
        } => {
            assert!(link.contains("add_service"));
            assert_eq!(link_type, "example_interfaces/srv/AddTwoInts");
            assert!(socket.contains("client") && socket.contains("add"));
            assert_eq!(socket_type, "std_srvs/srv/SetBool");
        }
        _ => panic!("Expected LinkSocketTypeMismatch, got: {:?}", err),
    }

    assert!(
        err_msg.contains("type mismatch") && err_msg.contains("SetBool"),
        "Error message should explain the type mismatch: {}",
        err_msg
    );
}

#[test]
fn test_multi_socket_type_consistency() {
    // F17: All sockets in a link must have the same type
    let path = error_fixture_path("multi_source_type_mismatch.yaml");
    let compiler = Compiler::new();
    let result = compiler.compile(&path, IndexMap::new());

    assert!(
        result.is_err(),
        "Multi-source link with mismatched types should fail"
    );

    let err = result.unwrap_err();
    let err_msg = format!("{}", err);

    match err {
        Error::LinkSocketTypeMismatch {
            link, socket_type, ..
        } => {
            assert!(link.contains("multi_pub"));
            assert_eq!(socket_type, "std_msgs/msg/Int32");
        }
        _ => panic!("Expected LinkSocketTypeMismatch, got: {:?}", err),
    }

    assert!(
        err_msg.contains("type mismatch"),
        "Error message should explain the type mismatch: {}",
        err_msg
    );
}
