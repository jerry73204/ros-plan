use ros_plan_runtime::{RestartPolicy, RuntimeConfig};
use std::time::Duration;

#[test]
fn test_default_runtime_config() {
    let config = RuntimeConfig::default();

    // Check default timeout values
    assert_eq!(config.graceful_shutdown_timeout, Duration::from_secs(5));
    assert_eq!(config.startup_timeout, Duration::from_secs(10));

    // Check default restart policy
    match config.restart_policy {
        RestartPolicy::OnFailure { max_retries, backoff } => {
            assert_eq!(max_retries, 3);
            assert_eq!(backoff, Duration::from_secs(1));
        }
        _ => panic!("Expected OnFailure restart policy as default"),
    }
}

#[test]
fn test_restart_policy_never() {
    let policy = RestartPolicy::Never;

    // Serialize and check format
    let json = serde_json::to_string(&policy).unwrap();
    assert!(json.contains("Never"));
}

#[test]
fn test_restart_policy_on_failure() {
    let policy = RestartPolicy::OnFailure {
        max_retries: 5,
        backoff: Duration::from_secs(2),
    };

    // Serialize and deserialize
    let json = serde_json::to_string(&policy).unwrap();
    let deserialized: RestartPolicy = serde_json::from_str(&json).unwrap();

    match deserialized {
        RestartPolicy::OnFailure { max_retries, backoff } => {
            assert_eq!(max_retries, 5);
            assert_eq!(backoff, Duration::from_secs(2));
        }
        _ => panic!("Expected OnFailure"),
    }
}

#[test]
fn test_restart_policy_always() {
    let policy = RestartPolicy::Always {
        backoff: Duration::from_millis(500),
    };

    // Serialize and deserialize
    let json = serde_json::to_string(&policy).unwrap();
    let deserialized: RestartPolicy = serde_json::from_str(&json).unwrap();

    match deserialized {
        RestartPolicy::Always { backoff } => {
            assert_eq!(backoff, Duration::from_millis(500));
        }
        _ => panic!("Expected Always"),
    }
}

#[test]
fn test_custom_runtime_config() {
    let config = RuntimeConfig {
        restart_policy: RestartPolicy::Never,
        graceful_shutdown_timeout: Duration::from_secs(10),
        startup_timeout: Duration::from_secs(30),
    };

    assert_eq!(config.graceful_shutdown_timeout, Duration::from_secs(10));
    assert_eq!(config.startup_timeout, Duration::from_secs(30));
    matches!(config.restart_policy, RestartPolicy::Never);
}
