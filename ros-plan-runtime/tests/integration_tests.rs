/// Integration tests for the runtime system
///
/// Note: These tests focus on API integration and state management.
/// Full end-to-end tests with actual ROS 2 nodes require a ROS 2 installation.
use indexmap::IndexMap;
use ros_plan_format::{expr::Value, parameter::ParamName};
use ros_plan_runtime::{
    EventLog, EventType, RestartPolicy, RuntimeConfig, RuntimeMetrics, RuntimeState,
};
use std::{path::PathBuf, time::Duration};

/// Test basic runtime state initialization
#[test]
fn test_runtime_state_initialization() {
    let mut params = IndexMap::new();
    params.insert("test_param".parse::<ParamName>().unwrap(), Value::I64(42));

    let state = RuntimeState::new(params.clone());

    assert_eq!(state.parameters.len(), 1);
    // Check parameter exists
    let param_value = state
        .parameters
        .get(&"test_param".parse::<ParamName>().unwrap());
    assert!(param_value.is_some());
    // Check value using match since Value doesn't implement PartialEq
    match param_value.unwrap() {
        Value::I64(v) => assert_eq!(*v, 42),
        _ => panic!("Expected I64 value"),
    }
    assert!(state.uptime().as_secs() < 1);
}

/// Test runtime configuration with different restart policies
#[test]
fn test_runtime_config_restart_policies() {
    let config_never = RuntimeConfig {
        restart_policy: RestartPolicy::Never,
        graceful_shutdown_timeout: Duration::from_secs(5),
        startup_timeout: Duration::from_secs(10),
    };

    assert!(matches!(config_never.restart_policy, RestartPolicy::Never));
    assert_eq!(config_never.graceful_shutdown_timeout.as_secs(), 5);

    let config_always = RuntimeConfig {
        restart_policy: RestartPolicy::Always {
            backoff: Duration::from_secs(2),
        },
        graceful_shutdown_timeout: Duration::from_secs(3),
        startup_timeout: Duration::from_secs(15),
    };

    assert!(matches!(
        config_always.restart_policy,
        RestartPolicy::Always { .. }
    ));

    let config_on_failure = RuntimeConfig {
        restart_policy: RestartPolicy::OnFailure {
            max_retries: 3,
            backoff: Duration::from_secs(1),
        },
        graceful_shutdown_timeout: Duration::from_secs(5),
        startup_timeout: Duration::from_secs(10),
    };

    if let RestartPolicy::OnFailure {
        max_retries,
        backoff,
    } = config_on_failure.restart_policy
    {
        assert_eq!(max_retries, 3);
        assert_eq!(backoff.as_secs(), 1);
    } else {
        panic!("Expected OnFailure policy");
    }
}

/// Test event log integration
#[test]
fn test_event_log_integration() {
    let mut event_log = EventLog::new(100);

    // Log runtime start
    event_log.log(EventType::RuntimeStart, None, "Runtime started".to_string());

    // Log node events
    let node_id = "test_node".parse().unwrap();
    event_log.log(
        EventType::NodeStart,
        Some(node_id),
        "Node test_node started".to_string(),
    );

    let node_id2 = "test_node".parse().unwrap();
    event_log.log(
        EventType::NodeStop,
        Some(node_id2),
        "Node test_node stopped".to_string(),
    );

    assert_eq!(event_log.events().len(), 3);

    // Filter by type
    let node_starts = event_log.events_by_type(&EventType::NodeStart);
    assert_eq!(node_starts.len(), 1);

    // Get recent events
    let recent = event_log.recent(2);
    assert_eq!(recent.len(), 2);
}

/// Test metrics collection integration
#[test]
fn test_metrics_collection_integration() {
    let mut metrics = RuntimeMetrics::new();

    let node1 = "node1".parse().unwrap();
    let node2 = "node2".parse().unwrap();

    // Simulate node lifecycle
    metrics.record_start(&node1);
    metrics.record_start(&node2);

    assert_eq!(metrics.total_starts, 2);

    metrics.record_crash(&node1);
    metrics.record_restart(&node1);
    metrics.record_start(&node1);

    assert_eq!(metrics.total_crashes, 1);
    assert_eq!(metrics.total_restarts, 1);
    assert_eq!(metrics.total_starts, 3);

    // Check per-node metrics
    let node1_metrics = metrics.get_node_metrics(&node1).unwrap();
    assert_eq!(node1_metrics.start_count, 2);
    assert_eq!(node1_metrics.crash_count, 1);
    assert_eq!(node1_metrics.restart_count, 1);

    let node2_metrics = metrics.get_node_metrics(&node2).unwrap();
    assert_eq!(node2_metrics.start_count, 1);
    assert_eq!(node2_metrics.crash_count, 0);
}

/// Test parameter update workflow (without actual ROS nodes)
#[test]
fn test_parameter_update_workflow() {
    let mut params = IndexMap::new();
    params.insert("rate".parse::<ParamName>().unwrap(), Value::I64(10));

    let mut state = RuntimeState::new(params);

    // Simulate parameter update
    state
        .parameters
        .insert("rate".parse::<ParamName>().unwrap(), Value::I64(20));

    // Check parameter updated
    let param_value = state.parameters.get(&"rate".parse::<ParamName>().unwrap());
    assert!(param_value.is_some());
    match param_value.unwrap() {
        Value::I64(v) => assert_eq!(*v, 20),
        _ => panic!("Expected I64 value"),
    }

    // Log the parameter update
    state.event_log.log(
        EventType::ParameterUpdate,
        None,
        "Parameter rate updated to 20".to_string(),
    );

    let param_updates = state.event_log.events_by_type(&EventType::ParameterUpdate);
    assert_eq!(param_updates.len(), 1);
}

/// Test launch tracker integration
#[test]
fn test_launch_tracker_integration() {
    let mut state = RuntimeState::new(IndexMap::new());

    // Register a launch include
    let mut args = IndexMap::new();
    args.insert("device".to_string(), "/dev/video0".to_string());

    state.launch_tracker.register_include(
        "camera".to_string(),
        PathBuf::from("/path/to/camera.launch.py"),
        args,
    );

    // Register nodes from the include
    state
        .launch_tracker
        .register_node("camera", "camera_node".parse().unwrap());
    state
        .launch_tracker
        .register_node("camera", "image_proc_node".parse().unwrap());

    // Check tracking
    let include = state.launch_tracker.includes.get("camera").unwrap();
    assert_eq!(include.node_idents.len(), 2);
    assert_eq!(
        include.file_path,
        PathBuf::from("/path/to/camera.launch.py")
    );

    // Check node source
    let source = state
        .launch_tracker
        .get_node_source(&"camera_node".parse().unwrap());
    assert_eq!(source, Some("camera"));
}

/// Test program diffing integration
#[test]
fn test_program_diff_integration() {
    // This test demonstrates the diff workflow without compiling actual plans
    // In a real scenario, you would compile two versions of a plan and diff them

    let params1 = IndexMap::new();
    let state1 = RuntimeState::new(params1);

    let mut params2 = IndexMap::new();
    params2.insert("rate".parse::<ParamName>().unwrap(), Value::I64(20));
    let state2 = RuntimeState::new(params2);

    // Verify parameter change tracked
    assert_ne!(state1.parameters.len(), state2.parameters.len());
}

/// Test status reporting integration
#[test]
fn test_status_reporting_integration() {
    use ros_plan_runtime::{IncludeStatus, NodeSource, NodeState, NodeStatus, RuntimeStatus};

    let mut params = IndexMap::new();
    params.insert("test_param".parse::<ParamName>().unwrap(), Value::I64(42));

    let status = RuntimeStatus {
        uptime: Duration::from_secs(120),
        parameters: params,
        nodes: vec![NodeStatus {
            ident: "test_node".parse().unwrap(),
            name: "test_node".to_string(),
            namespace: Some("/test".to_string()),
            state: NodeState::Running,
            pid: Some(1234),
            uptime: Some(Duration::from_secs(100)),
            restart_count: 0,
            source: NodeSource::Plan,
        }],
        includes: vec![IncludeStatus {
            name: "camera".to_string(),
            file_path: PathBuf::from("/path/to/camera.launch.py"),
            node_count: 2,
            parameter_deps: vec!["camera_device".parse().unwrap()],
        }],
    };

    // Test table formatting
    let table = status.format_table();
    assert!(table.contains("Runtime Status"));
    assert!(table.contains("test_node"));
    assert!(table.contains("Running"));

    // Test JSON formatting
    let json = status.format_json().unwrap();
    assert!(json.contains("test_node"));
    assert!(json.contains("\"uptime\""));

    // Test filtering
    assert_eq!(status.running_count(), 1);
    assert_eq!(status.crashed_count(), 0);
}

/// Test multiple operations on runtime state
#[test]
fn test_runtime_state_multiple_operations() {
    let mut params = IndexMap::new();
    params.insert("param1".parse::<ParamName>().unwrap(), Value::I64(1));
    params.insert(
        "param2".parse::<ParamName>().unwrap(),
        Value::String("test".to_string()),
    );

    let mut state = RuntimeState::new(params);

    // Record various events
    state
        .event_log
        .log(EventType::RuntimeStart, None, "Runtime started".to_string());

    let node1 = "node1".parse().unwrap();
    state.metrics.record_start(&node1);
    state.event_log.log(
        EventType::NodeStart,
        Some(node1),
        "Node started".to_string(),
    );

    // Update parameter
    state
        .parameters
        .insert("param1".parse::<ParamName>().unwrap(), Value::I64(2));
    state.event_log.log(
        EventType::ParameterUpdate,
        None,
        "param1 updated".to_string(),
    );

    // Crash and restart
    let node1 = "node1".parse().unwrap();
    state.metrics.record_crash(&node1);
    state.event_log.log(
        EventType::NodeCrash,
        Some(node1),
        "Node crashed".to_string(),
    );

    let node1 = "node1".parse().unwrap();
    state.metrics.record_restart(&node1);
    state.metrics.record_start(&node1);
    state.event_log.log(
        EventType::NodeRestart,
        Some(node1),
        "Node restarted".to_string(),
    );

    // Verify state
    assert_eq!(state.event_log.events().len(), 5);
    assert_eq!(state.metrics.total_starts, 2);
    assert_eq!(state.metrics.total_crashes, 1);
    assert_eq!(state.metrics.total_restarts, 1);

    let node_metrics = state.metrics.get_node_metrics(&"node1".parse().unwrap());
    assert!(node_metrics.is_some());
    assert_eq!(node_metrics.unwrap().crash_count, 1);
}

/// Test metrics reset functionality
#[test]
fn test_metrics_reset() {
    let mut metrics = RuntimeMetrics::new();

    let node1 = "node1".parse().unwrap();
    metrics.record_start(&node1);
    metrics.record_crash(&node1);
    metrics.record_restart(&node1);

    assert_eq!(metrics.total_starts, 1);
    assert_eq!(metrics.total_crashes, 1);
    assert_eq!(metrics.total_restarts, 1);

    // Reset metrics
    metrics.reset();

    assert_eq!(metrics.total_starts, 0);
    assert_eq!(metrics.total_crashes, 0);
    assert_eq!(metrics.total_restarts, 0);
    assert!(metrics.all_node_metrics().is_empty());
}

/// Test event log size limit
#[test]
fn test_event_log_size_limit() {
    let mut event_log = EventLog::new(5);

    // Add more events than the limit
    for i in 0..10 {
        event_log.log(
            EventType::NodeStart,
            Some(format!("node{}", i).parse().unwrap()),
            format!("Event {}", i),
        );
    }

    // Should only keep the last 5
    assert_eq!(event_log.events().len(), 5);

    // Verify oldest events were removed
    let events = event_log.events();
    assert!(events[0].message.contains("Event 5"));
    assert!(events[4].message.contains("Event 9"));
}
