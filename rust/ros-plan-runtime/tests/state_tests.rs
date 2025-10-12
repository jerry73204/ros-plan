use indexmap::IndexMap;
use ros_plan_format::{expr::Value, parameter::ParamName};
use ros_plan_runtime::RuntimeState;
use std::time::Duration;

#[test]
fn test_runtime_state_creation() {
    let mut params = IndexMap::new();
    params.insert("test_param".parse::<ParamName>().unwrap(), Value::I64(42));

    let state = RuntimeState::new(params.clone());

    assert_eq!(state.parameters.len(), 1);
    assert!(state
        .parameters
        .contains_key(&"test_param".parse::<ParamName>().unwrap()));
}

#[test]
fn test_runtime_state_uptime() {
    let params = IndexMap::new();
    let state = RuntimeState::new(params);

    // Uptime should be very small initially
    let uptime = state.uptime();
    assert!(uptime < Duration::from_secs(1));

    // Wait a bit
    std::thread::sleep(Duration::from_millis(10));

    // Uptime should have increased
    let uptime2 = state.uptime();
    assert!(uptime2 > uptime);
    assert!(uptime2 >= Duration::from_millis(10));
}

#[test]
fn test_runtime_state_empty_parameters() {
    let params = IndexMap::new();
    let state = RuntimeState::new(params);

    assert_eq!(state.parameters.len(), 0);
}

#[test]
fn test_runtime_state_multiple_parameters() {
    let mut params = IndexMap::new();
    params.insert("param1".parse::<ParamName>().unwrap(), Value::Bool(true));
    params.insert("param2".parse::<ParamName>().unwrap(), Value::F64(123.45));
    params.insert(
        "param3".parse::<ParamName>().unwrap(),
        Value::String("test".to_string()),
    );

    let state = RuntimeState::new(params);

    assert_eq!(state.parameters.len(), 3);
    assert!(state
        .parameters
        .contains_key(&"param1".parse::<ParamName>().unwrap()));
    assert!(state
        .parameters
        .contains_key(&"param2".parse::<ParamName>().unwrap()));
    assert!(state
        .parameters
        .contains_key(&"param3".parse::<ParamName>().unwrap()));
}
