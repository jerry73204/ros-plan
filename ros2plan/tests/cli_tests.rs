/// CLI integration tests
///
/// Note: These tests verify CLI argument parsing and command structure.
/// Full end-to-end CLI tests with IPC require implementing the IPC server.
use clap::Parser;
use ros2plan::cli::{ArgAssign, Cli};

/// Test CLI command parsing for compile
#[test]
fn test_compile_command_parsing() {
    let args = vec!["ros2plan", "compile", "test.yaml"];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_ok());

    if let Ok(Cli::Compile(compile_args)) = cli {
        assert_eq!(compile_args.plan_file.to_str().unwrap(), "test.yaml");
        assert!(compile_args.args.is_none());
        assert!(compile_args.output_file.is_none());
    } else {
        panic!("Expected Compile command");
    }
}

/// Test CLI command parsing for compile with output file
#[test]
fn test_compile_with_output() {
    let args = vec!["ros2plan", "compile", "test.yaml", "-o", "output.yaml"];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_ok());

    if let Ok(Cli::Compile(compile_args)) = cli {
        assert_eq!(compile_args.plan_file.to_str().unwrap(), "test.yaml");
        assert!(compile_args.output_file.is_some());
        assert_eq!(
            compile_args.output_file.unwrap().to_str().unwrap(),
            "output.yaml"
        );
    } else {
        panic!("Expected Compile command");
    }
}

/// Test CLI command parsing for run
#[test]
fn test_run_command_parsing() {
    let args = vec!["ros2plan", "run", "test.yaml"];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_ok());

    if let Ok(Cli::Run(run_args)) = cli {
        assert_eq!(run_args.plan_file.to_str().unwrap(), "test.yaml");
        assert!(run_args.args.is_none());
    } else {
        panic!("Expected Run command");
    }
}

/// Test argument assignment parsing - simple integer
#[test]
fn test_arg_assign_integer() {
    let arg = "rate=10".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "rate");

    // Value should parse as integer
    use ros_plan_format::expr::Value;
    match arg.value {
        Value::I64(v) => assert_eq!(v, 10),
        _ => panic!("Expected I64 value"),
    }
}

/// Test argument assignment parsing - string
#[test]
fn test_arg_assign_string() {
    let arg = "device=/dev/video0".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "device");

    use ros_plan_format::expr::Value;
    match arg.value {
        Value::String(s) => assert_eq!(s, "/dev/video0"),
        _ => panic!("Expected String value"),
    }
}

/// Test argument assignment parsing - with type annotation
#[test]
fn test_arg_assign_with_type() {
    let arg = "rate:i64=10".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "rate");

    use ros_plan_format::expr::Value;
    match arg.value {
        Value::I64(v) => assert_eq!(v, 10),
        _ => panic!("Expected I64 value"),
    }
}

/// Test argument assignment parsing - float
#[test]
fn test_arg_assign_float() {
    let arg = "rate:f64=10.5".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "rate");

    use ros_plan_format::expr::Value;
    match arg.value {
        Value::F64(v) => assert!((v - 10.5).abs() < 0.001),
        _ => panic!("Expected F64 value"),
    }
}

/// Test argument assignment parsing - boolean
#[test]
fn test_arg_assign_boolean() {
    let arg = "enabled:bool=true".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "enabled");

    use ros_plan_format::expr::Value;
    match arg.value {
        Value::Bool(v) => assert!(v),
        _ => panic!("Expected Bool value"),
    }
}

/// Test argument assignment parsing - invalid format
#[test]
fn test_arg_assign_invalid_format() {
    let arg = "invalid".parse::<ArgAssign>();
    assert!(arg.is_err());

    let err = arg.unwrap_err().to_string();
    assert!(err.contains("name=value"));
}

/// Test argument assignment parsing - invalid parameter name
#[test]
fn test_arg_assign_invalid_name() {
    // Parameter names with invalid characters should fail
    let arg = "invalid-name=10".parse::<ArgAssign>();
    assert!(arg.is_err());
}

/// Test CLI with multiple arguments
#[test]
fn test_cli_with_multiple_args() {
    let args = vec![
        "ros2plan",
        "compile",
        "test.yaml",
        "rate=10",
        "device=/dev/video0",
    ];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_ok());

    if let Ok(Cli::Compile(compile_args)) = cli {
        assert!(compile_args.args.is_some());
        let args = compile_args.args.unwrap();
        assert_eq!(args.len(), 2);
        assert_eq!(args[0].name.as_str(), "rate");
        assert_eq!(args[1].name.as_str(), "device");
    } else {
        panic!("Expected Compile command");
    }
}

/// Test help command
#[test]
fn test_help_command() {
    let args = vec!["ros2plan", "--help"];
    let cli = Cli::try_parse_from(args);
    // Help exits with error in clap
    assert!(cli.is_err());
}

/// Test missing required arguments
#[test]
fn test_missing_plan_file() {
    let args = vec!["ros2plan", "compile"];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_err());
}

/// Test invalid command
#[test]
fn test_invalid_command() {
    let args = vec!["ros2plan", "invalid"];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_err());
}

/// Test compile with long output flag
#[test]
fn test_compile_with_long_output_flag() {
    let args = vec![
        "ros2plan",
        "compile",
        "test.yaml",
        "--output-file",
        "out.yaml",
    ];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_ok());

    if let Ok(Cli::Compile(compile_args)) = cli {
        assert!(compile_args.output_file.is_some());
        assert_eq!(
            compile_args.output_file.unwrap().to_str().unwrap(),
            "out.yaml"
        );
    }
}

/// Test run command with arguments
#[test]
fn test_run_with_args() {
    let args = vec![
        "ros2plan",
        "run",
        "test.yaml",
        "rate=20",
        "enabled:bool=true",
    ];
    let cli = Cli::try_parse_from(args);
    assert!(cli.is_ok());

    if let Ok(Cli::Run(run_args)) = cli {
        assert_eq!(run_args.plan_file.to_str().unwrap(), "test.yaml");
        assert!(run_args.args.is_some());

        let args = run_args.args.unwrap();
        assert_eq!(args.len(), 2);
        assert_eq!(args[0].name.as_str(), "rate");
        assert_eq!(args[1].name.as_str(), "enabled");
    }
}

/// Test argument with complex string value
#[test]
fn test_arg_with_complex_string() {
    let arg = "namespace=/robot/camera".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "namespace");

    use ros_plan_format::expr::Value;
    match arg.value {
        Value::String(s) => assert_eq!(s, "/robot/camera"),
        _ => panic!("Expected String value"),
    }
}

/// Test argument with negative number
#[test]
fn test_arg_with_negative_number() {
    let arg = "offset:i64=-5".parse::<ArgAssign>();
    assert!(arg.is_ok());

    let arg = arg.unwrap();
    assert_eq!(arg.name.as_str(), "offset");

    use ros_plan_format::expr::Value;
    match arg.value {
        Value::I64(v) => assert_eq!(v, -5),
        _ => panic!("Expected I64 value"),
    }
}
