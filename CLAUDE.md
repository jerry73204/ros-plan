# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS-Plan is a comprehensive ROS 2 toolchain that provides:
1. **Declarative plan format** - YAML-based node configuration with socket/link abstraction
2. **Plan compiler** - Transforms plans into executable ROS 2 launch files
3. **Runtime system** - Process management with graceful shutdown, restart policies, and dynamic parameter updates
4. **Metadata extraction** - launch2dump tool for analyzing existing launch files

The project consists of both Rust (core compiler and runtime) and Python (launch file integration) components.

## Project Status

**Current Phase**: Phase 11 Complete (Testing & Documentation)
- ✅ 330 tests passing (73 compiler + 163 format + 47 runtime + 18 CLI + 36 ros-utils)
- ✅ Zero warnings (clippy + compiler)
- ✅ 54 features implemented across 11 phases
- ✅ Complete documentation (user guide, architecture, configuration)

**Next Phase**: Phase 12 (Launch-to-Plan Conversion) - Design phase only, no implementation yet
- See `book/src/launch2plan.md` for design document
- Multi-pass workflow with TODO stubs and pattern learning
- Preserves include hierarchy, explores all conditional branches
- **Do not implement without explicit discussion and approval**

## Core Architecture

### Rust Crates (Workspace Structure)

1. **ros-plan-format** - Data structures and parsing for ROS plan files (YAML format)
   - Handles node definitions, links, sockets, parameters, QoS settings
   - Key types: Plan, Node, Link, NodeSocket, PlanSocket
   - Custom YAML tags: !pub, !sub, !pubsub, !i64, !f64, !bool, etc.

2. **ros-plan-compiler** - Core compilation logic
   - Main entry point: `Compiler::compile()`
   - Evaluation, linking, program generation
   - Lua integration for dynamic expressions
   - Link resolution and validation

3. **ros-plan-runtime** - Process management and lifecycle
   - Node spawning with ROS 2 arguments
   - Graceful shutdown (SIGTERM → SIGKILL)
   - Restart policies: Never, Always, OnFailure
   - Dynamic parameter updates with program diffing
   - Launch include tracking and reloading
   - Status reporting (table and JSON formats)
   - Event logging with circular buffer
   - Metrics collection (starts, stops, crashes, restarts)

4. **ros2plan** - CLI tool
   - Commands: `compile`, `run` (with runtime)
   - Usage: `ros2plan compile <plan-file> [args]`
   - Argument passing: `key=value` or `key:type=value`

5. **ros-utils** - ROS 2 integration utilities
   - Package and executable resolution via `ros2` CLI
   - Node command-line generation with remapping
   - ROS 2 conventions compliance

### Python Components

- **launch2dump** - Metadata extraction from ROS 2 launch files
  - Visits launch entities without spawning processes
  - Extracts nodes, parameters, remappings, containers
  - JSON/YAML output formats
  - Uses UV for dependency management (migrated from Rye)

## Key File Formats

### Plan Files (*.yaml)
Plans define ROS 2 nodes and their connections. Example structure:
```yaml
node:
  node_name:
    pkg: package_name
    exec: executable_name
    socket:
      socket_name: !pub/!sub
link:
  link_name: !pubsub
    type: msg/type/Name
    src: ["node/socket"]
    dst: ["node/socket"]
```

## Development Commands

This project uses a Makefile for common development tasks. Use `make help` to see all available targets.

### Building
```bash
# Build all targets (uses release-with-debug profile)
make build

# Build specific crate
cargo build -p ros-plan-compiler --profile release-with-debug
```

### Testing
```bash
# Run all tests (uses cargo-nextest with release-with-debug profile)
make test

# Run tests for specific crate
cargo nextest run -p ros-plan-compiler --cargo-profile release-with-debug

# Run tests with standard cargo test
cargo test -p ros-plan-compiler --profile release-with-debug
```

### Code Quality
```bash
# Format code
make format

# Check formatting and run clippy (with -D warnings)
make lint

# Format code manually
cargo +nightly fmt

# Run clippy manually
cargo clippy --all-targets --all-features -- -D warnings
```

### Cleaning
```bash
# Clean build artifacts
make clean
```

### Running the Compiler
```bash
# Compile a plan file
cargo run --bin ros2plan -- compile examples/introduction.yaml

# With arguments
cargo run --bin ros2plan -- compile examples/eval.yaml --args key=value

# With output file
cargo run --bin ros2plan -- compile examples/introduction.yaml -o output.launch
```

### Python Development
```bash
# Install Python dependencies (launch2dump uses UV)
cd launch2dump && uv sync

# Run launch2dump
cd launch2dump && uv run launch2dump <launch-file>

# Run Python tests (requires ROS 2 sourced)
source /opt/ros/humble/setup.bash
cd launch2dump && uv run pytest
```

## Important Implementation Details

### Compiler & Format
- Uses mlua (Lua) for dynamic evaluation of expressions in plan files
- Plans support parameter substitution and conditional compilation using the `when` clause
- Custom YAML tags: !pub, !sub, !pubsub, !i64, !f64, !bool, !path, !expr
- Socket/link abstraction decouples node definitions from topic names
- Transparent includes allow multi-level socket forwarding
- Node identifiers (not names) are used for references

### Runtime System
- Spawns ROS 2 nodes as child processes using `Command`
- Graceful shutdown: SIGTERM → wait → SIGKILL
- Restart policies with configurable backoff
- Program diffing detects parameter changes for selective restarts
- Launch include tracking maps nodes to their source files
- Event logging uses a circular buffer (configurable size)
- Status reporting supports both human-readable tables and JSON

### ROS 2 Integration
- Resolves packages and executables using `ros2 pkg` and `ros2 run` commands
- Generates node command-line arguments following ROS 2 conventions
- Supports remapping, parameters, namespaces, ROS arguments
- Compatible with standard ROS 2 launch files via launch2dump

### Testing
- 330 total tests across all crates
- Uses cargo-nextest for parallel test execution
- Python tests require ROS 2 environment sourced
- All tests run with `release-with-debug` profile for speed
- Zero tolerance for clippy warnings (`-D warnings`)

## Documentation

Comprehensive documentation in `book/src/`:
- `roadmap.md` - Implementation status and feature tracking
- `plan-format.md` - Plan YAML specification
- `runtime_user_guide.md` - Runtime system usage
- `runtime_architecture.md` - Internal design
- `runtime_config.md` - Configuration reference
- `launch2plan.md` - Launch-to-plan conversion design (Phase 12, not yet implemented)

## Development Workflow

1. **Before making changes**: Read relevant documentation in `book/src/`
2. **During development**: Run `make lint` frequently to catch issues early
3. **After changes**: Always run `make test` to verify all tests pass
4. **Before committing**: Ensure `make lint && make test` passes completely
5. **Phase 12 note**: Do not implement launch2plan without explicit approval - design phase only

## Common Patterns

### Value Types in Format
```rust
// ros-plan-format uses an enum for values
pub enum Value {
    I64(i64),
    F64(f64),
    Bool(bool),
    String(String),
    Path(PathBuf),
    Expr(String),  // Lua expression
}
```

### Socket References
```yaml
# Socket reference format: node_id/socket_name
link:
  camera_image: !pubsub
    src: ["camera/image"]      # Node ID "camera", socket "image"
    dst: ["detector/image"]     # Node ID "detector", socket "image"
```

### When Clauses
```yaml
node:
  optional_node:
    pkg: my_pkg
    exec: my_node
    when: "$(enable_feature)"  # Lua expression in $()
```

### Restart Policies
```rust
pub enum RestartPolicy {
    Never,                              // No restart
    Always { backoff: Duration },       // Always restart with delay
    OnFailure {                         // Restart on non-zero exit
        max_retries: u32,
        backoff: Duration,
    },
}
```