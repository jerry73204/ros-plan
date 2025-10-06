# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS-Plan is a ROS 2 launch plan compiler that processes YAML-based node configuration plans into executable ROS 2 launch files. The project consists of both Rust and Python components that work together to compile and process ROS 2 node configurations.

## Core Architecture

### Rust Crates (Workspace Structure)

1. **ros-plan-format** - Defines the data structures and parsing logic for ROS plan files (YAML format)
   - Handles node definitions, links, sockets, parameters, and QoS settings
   - Key types: Plan, Node, Link, NodeSocket, PlanSocket

2. **ros-plan-compiler** - Core compilation logic that transforms plan files into executable programs
   - Main entry point: `Compiler::compile()`
   - Handles evaluation, linking, and program generation
   - Uses Lua scripting for dynamic evaluation

3. **ros2plan** - CLI tool for compiling plan files
   - Main binary that users interact with
   - Usage: `ros2plan compile <plan-file> [args]`

4. **ros-utils** - Utility functions for ROS 2 package and executable resolution
   - Finds ROS 2 packages, executables, and configurations
   - Generates command-line arguments for nodes

### Python Components

- **launch2dump** - Tool for converting ROS 2 launch files to plan format
- Uses Rye for Python dependency management

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
# Install Python dependencies (uses Rye)
rye sync

# Run launch2dump
rye run launch2dump
```

## Important Implementation Details

- The compiler uses mlua (Lua) for dynamic evaluation of expressions in plan files
- Plans support parameter substitution and conditional compilation using the `when` clause
- The system resolves ROS 2 packages and executables using `ros2` CLI commands
- Node command-line generation follows ROS 2 conventions for remapping and parameter passing
- The format uses custom YAML tags (!pub, !sub, !pubsub, etc.) for socket type definitions