# ros2plan (Rust CLI)

**Location:** `ros2plan/`

**Purpose:** Command-line interface for compiling and running ROS-Plan files

**Status:** ✅ Production Ready
- **Tests:** 18 passing
- **Features:** All core features complete
- **Coverage:** compile command, run command, argument parsing, output specification

---

## Features

### F40: Compile Command ✅

- Compile plan files to ROS 2 launch format
- Argument passing: `key=value` or `key:type=value`
- Output file specification with `-o`
- Error reporting and validation

### F41: Run Command ✅

- Execute plans with runtime process management
- Signal handling (SIGINT/SIGTERM)
- Graceful shutdown
- Runtime state tracking

### Additional Features

- 18 CLI tests covering all command variations
- Type annotations for parameters (!i64, !f64, !bool, !str, !path)
- Complex value parsing (strings with spaces, negative numbers)
- Clear error messages for invalid inputs
