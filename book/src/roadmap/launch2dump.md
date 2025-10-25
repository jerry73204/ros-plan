# launch2dump (Python)

**Location:** `python/launch2dump/`

**Purpose:** Extract metadata from ROS 2 launch files without spawning processes

**Status:** ✅ Production Ready
- **Tests:** 25 passing
- **Features:** 6/6 complete (100%)
- **Coverage:** Launch file visiting, node extraction, parameter capture, JSON/YAML output

---

## Phase 1: Launch Integration ✅

**Goal:** Support loading ROS 2 launch files into plan files via PyO3-based Python integration

**Status:** ✅ Complete (6/6 features)

**Features:** F32 ✅, F33 ✅, F34 ✅, F35 ✅, F36 ✅, F39 ✅

**Overview:** Integrates Python-based ROS 2 launch file loading directly into the ros-plan compiler. Using PyO3 and a launch2dump utility, the compiler can now load and merge launch files at compile-time, extracting node metadata and incorporating it into the compiled program.

**Completed Features:**
- ✅ F32: UV Python Package Manager Integration
- ✅ F33: Launch2Dump Python Loader API
- ✅ F34: Launch2Dump CLI Tool
- ✅ F35: Serialization Format Improvements
- ✅ F36: PyO3 Integration for Launch Loading
- ✅ F39: Launch Integration Tests

**Key Achievements:**
- Python launch file loading via PyO3 FFI
- Metadata extraction and serialization using JSON
- CLI tool for debugging launch file metadata
- Comprehensive test coverage for launch integration
- Fixed YAML serialization for `!lua` tags
