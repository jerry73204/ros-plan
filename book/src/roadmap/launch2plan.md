## Package: launch2plan (Python)

**Location:** `python/launch2plan/`

**Purpose:** Convert ROS 2 launch files to ROS-Plan format

**Status:** 🚧 In Progress (8/13 phases complete)
- **Tests:** 68 passing (85 total across phases)
- **Features:** Phases 1-8 ✅, Phases 9-13 🔴
- **Coverage:** Branch exploration, introspection, socket inference, plan generation, arguments, conditionals, includes, metadata

### Phase 1: Foundation & Basic Visitor ✅

**Goal:** Set up project structure and basic launch file visiting

**Status:** ✅ Complete (3/3 tests passing)

**Implementation:**
- [X] Created `launch2plan` package structure
- [X] Set up pyproject.toml with dependencies
- [X] Created CLI with `convert` command
- [X] Implemented branch-exploring visitor (explores ALL branches)
- [X] Extract node metadata (package, executable, name, namespace, remappings)
- [X] Track condition expressions for `when` clause generation
- [X] Detect includes (basic - no recursion yet)

**Test Cases:** 3/3 passing
- [X] Single node discovery
- [X] Multiple nodes with remappings
- [X] Basic CLI invocation

**Deliverable:** ✅ Can visit a simple launch file and list discovered nodes

---

### Phase 2: RMW Introspection Integration ✅

**Goal:** Integrate ros2-introspect for accurate socket inference

**Status:** ✅ Complete (5/5 tests passing)

**Implementation:**
- [X] Created `introspection.py` module
- [X] Implemented `IntrospectionService` class with caching
- [X] Added `get_socket_info()` and `get_all_topics()` methods
- [X] Graceful handling of introspection failures
- [X] Cache introspection results per package::executable

**Test Cases:** 5/5 passing
- [X] Demo nodes (talker/listener)
- [X] Caching behavior
- [X] Introspection fallback
- [X] Socket direction resolution
- [X] Message type resolution

**Deliverable:** ✅ Can introspect nodes and determine socket directions + message types

---

### Phase 3: Socket Inference & TODO Generation ✅

**Goal:** Integrate introspection with node conversion, generate TODO markers for unknowns

**Status:** ✅ Complete (6/6 tests passing)

**Implementation:**
- [X] Created `inference.py` module with `SocketInferenceEngine` class
- [X] Implemented `infer_sockets_for_node()` with introspection integration
- [X] Added topic name normalization
- [X] Generate TODO markers when introspection fails
- [X] Add helpful comments to TODO markers
- [X] Support batch inference

**Test Cases:** 6/6 passing
- [X] Successful resolution
- [X] Generate TODO when introspection fails
- [X] Generate TODO when socket not found
- [X] Helpful TODO comments
- [X] Topic name normalization
- [X] Batch inference

**Deliverable:** ✅ Node conversion with introspection-based inference or explicit TODOs

---

### Phase 4: Plan Builder & Link Generation ✅

**Goal:** Generate complete ROS-Plan YAML with sockets and links

**Status:** ✅ Complete (10/10 tests passing)

**Implementation:**
- [X] Created `builder.py` module with `PlanBuilder` class
- [X] Implemented `build_plan()` to generate complete plan
- [X] Generate node sections with pkg, exec, namespace, parameters, sockets
- [X] Used ruamel.yaml for YAML formatting
- [X] Infer links by grouping remappings by resolved topic name
- [X] Generate link sections with message types from introspection
- [X] Support TODO markers in links
- [X] Handle multiple publishers and subscribers per link
- [X] Support conditional nodes with `when` clauses

**Test Cases:** 10/10 passing
- [X] Node YAML generation
- [X] Socket with directions and TODO markers
- [X] Link discovery from remappings
- [X] Link YAML with types
- [X] Complete plan output
- [X] TODO socket handling
- [X] TODO message type in links
- [X] Multi-endpoint links
- [X] Conditional node
- [X] YAML string conversion

**Deliverable:** ✅ Generate valid, compilable plan YAML files with sockets and links

---

### Phase 5: Argument & Parameter Conversion ✅

**Goal:** Convert launch arguments and parameters to plan format

**Status:** ✅ Complete (26/26 tests passing)

**Implementation:**
- [X] Extended `visitor.py` to capture `DeclareLaunchArgument` actions
- [X] Added `LaunchArgumentMetadata` dataclass
- [X] Created `arg_inference.py` module for type inference
- [X] Infer types: bool, i64, f64, str, todo
- [X] Extended `builder.py` with `_build_arg_section()`
- [X] Generate arg section with proper type tags
- [X] Implemented `_convert_launch_configurations()`
- [X] Recursive conversion for nested parameter dictionaries
- [X] Support LaunchConfiguration in lists and nested structures

**Test Cases:** 26/26 passing
- [X] 17 arg_inference tests (bool, int, float, string, edge cases)
- [X] 9 builder tests (arg section generation, LaunchConfiguration substitution)

**Deliverable:** ✅ Complete argument and parameter handling with type inference and substitution

---

### Phase 6: Conditional Branch Exploration ✅

**Goal:** Handle conditional nodes and generate `when` clauses

**Status:** ✅ Complete (18/18 tests passing)

**Implementation:**
- [X] Enhanced `extract_condition_expression()` with robust condition handling
- [X] Check UnlessCondition before IfCondition (subclass relationship)
- [X] Extract predicate from name-mangled attribute
- [X] Handle LaunchConfiguration substitutions
- [X] Handle UnlessCondition with negation
- [X] Support TextSubstitution for literal values
- [X] Case-insensitive boolean text handling
- [X] Multiple substitutions with Lua concatenation
- [X] Nested condition tracking with condition_stack
- [X] Compound expressions with "and" operator

**Test Cases:** 18/18 passing
- [X] IfCondition and UnlessCondition with LaunchConfiguration
- [X] Literal text values (true/false, 1/0, custom)
- [X] None condition handling
- [X] Single and nested condition stacks
- [X] Mixed If/Unless conditions
- [X] Case-insensitive boolean text

**Deliverable:** ✅ Complete support for conditional nodes with `when` clauses

---

### Phase 7: Include Handling & Plan Hierarchy ✅

**Goal:** Preserve launch file structure with plan includes

**Status:** ✅ Complete (8/8 tests passing)

**Implementation:**
- [X] Enhanced `visit_include_launch_description()` to recursively process includes
- [X] Implemented proper include path resolution
- [X] Added cycle detection using include_stack
- [X] Extended `builder.py` with `_build_include_section()`
- [X] Generate include sections with file reference and argument forwarding
- [X] Infer argument types for included files
- [X] Support conditional includes with `when` clauses
- [X] Handle duplicate include names with numeric suffixes

**Test Cases:** 8/8 passing
- [X] Include detection with path resolution
- [X] Argument capture and type inference
- [X] LaunchConfiguration substitution in arguments
- [X] Simple and nested cycle prevention
- [X] Include section YAML generation
- [X] Duplicate name handling

**Deliverable:** ✅ Full support for launch file includes with plan includes, argument forwarding, and cycle detection

**Limitation:** Current implementation inlines included content into single plan file. Phase 13 will generate separate plan files for each launch file.

---

### Phase 8: Metadata Tracking ✅

**Goal:** Track conversion state for transparency and debugging

**Status:** ✅ Complete (13/13 tests passing)

**Implementation:**
- [X] Created `metadata.py` with data structures
- [X] Modified `builder.py` to collect TODOs during plan generation
- [X] Implemented `MetadataManager` for saving/loading metadata to JSON
- [X] Created `PlanParser` for plan YAML parsing and TODO discovery
- [X] Implemented `TodoStatusUpdater` for detecting user-completed TODOs
- [X] Added `statistics.py` for conversion statistics calculation
- [X] Integrated metadata generation into CLI with SHA256 staleness detection
- [X] Added `status` subcommand to display TODO completion progress

**Test Cases:** 13/13 passing
- [X] Dataclass serialization and JSON round-trip
- [X] Metadata persistence (save/load)
- [X] Plan YAML parsing and TODO discovery
- [X] JSONPath navigation
- [X] User edit detection (completed TODOs)
- [X] Source hash checking for staleness
- [X] Statistics computation
- [X] Builder TODO collection

**Deliverable:** ✅ Transparent conversion tracking with explicit TODO markers and user edit detection

---

### Phase 9: Validation & Compilation (Future Work)

**Goal:** Validate generated plans and ensure they compile

**Status:** 🔴 Not Started

**Planned Features:**
1. Create `validator.py` module
2. Implement plan compilation check (call `ros2plan compile`)
3. Parse and report compilation errors
4. Add `validate` CLI command
5. Enhance `status` CLI command

---

### Phase 10: End-to-End Testing & Examples (Future Work)

**Goal:** Comprehensive testing with real-world launch files

**Status:** 🔴 Not Started

**Planned Features:**
1. Create test fixtures (simple, complex, with includes)
2. Test complete conversion workflow
3. Validate all generated plans compile successfully
4. Create example conversions for documentation
5. Test edge cases (missing packages, invalid syntax)

---

### Phase 11: QoS Profile Handling (Future Work)

**Goal:** Preserve QoS settings from introspection

**Status:** 🔴 Not Started

**Planned Features:**
1. Extract QoS profiles from introspection results
2. Generate QoS sections in plan YAML
3. Support common QoS presets
4. Handle custom QoS settings

---

### Phase 12: Pattern Learning (Future Work)

**Goal:** Learn from user corrections to auto-fill similar TODOs

**Status:** 🔴 Not Started

**Planned Features:**
1. Detect user completions of TODO markers
2. Build pattern database
3. Apply patterns to similar cases
4. `refine` CLI command for pattern application

---

### Phase 13: Modular Plan Generation (Future Work)

**Goal:** Achieve strict 1-to-1 graph equivalence between launch file tree and plan file tree

**Status:** 🔴 Not Started (Design complete - see launch2plan.md)

**Problem Statement:** Current implementation (Phases 1-8) generates a single monolithic plan file with all included content inlined. This does NOT achieve graph equivalence with the launch file structure.

**Core Principle:** **One launch file → one plan file** (regardless of how many times included or with what arguments)

**Architecture Changes:**

Transform from:
```
launch2plan convert robot.launch.py
→ robot.plan.yaml (single file, all content inlined)
```

To:
```
launch2plan convert robot.launch.py --output-dir output/
→ output/robot.plan.yaml           (parent with include references)
→ output/sensors/camera.plan.yaml  (separate file, accepts arguments)
→ output/nav2_bringup/launch/navigation.plan.yaml  (package-based)
→ output/_path/home/user/custom/special.plan.yaml  (path-based)
```

**Work Items:**

1. **F73: Multi-File Output Infrastructure** (6 hours)
2. **F74: Package Detection** (4 hours)
3. **F75: Include References (Not Inlining)** (5 hours)
4. **F76: Argument Substitution Generation** (4 hours)
5. **F77: Deduplication by File Path** (3 hours)
6. **F78: Output Directory Management** (3 hours)

**Test Count:** ~30 new tests

**Deliverable:** Modular plan generation with strict 1-to-1 graph equivalence to launch file structure
