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
- ✅ Created `launch2plan` package structure
- ✅ Set up pyproject.toml with dependencies
- ✅ Created CLI with `convert` command
- ✅ Implemented branch-exploring visitor (explores ALL branches)
- ✅ Extract node metadata (package, executable, name, namespace, remappings)
- ✅ Track condition expressions for `when` clause generation
- ✅ Detect includes (basic - no recursion yet)

**Test Cases:** 3/3 passing
- ✅ Single node discovery
- ✅ Multiple nodes with remappings
- ✅ Basic CLI invocation

**Deliverable:** ✅ Can visit a simple launch file and list discovered nodes

---

### Phase 2: RMW Introspection Integration ✅

**Goal:** Integrate ros2-introspect for accurate socket inference

**Status:** ✅ Complete (5/5 tests passing)

**Implementation:**
- ✅ Created `introspection.py` module
- ✅ Implemented `IntrospectionService` class with caching
- ✅ Added `get_socket_info()` and `get_all_topics()` methods
- ✅ Graceful handling of introspection failures
- ✅ Cache introspection results per package::executable

**Test Cases:** 5/5 passing
- ✅ Demo nodes (talker/listener)
- ✅ Caching behavior
- ✅ Introspection fallback
- ✅ Socket direction resolution
- ✅ Message type resolution

**Deliverable:** ✅ Can introspect nodes and determine socket directions + message types

---

### Phase 3: Socket Inference & TODO Generation ✅

**Goal:** Integrate introspection with node conversion, generate TODO markers for unknowns

**Status:** ✅ Complete (6/6 tests passing)

**Implementation:**
- ✅ Created `inference.py` module with `SocketInferenceEngine` class
- ✅ Implemented `infer_sockets_for_node()` with introspection integration
- ✅ Added topic name normalization
- ✅ Generate TODO markers when introspection fails
- ✅ Add helpful comments to TODO markers
- ✅ Support batch inference

**Test Cases:** 6/6 passing
- ✅ Successful resolution
- ✅ Generate TODO when introspection fails
- ✅ Generate TODO when socket not found
- ✅ Helpful TODO comments
- ✅ Topic name normalization
- ✅ Batch inference

**Deliverable:** ✅ Node conversion with introspection-based inference or explicit TODOs

---

### Phase 4: Plan Builder & Link Generation ✅

**Goal:** Generate complete ROS-Plan YAML with sockets and links

**Status:** ✅ Complete (10/10 tests passing)

**Implementation:**
- ✅ Created `builder.py` module with `PlanBuilder` class
- ✅ Implemented `build_plan()` to generate complete plan
- ✅ Generate node sections with pkg, exec, namespace, parameters, sockets
- ✅ Used ruamel.yaml for YAML formatting
- ✅ Infer links by grouping remappings by resolved topic name
- ✅ Generate link sections with message types from introspection
- ✅ Support TODO markers in links
- ✅ Handle multiple publishers and subscribers per link
- ✅ Support conditional nodes with `when` clauses

**Test Cases:** 10/10 passing
- ✅ Node YAML generation
- ✅ Socket with directions and TODO markers
- ✅ Link discovery from remappings
- ✅ Link YAML with types
- ✅ Complete plan output
- ✅ TODO socket handling
- ✅ TODO message type in links
- ✅ Multi-endpoint links
- ✅ Conditional node
- ✅ YAML string conversion

**Deliverable:** ✅ Generate valid, compilable plan YAML files with sockets and links

---

### Phase 5: Argument & Parameter Conversion ✅

**Goal:** Convert launch arguments and parameters to plan format

**Status:** ✅ Complete (26/26 tests passing)

**Implementation:**
- ✅ Extended `visitor.py` to capture `DeclareLaunchArgument` actions
- ✅ Added `LaunchArgumentMetadata` dataclass
- ✅ Created `arg_inference.py` module for type inference
- ✅ Infer types: bool, i64, f64, str, todo
- ✅ Extended `builder.py` with `_build_arg_section()`
- ✅ Generate arg section with proper type tags
- ✅ Implemented `_convert_launch_configurations()`
- ✅ Recursive conversion for nested parameter dictionaries
- ✅ Support LaunchConfiguration in lists and nested structures

**Test Cases:** 26/26 passing
- ✅ 17 arg_inference tests (bool, int, float, string, edge cases)
- ✅ 9 builder tests (arg section generation, LaunchConfiguration substitution)

**Deliverable:** ✅ Complete argument and parameter handling with type inference and substitution

---

### Phase 6: Conditional Branch Exploration ✅

**Goal:** Handle conditional nodes and generate `when` clauses

**Status:** ✅ Complete (18/18 tests passing)

**Implementation:**
- ✅ Enhanced `extract_condition_expression()` with robust condition handling
- ✅ Check UnlessCondition before IfCondition (subclass relationship)
- ✅ Extract predicate from name-mangled attribute
- ✅ Handle LaunchConfiguration substitutions
- ✅ Handle UnlessCondition with negation
- ✅ Support TextSubstitution for literal values
- ✅ Case-insensitive boolean text handling
- ✅ Multiple substitutions with Lua concatenation
- ✅ Nested condition tracking with condition_stack
- ✅ Compound expressions with "and" operator

**Test Cases:** 18/18 passing
- ✅ IfCondition and UnlessCondition with LaunchConfiguration
- ✅ Literal text values (true/false, 1/0, custom)
- ✅ None condition handling
- ✅ Single and nested condition stacks
- ✅ Mixed If/Unless conditions
- ✅ Case-insensitive boolean text

**Deliverable:** ✅ Complete support for conditional nodes with `when` clauses

---

### Phase 7: Include Handling & Plan Hierarchy ✅

**Goal:** Preserve launch file structure with plan includes

**Status:** ✅ Complete (8/8 tests passing)

**Implementation:**
- ✅ Enhanced `visit_include_launch_description()` to recursively process includes
- ✅ Implemented proper include path resolution
- ✅ Added cycle detection using include_stack
- ✅ Extended `builder.py` with `_build_include_section()`
- ✅ Generate include sections with file reference and argument forwarding
- ✅ Infer argument types for included files
- ✅ Support conditional includes with `when` clauses
- ✅ Handle duplicate include names with numeric suffixes

**Test Cases:** 8/8 passing
- ✅ Include detection with path resolution
- ✅ Argument capture and type inference
- ✅ LaunchConfiguration substitution in arguments
- ✅ Simple and nested cycle prevention
- ✅ Include section YAML generation
- ✅ Duplicate name handling

**Deliverable:** ✅ Full support for launch file includes with plan includes, argument forwarding, and cycle detection

**Limitation:** Current implementation inlines included content into single plan file. Phase 13 will generate separate plan files for each launch file.

---

### Phase 8: Metadata Tracking ✅

**Goal:** Track conversion state for transparency and debugging

**Status:** ✅ Complete (13/13 tests passing)

**Implementation:**
- ✅ Created `metadata.py` with data structures
- ✅ Modified `builder.py` to collect TODOs during plan generation
- ✅ Implemented `MetadataManager` for saving/loading metadata to JSON
- ✅ Created `PlanParser` for plan YAML parsing and TODO discovery
- ✅ Implemented `TodoStatusUpdater` for detecting user-completed TODOs
- ✅ Added `statistics.py` for conversion statistics calculation
- ✅ Integrated metadata generation into CLI with SHA256 staleness detection
- ✅ Added `status` subcommand to display TODO completion progress

**Test Cases:** 13/13 passing
- ✅ Dataclass serialization and JSON round-trip
- ✅ Metadata persistence (save/load)
- ✅ Plan YAML parsing and TODO discovery
- ✅ JSONPath navigation
- ✅ User edit detection (completed TODOs)
- ✅ Source hash checking for staleness
- ✅ Statistics computation
- ✅ Builder TODO collection

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
