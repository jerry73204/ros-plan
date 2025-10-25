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

**Limitation:** Current implementation inlines included content into single plan file. Phase 9 will generate separate plan files for each launch file.

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

### Phase 9: Modular Plan Generation (Refactor)

**Goal:** Achieve strict 1-to-1 graph equivalence between launch file tree and plan file tree

**Status:** 🔴 Not Started (Design complete)

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

---

#### Subphase 9.1: Multi-File Output Infrastructure

**Goal:** Refactor builder to support multiple output files with file registry

**Status:** 🔴 Not Started

**Estimated Time:** 6 hours

**Refactoring Steps:**

1. **Create `file_registry.py` module**
   - [ ] Implement `FileRegistry` class to track all generated plan files
   - [ ] Add `register_file()` method with path deduplication
   - [ ] Add `get_output_path()` for consistent path resolution
   - [ ] Support package-based and path-based naming strategies

2. **Refactor `builder.py`**
   - [ ] Change `PlanBuilder.build_plan()` to return dict of `{path: yaml_content}`
   - [ ] Add `output_dir` parameter to builder constructor
   - [ ] Remove inlining logic from `_build_include_section()`
   - [ ] Generate include references instead of inlining content

3. **Update CLI**
   - [ ] Add `--output-dir` argument to `convert` command
   - [ ] Create output directory if it doesn't exist
   - [ ] Write all files from registry to disk
   - [ ] Report summary of generated files

**Test Cases:**
- [ ] FileRegistry tracks multiple files without duplicates
- [ ] Builder generates multiple plan files
- [ ] CLI creates output directory structure
- [ ] Path collision detection works correctly

**Deliverable:** Infrastructure for generating multiple plan files with proper path management

---

#### Subphase 9.2: Package Detection & Path Resolution

**Goal:** Detect if launch file belongs to a ROS package and determine output paths

**Status:** 🔴 Not Started

**Estimated Time:** 4 hours

**Refactoring Steps:**

1. **Create `package_detector.py` module**
   - [ ] Implement `detect_package()` function
   - [ ] Search for `package.xml` in parent directories
   - [ ] Extract package name from XML
   - [ ] Return `PackageInfo(name, root_path, relative_path)`

2. **Add path resolution strategies**
   - [ ] Package-based: `output_dir/package_name/relative/path.plan.yaml`
   - [ ] Path-based: `output_dir/_path/absolute/path.plan.yaml` (for non-package files)
   - [ ] Implement `resolve_output_path()` in `file_registry.py`

3. **Update visitor**
   - [ ] Detect package for each included launch file
   - [ ] Store package info in metadata
   - [ ] Pass package info to builder

**Test Cases:**
- [ ] Detect package from launch file inside package
- [ ] Handle launch file outside any package
- [ ] Resolve package-based output paths correctly
- [ ] Resolve path-based output paths correctly
- [ ] Handle nested package structures

**Deliverable:** Automatic package detection with proper output path resolution

---

#### Subphase 9.3: Include References (Stop Inlining)

**Goal:** Generate plan includes that reference other plan files instead of inlining content

**Status:** 🔴 Not Started

**Estimated Time:** 5 hours

**Refactoring Steps:**

1. **Refactor include handling in `visitor.py`**
   - [ ] Track include metadata without recursing into content
   - [ ] Store include source path and arguments
   - [ ] Create placeholder for deferred processing

2. **Refactor `builder.py` include section**
   - [ ] Change `_build_include_section()` to generate file references
   - [ ] Use relative paths from parent plan to included plan
   - [ ] Generate `file: package://pkg/path.plan.yaml` for package-based
   - [ ] Generate `file: ../../other.plan.yaml` for relative paths
   - [ ] Keep argument forwarding logic

3. **Implement two-pass processing**
   - [ ] Pass 1: Visit launch file, collect nodes and include references
   - [ ] Pass 2: Process each unique include separately
   - [ ] Recursively generate plan files for all includes
   - [ ] Track visited files to prevent infinite loops

**Test Cases:**
- [ ] Simple include generates separate plan file
- [ ] Include reference uses correct relative path
- [ ] Package-based includes use `package://` syntax
- [ ] Arguments are forwarded correctly
- [ ] Nested includes work (3+ levels deep)
- [ ] Cycle detection prevents infinite recursion

**Deliverable:** Plan includes reference separate plan files instead of inlining content

---

#### Subphase 9.4: Argument Substitution in Include References

**Goal:** Properly generate argument passing in include references

**Status:** 🔴 Not Started

**Estimated Time:** 4 hours

**Refactoring Steps:**

1. **Analyze argument flow**
   - [ ] Identify which parent arguments are passed to includes
   - [ ] Track argument transformations (renames, defaults)
   - [ ] Detect which arguments need type conversion

2. **Enhance `_build_include_section()`**
   - [ ] Generate `arg:` subsection for each include
   - [ ] Use `$(arg_name)` syntax for argument forwarding
   - [ ] Handle renamed arguments: `child_arg: $(parent_arg)`
   - [ ] Handle literal values: `child_arg: literal_value`
   - [ ] Preserve type annotations

3. **Update argument inference**
   - [ ] Infer types for include arguments from defaults
   - [ ] Propagate type information across include boundaries
   - [ ] Handle missing type information with TODO markers

**Test Cases:**
- [ ] Simple argument forwarding
- [ ] Renamed argument passing
- [ ] Mixed forwarded and literal arguments
- [ ] Type preservation across includes
- [ ] Default value handling
- [ ] Missing argument detection

**Deliverable:** Correct argument substitution in include references with type preservation

---

#### Subphase 9.5: File Deduplication by Path

**Goal:** Ensure each unique launch file generates exactly one plan file

**Status:** 🔴 Not Started

**Estimated Time:** 3 hours

**Refactoring Steps:**

1. **Implement canonical path tracking**
   - [ ] Resolve all launch file paths to canonical absolute paths
   - [ ] Use canonical path as deduplication key
   - [ ] Store first-seen mapping in `FileRegistry`

2. **Handle multiple inclusions**
   - [ ] Detect when same file is included multiple times
   - [ ] Skip regeneration if already processed
   - [ ] Verify argument compatibility across inclusions
   - [ ] Warn if same file included with different arguments

3. **Update processing logic**
   - [ ] Check registry before processing each include
   - [ ] Reuse existing output path for duplicate includes
   - [ ] Update include references to point to single generated file

**Test Cases:**
- [ ] Same file included twice generates only one plan
- [ ] Include references point to same plan file
- [ ] Different paths to same file (symlinks) are deduplicated
- [ ] Warning issued for incompatible argument sets
- [ ] Deduplication works across nested includes

**Deliverable:** Each unique launch file generates exactly one plan file regardless of inclusion count

---

#### Subphase 9.6: Output Directory Management & Reporting

**Goal:** Organize output files and provide clear conversion summary

**Status:** 🔴 Not Started

**Estimated Time:** 3 hours

**Refactoring Steps:**

1. **Implement directory creation**
   - [ ] Create output directory structure recursively
   - [ ] Handle package subdirectories
   - [ ] Handle `_path/` prefix for absolute paths
   - [ ] Preserve relative structure from packages

2. **Add conversion summary**
   - [ ] Count total files generated
   - [ ] List all generated plan files with source mapping
   - [ ] Report statistics (nodes, links, includes, TODOs)
   - [ ] Show file tree visualization

3. **Update metadata**
   - [ ] Store multi-file conversion metadata
   - [ ] Track source→output file mapping
   - [ ] Enable metadata-based regeneration
   - [ ] Support `status` command for multi-file plans

**Test Cases:**
- [ ] Directory structure created correctly
- [ ] Summary reports all files
- [ ] Metadata tracks multi-file conversion
- [ ] File tree visualization is accurate
- [ ] Status command works with multi-file plans

**Deliverable:** Clean output directory organization with comprehensive conversion reporting

---

#### Subphase 9.7: Integration Testing & Examples

**Goal:** Validate multi-file generation with real-world launch file hierarchies

**Status:** 🔴 Not Started

**Estimated Time:** 4 hours

**Testing Steps:**

1. **Create test fixtures**
   - [ ] Simple hierarchy: parent + 1 include
   - [ ] Medium hierarchy: parent + 3 includes (1 nested)
   - [ ] Complex hierarchy: 10+ files with multiple nesting levels
   - [ ] Mixed packages: includes from different packages
   - [ ] Absolute paths: includes from outside any package

2. **End-to-end tests**
   - [ ] Convert each fixture
   - [ ] Verify file count matches expected
   - [ ] Verify include references are correct
   - [ ] Compile all generated plans successfully
   - [ ] Check argument flow through hierarchy

3. **Documentation examples**
   - [ ] Create example conversion in documentation
   - [ ] Show before/after file structure
   - [ ] Document command usage
   - [ ] Add troubleshooting guide

**Test Cases:**
- [ ] 5+ integration tests covering various hierarchies
- [ ] All generated plans compile without errors
- [ ] Include paths resolve correctly
- [ ] Arguments propagate through hierarchy

**Deliverable:** Validated multi-file generation with comprehensive test coverage and documentation

---

**Phase 9 Summary:**

- **Total Subphases:** 7
- **Estimated Time:** 29 hours
- **Test Count:** ~35 new tests
- **Key Deliverable:** Modular plan generation with strict 1-to-1 graph equivalence to launch file structure

---

### Phase 10: Validation & Compilation (Future Work)

**Goal:** Validate generated plans and ensure they compile

**Status:** 🔴 Not Started

**Planned Features:**
1. Create `validator.py` module
2. Implement plan compilation check (call `ros2plan compile`)
3. Parse and report compilation errors
4. Add `validate` CLI command
5. Enhance `status` CLI command

---

### Phase 11: End-to-End Testing & Examples (Future Work)

**Goal:** Comprehensive testing with real-world launch files

**Status:** 🔴 Not Started

**Planned Features:**
1. Create test fixtures (simple, complex, with includes)
2. Test complete conversion workflow
3. Validate all generated plans compile successfully
4. Create example conversions for documentation
5. Test edge cases (missing packages, invalid syntax)

---

### Phase 12: QoS Profile Handling (Future Work)

**Goal:** Preserve QoS settings from introspection

**Status:** 🔴 Not Started

**Planned Features:**
1. Extract QoS profiles from introspection results
2. Generate QoS sections in plan YAML
3. Support common QoS presets
4. Handle custom QoS settings

---

### Phase 13: Pattern Learning (Future Work)

**Goal:** Learn from user corrections to auto-fill similar TODOs

**Status:** 🔴 Not Started

**Planned Features:**
1. Detect user completions of TODO markers
2. Build pattern database
3. Apply patterns to similar cases
4. `refine` CLI command for pattern application
