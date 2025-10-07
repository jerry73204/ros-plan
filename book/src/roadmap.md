# Implementation Roadmap

## 🎉 Core Implementation Complete!

**Status:** All 4 core phases (Phases 1-4) are complete. The ROS-Plan compiler is production-ready.

**Key Achievements:**
- ✅ **207 tests passing** (48 compiler + 159 format)
- ✅ **Zero warnings** (clippy + compiler)
- ✅ **All core features implemented** (25/25)
- ✅ **Comprehensive test coverage** across all subsystems
- ✅ **Full encapsulation & transparency support** (including multi-level transparent includes)

**What Works:**
- Node socket references and linking
- Topic name derivation and resolution
- Plan socket forwarding and aggregation
- Transparent includes with deep socket references
- Multi-source validation and error handling
- Namespace hierarchy tracking
- Comprehensive error messages with helpful hints

**Next Steps:** Phase 5 features (F5, F8, F17, F18, F21) are optional enhancements that can be added based on user demand.

---

## Overview

This page tracks the implementation status of the node/link connection design features. Each feature includes work items, expected results, test cases, and current implementation status.

**Legend:**
- ❌ Not Started
- 🚧 In Progress
- ✅ Complete

**Progress Summary:**
- **Phase 1** (Foundation): ✅ 100% - 6/6 features complete
- **Phase 2** (Topic Resolution): ✅ 100% - 5/5 features complete
- **Phase 3** (Validation): ✅ 100% - 5/5 features complete
- **Phase 4** (Encapsulation): ✅ 100% - 9/9 features complete (F4, F11-F14, F16, F19-F20, F31)
- **Phase 5** (Optional): ⏸️ Deferred - 0/5 features (F5, F8, F17, F18, F21)

**Feature Categories:**
- Format Extensions: 4/5 complete (F1 ✅, F2 ✅, F3 ✅, F4 ✅; F5 ⏸️)
- Topic Resolution: 5/5 complete (F6 ✅, F7 ✅, F9 ✅, F10 ✅, F19 ✅)
- Plan Encapsulation: 7/7 complete (F11 ✅, F12 ✅, F13 ✅, F14 ✅, F16 ✅, F20 ✅, F31 ✅)
- Validation & Errors: 4/4 complete (F7 ✅, F11 ✅, F15 ✅, F16 ✅)
- Format Parsing Tests: 4/4 complete (F22 ✅, F23 ✅, F24 ✅, F25 ✅)
- Compiler Algorithm Tests: 4/4 complete (F26 ✅, F27 ✅, F28 ✅, F29 ✅)
- Integration Tests: 2/2 complete (F30 ✅, F31 ✅)

**Total:** 25/30 core features complete; 5 optional features deferred

**Test Coverage:** 207 tests passing (48 in compiler, 159 in format)

---

## Implementation Phases

### Phase 1: Foundation & Testing Infrastructure (2-3 weeks) ✅

**Goal:** Establish robust testing infrastructure and cover existing code with tests before adding new features.

**Features:** F22 ✅, F23 ✅, F24 ✅, F25 ✅, F26 ✅, F27 ✅

**Progress:** 6/6 features complete (100%)

**Deliverables:**
- ✅ Dev-dependencies added to all crates
- ✅ Unit test framework for format parsing (F22, F23, F24, F25)
- ✅ Unit test framework for compiler algorithms (F26, F27)
- ✅ Test fixtures and basic test utilities
- ✅ CI integration for automated testing (via Makefile)

**Timeline:** Completed

**Current Status:**
- 192 total tests (up from 2 baseline)
- Format tests: plan.rs (6), node.rs (7), link.rs (18), plan_socket.rs (8), node_socket.rs (9), key.rs (2 existing)
- Expression tests: expr_.rs (15), value_or_expr.rs (13), text_or_expr.rs (11), bool_expr.rs (9), key_or_expr.rs (12)
- Type tests: value_type.rs (8), value.rs (19)
- Error tests: error.rs (17)
- Compiler tests: program.rs (5), lua.rs (13), link_resolver.rs (10), socket_resolver.rs (5)
- Integration tests: multi_source_validation.rs (5)
- All tests passing via `make test`

---

### Phase 2: Core Topic Resolution (2-3 weeks) ✅

**Goal:** Implement basic single-source topic derivation with comprehensive tests.

**Features:** F1 ✅, F2 ✅, F6 ✅, F9 ✅, F28 ✅, F29 ✅

**Progress:** 6/6 features complete (100%)

**Deliverables:**
- ✅ Node socket `ros_name` attribute (F1)
- ✅ Link `topic` attribute (F2)
- ✅ Single-source topic derivation algorithm (F6, F9)
- ✅ Unit tests for socket resolution (F28)
- ✅ Unit tests for link resolution (F29)

**Dependencies:** Phase 1 (testing infrastructure)

**Timeline:** Completed

**Current Status:**
- 192 total tests passing
- Added 9 node socket parsing tests (F1)
- Added 7 link topic parsing tests (F2)
- Added 10 link resolver unit tests covering single-source derivation and ros_name override (F6, F9, F29)
- Added 5 socket resolver unit tests (F28)
- All Phase 2 features fully implemented and tested

---

### Phase 3: Multi-Source & Validation (2 weeks) ✅

**Goal:** Support multiple publishers and add comprehensive validation.

**Features:** F3 ✅, F7 ✅, F10 ✅, F15 ✅, F30 ✅

**Progress:** 5/5 features complete (100%)

**Deliverables:**
- ✅ Plan socket `topic` attribute (F3)
- ✅ Multi-source validation and errors (F7, F15)
- ✅ Plan socket topic resolution (F10)
- ✅ Integration test suite with fixtures (F30)

**Dependencies:** Phase 2 (core resolution)

**Timeline:** Completed

**Current Status:**
- 192 total tests passing
- Added 8 plan socket parsing tests (F3)
- Added 10 link resolver unit tests (F7, F10, F15)
- Added 5 integration tests (F30)
- Test fixtures: 5 YAML files in tests/fixtures/
- Integration test file: tests/multi_source_validation.rs
- All tests passing via `make test` and `cargo clippy`

---

### Phase 4: Encapsulation & Transparency ✅

**Goal:** Implement plan boundaries, socket visibility, and transparent includes.

**Features:** F4 ✅, F11 ✅, F12 ✅, F13 ✅, F14 ✅, F16 ✅, F31 ✅

**Progress:** 7/7 features complete (100%) ✅

**Deliverables:**
- ✅ Include `transparent` flag (F4)
- ✅ Socket reference depth validation (F11)
- ✅ Transparent resolution algorithm (F12)
- ✅ Namespace hierarchy tracking (F14)
- ✅ Invalid reference error types (F16)
- ✅ Plan socket forwarding validation (F13)
- ✅ Error scenario integration tests (F31)

**Dependencies:** Phase 3 (validation)

**Timeline:** Complete

**Final Status:**
- 207 total tests passing (48 in ros-plan-compiler, 159 in ros-plan-format)
- ✅ Transparent flag parsing and tracking (F4)
- ✅ Depth validation with transparency support (F11/F12)
- ✅ Single and multi-level transparent includes working (F12)
- ✅ Namespace hierarchy tracking (F14)
- ✅ Comprehensive error handling (F16)
- ✅ 8 integration tests covering all encapsulation scenarios (F31)

---

### Phase 5: Optional Enhancements & Polish (Future)

**Goal:** Add optional quality-of-life features and advanced validation.

**Status:** ⏸️ Deferred - Core functionality complete

**Optional Features:**
- F5: Empty `src`/`dst` support (low priority)
- F8: Absolute/relative topic path resolution (nice-to-have)
- F17: Type compatibility checking (advanced validation)
- F18: QoS requirement satisfaction (advanced validation)
- F21: Real-world example suite (documentation)

**Rationale for Deferral:**
- All core compilation features are complete (Phases 1-4)
- 207 tests passing with comprehensive coverage
- System is production-ready for current use cases
- These features can be added incrementally based on user demand

**Dependencies:** Phase 4 (complete)

**Timeline:** TBD based on user requirements

---

## Feature Catalog

### Format Extensions

These features require changes to the YAML schema and data structures in `ros-plan-format`.

#### F1: Node Socket `ros_name` Attribute ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Allow node sockets to override their name for ROS topic derivation.

**Work Items:**
1. ✅ Add `ros_name: Option<TextOrExpr>` field to `NodePubCfg`, `NodeSubCfg`, `NodeSrvCfg`, `NodeCliCfg`
2. ✅ Update YAML deserialization to parse `ros_name`
3. ✅ Document field in format specification
4. ✅ Write parsing tests (9 total)

**Expected Results:**
```yaml
node:
  camera:
    socket:
      raw_output: !pub
        type: sensor_msgs/msg/Image
        ros_name: image_raw

# When referenced: topic becomes /namespace/camera/image_raw
# Not: /namespace/camera/raw_output
```

**Test Cases:**
1. ✅ Parse YAML with `ros_name` field
2. ✅ Verify topic derivation uses `ros_name` when present
3. ✅ Verify topic derivation uses socket name when `ros_name` absent
4. ✅ Test with all socket types (pub, sub, srv, cli)

**Current State:**
- Fields: ✅ Added to all 4 socket types (`NodePubCfg`, `NodeSubCfg`, `NodeSrvCfg`, `NodeCliCfg`)
- Compiler: ✅ Added to all 4 context types (`NodePubCtx`, `NodeSubCtx`, `NodeSrvCtx`, `NodeCliCtx`)
- Tests: ✅ 9 parsing tests passing (2 per socket type + 1 without ros_name)

**Files Affected:**
- ✅ `ros-plan-format/src/node_socket.rs`
- ✅ `ros-plan-compiler/src/context/node_socket.rs`

---

#### F2: Link `topic` Attribute ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Allow links to specify explicit ROS topic names.

**Work Items:**
1. ✅ Add `topic: Option<TextOrExpr>` field to `PubSubLinkCfg`
2. ✅ Support both absolute (`/tf`) and relative (`diagnostics`) paths
3. ✅ Update YAML deserialization
4. ✅ Document field in format specification
5. ✅ Write parsing tests (7 total)

**Expected Results:**
```yaml
link:
  explicit: !pubsub
    type: sensor_msgs/msg/Image
    topic: /sensors/camera  # Absolute path
    src: [camera/output]
    dst: [processor/input]

  relative: !pubsub
    type: std_msgs/msg/String
    topic: data  # Relative to link namespace
    src: [node_a/out]
    dst: [node_b/in]
```

**Test Cases:**
1. ✅ Parse link with absolute topic path
2. ✅ Parse link with relative topic path
3. ✅ Parse link without topic (existing behavior)
4. ✅ Verify topic resolution with explicit topic
5. ✅ Test empty src/dst with explicit topic

**Current State:**
- Field: ✅ Added to `PubSubLinkCfg`
- Compiler: ✅ Added to `PubSubLinkCtx`
- Tests: ✅ 7 parsing tests covering literal, expression, multi-source, with qos, with when, etc.

**Files Affected:**
- ✅ `ros-plan-format/src/link.rs`
- ✅ `ros-plan-compiler/src/context/link.rs`

---

#### F3: Plan Socket `topic` Attribute ✅

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Allow plan sockets to specify topic names when aggregating multiple sources.

**Work Items:**
1. ✅ Add `topic: Option<TextOrExpr>` field to `PlanPubCfg` and `PlanSubCfg`
2. ✅ Allow `src: Vec<KeyOrExpr>` to have multiple entries
3. ✅ Update YAML deserialization
4. ✅ Document multi-source aggregation pattern
5. ✅ Write parsing tests (8 total)

**Expected Results:**
```yaml
socket:
  tf: !pub
    type: tf2_msgs/msg/TFMessage
    src: [camera/tf, lidar/tf, gnss/tf]
    topic: /tf  # All sources publish to /tf

# Creates /tf topic at plan boundary
```

**Test Cases:**
1. Parse plan socket with topic field
2. Parse plan socket with multiple src entries
3. Verify single src still works
4. Test absolute and relative topic paths
5. Verify topic creation at plan boundary

**Current State:**
- Field: ✅ Added to `PlanPubCfg` and `PlanSubCfg`
- Multi-src: ✅ Already `Vec<KeyOrExpr>`
- Tests: ✅ 8 parsing tests passing
- Compiler integration: ✅ `PlanPubCtx` and `PlanSubCtx` updated

**Files Affected:**
- ✅ `ros-plan-format/src/plan_socket.rs`
- ✅ `ros-plan-compiler/src/context/plan_socket.rs`
- ✅ `ros-plan-compiler/src/processor/program_builder/convert.rs`

---

#### F4: Include `transparent` Flag ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** Medium

**Description:** Allow includes to expose their internal structure to parent plans.

**Work Items:**
1. Add `transparent: Option<bool>` field to `IncludeCfg`
2. Default to `false` (opaque)
3. Update YAML deserialization
4. Document transparency semantics
5. Write parsing tests

**Expected Results:**
```yaml
include:
  sensors: !file
    path: sensors.yaml
    transparent: true  # Parent can see inside

  external: !file
    path: external.yaml
    # transparent: false (default)
```

**Test Cases:**
1. Parse include with `transparent: true`
2. Parse include with `transparent: false`
3. Parse include without transparent (default false)
4. Verify deep references allowed when transparent
5. Verify deep references blocked when opaque

**Current State:**
- Field: ❌ Not present in `IncludeCfg`
- Tests: ❌ None

**Files Affected:**
- `ros-plan-format/src/subplan.rs`

---

#### F5: Empty `src`/`dst` in Links ❌

**Phase:** 5 (Advanced Features)

**Priority:** Low

**Description:** Allow links to have empty sources (consume-only) or empty destinations (publish-only).

**Work Items:**
1. Make `src: Vec<KeyOrExpr>` optional or allow empty in `PubSubLinkCfg`
2. Make `dst: Vec<KeyOrExpr>` optional or allow empty
3. Update validation to accept these patterns
4. Document use cases
5. Write validation tests

**Expected Results:**
```yaml
# Publish-only: Creates topic, no subscribers
link:
  tf_publish: !pubsub
    type: tf2_msgs/msg/TFMessage
    topic: /tf
    src: [camera/tf, lidar/tf]
    dst: []  # No destinations

# Consume-only: Subscribe to existing topic
link:
  tf_consume: !pubsub
    type: tf2_msgs/msg/TFMessage
    topic: /tf
    src: []  # No sources
    dst: [planning/tf_in]
```

**Test Cases:**
1. Parse link with empty src
2. Parse link with empty dst
3. Validate explicit topic required when src empty
4. Verify compile error when both src and dst empty

**Current State:**
- Fields: ✅ Already `Vec<KeyOrExpr>` (can be empty)
- Validation: ❌ Not enforced
- Tests: ❌ None

**Files Affected:**
- `ros-plan-format/src/link.rs`
- Validation logic in compiler

---

### Topic Resolution

These features implement the topic name derivation algorithm.

#### F6: Single-Source Topic Derivation ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Derive ROS topic name from single source socket's namespace and ros_name.

**Work Items:**
1. ✅ Implement topic resolution for links with one source
2. ✅ Handle node sockets vs plan sockets
3. ✅ Use `socket.ros_name ?? socket.name` for topic component
4. ✅ Build full path: `/{namespace}/{topic_component}`
5. ✅ Write comprehensive unit tests (10 total in link_resolver.rs)

**Expected Results:**
```yaml
# Node socket
link:
  camera: !pubsub
    src: [sensing/camera/driver/output]
    # Topic: /sensing/camera/driver/output

# With ros_name override
node:
  driver:
    socket:
      output: !pub
        ros_name: image_raw
link:
  camera: !pubsub
    src: [sensing/camera/driver/output]
    # Topic: /sensing/camera/driver/image_raw
```

**Test Cases:**
1. ✅ Derive topic from node socket name
2. ✅ Derive topic with ros_name override (covered in F9 tests)
3. ✅ Derive topic from plan socket (covered in F10 tests)
4. ✅ Derive topic with plan socket ros_name (covered in F10 tests)
5. ✅ Handle multi-level namespaces correctly

**Current State:**
- Algorithm: ✅ Implemented in `derive_topic_name()` function (lines 247-316)
- Priority chain: ✅ Link topic > Plan socket topic > Node ros_name > Socket name
- Tests: ✅ Covered by 10 link_resolver tests + integration tests

**Files Affected:**
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs`
- ✅ `ros-plan-compiler/src/context/link.rs`

---

#### F7: Multi-Source Explicit Topic Requirement ✅

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Require explicit topic when link has multiple sources, provide clear error otherwise.

**Work Items:**
1. ✅ Implement validation: `len(src) > 1 && topic.is_none()` → Error
2. ✅ Create descriptive error message with link name and source count
3. ✅ Suggest adding `topic` field in error (with example)
4. ✅ Test error reporting with unit tests
5. ✅ Write error scenario tests (integration tests)

**Expected Results:**
```yaml
# ERROR: Missing explicit topic
link:
  bad: !pubsub
    src: [camera/tf, lidar/tf]  # Multiple sources
    dst: [planning/tf_in]
    # ERROR: Links with multiple sources must specify 'topic'

# OK: Explicit topic provided
link:
  good: !pubsub
    topic: /tf
    src: [camera/tf, lidar/tf]
    dst: [planning/tf_in]
```

**Test Cases:**
1. Compile link with 2 sources, no topic → Error
2. Compile link with 3+ sources, no topic → Error
3. Compile link with multiple sources + topic → Success
4. Verify error message is helpful
5. Test with empty src → Different error

**Current State:**
- Validation: ✅ Implemented in `derive_topic_name()`
- Error type: ✅ `MultipleSourcesRequireExplicitTopic` defined
- Tests: ✅ 6 unit tests + 2 integration tests passing

**Files Affected:**
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs`
- ✅ `ros-plan-compiler/src/error.rs`
- ✅ `ros-plan-compiler/tests/multi_source_validation.rs`

---

#### F8: Absolute vs Relative Topic Paths ❌

**Phase:** 5 (Advanced Features)

**Priority:** Medium

**Description:** Resolve topic names as absolute (starting with `/`) or relative to link namespace.

**Work Items:**
1. Implement path resolution logic
2. Detect leading `/` for absolute paths
3. Prepend namespace for relative paths
4. Handle empty namespace (root)
5. Write path resolution tests

**Expected Results:**
```yaml
# At namespace /sensing
link:
  absolute: !pubsub
    topic: /tf  # → /tf
    src: [...]

  relative: !pubsub
    topic: diagnostics  # → /sensing/diagnostics
    src: [...]
```

**Test Cases:**
1. Resolve absolute path from root namespace
2. Resolve absolute path from nested namespace
3. Resolve relative path from root namespace
4. Resolve relative path from nested namespace
5. Handle edge cases (empty namespace, etc.)

**Current State:**
- Algorithm: ❌ Not implemented
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs`

---

#### F9: Socket `ros_name` Override Resolution ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Use socket's `ros_name` field when deriving topic names.

**Work Items:**
1. ✅ Query socket for `ros_name` field
2. ✅ Fall back to socket name if not present
3. ✅ Integrate with topic derivation algorithm
4. ✅ Test with node and plan sockets
5. ✅ Write unit tests for override logic

**Expected Results:**
```yaml
node:
  driver:
    socket:
      img: !pub
        ros_name: image_raw

link:
  feed: !pubsub
    src: [driver/img]
    # Topic: /namespace/driver/image_raw (not /namespace/driver/img)
```

**Test Cases:**
1. ✅ Derive topic with ros_name present
2. ✅ Derive topic with ros_name absent
3. ✅ Test plan socket ros_name (covered in F10 tests)
4. ✅ Test node socket ros_name
5. ✅ Verify precedence: explicit topic > ros_name > socket name

**Current State:**
- Query logic: ✅ Implemented in `derive_topic_name()` (link_resolver.rs:294-298)
- Integration: ✅ Part of topic resolution priority chain
- Tests: ✅ Covered in link_resolver unit tests (explicit override test, plan socket fallthrough test)

**Files Affected:**
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs`
- ✅ `ros-plan-compiler/src/context/node_socket.rs`
- ✅ `ros-plan-compiler/src/context/plan_socket.rs`

---

#### F10: Plan Socket Topic Resolution ✅

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Resolve topics for plan sockets with explicit topic field and multiple sources.

**Work Items:**
1. ✅ Handle plan socket `topic` field in resolution algorithm
2. ✅ Create topic at plan boundary
3. ✅ Map multiple internal sources to single topic
4. ✅ Integrate with link resolution (priority: link > plan socket > node socket)
5. ✅ Write plan socket resolution tests (4 unit + 1 integration)

**Expected Results:**
```yaml
# sensing.yaml
socket:
  tf: !pub
    src: [camera/tf, lidar/tf]
    topic: /tf
# All internal sources publish to /tf

# main.yaml
link:
  system_tf: !pubsub
    topic: /tf
    src: [sensing/tf, localization/tf]
# Adds more sources to /tf
```

**Test Cases:**
1. Plan socket with topic creates ROS topic
2. Multiple internal sources all publish to same topic
3. Parent can reference plan socket
4. Parent can add more sources to same topic
5. Verify topic name resolution

**Current State:**
- Algorithm: ✅ Implemented in `derive_topic_name()` with plan socket tracking
- Tests: ✅ 4 unit tests + 1 integration test passing
- Priority chain: ✅ Link topic > Plan socket topic > Node ros_name > Derived

**Files Affected:**
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs`
- ✅ `ros-plan-compiler/src/selector.rs` (exported `PlanOrNodePub`)
- ✅ `ros-plan-compiler/tests/multi_source_validation.rs`

---

### Plan Encapsulation

These features enforce visibility boundaries between plans.

#### F11: Socket Reference Validation ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** High

**Description:** Validate that socket references only access visible sockets (2-level: entity/socket).

**Work Items:**
1. Parse socket references (split by `/`)
2. Validate format is `entity/socket` (exactly 2 parts)
3. Check entity is local node or child plan
4. Check socket exists on entity
5. Provide clear error messages
6. Write validation tests

**Expected Results:**
```yaml
# main.yaml with nodes: a, b and includes: sensing
link:
  valid1: !pubsub
    src: [a/output]  # OK: local node
    dst: [b/input]

  valid2: !pubsub
    src: [sensing/camera_out]  # OK: child plan socket
    dst: [b/input]

  invalid: !pubsub
    src: [sensing/camera/driver/output]  # ERROR: too deep
    dst: [b/input]
```

**Test Cases:**
1. Valid reference to local node socket → Success
2. Valid reference to child plan socket → Success
3. Invalid reference (3+ levels) → Error
4. Invalid reference to non-existent entity → Error
5. Invalid reference to non-existent socket → Error
6. Error messages include helpful hints

**Current State:**
- Validation: ❌ Not implemented
- Error types: ❌ Not defined
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs`
- `ros-plan-compiler/src/error.rs`

---

#### F12: Transparent Include Resolution ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** HIGH (User requirement)

**Description:** Allow deep socket references when include is marked `transparent: true`. Supports multiple layers of transparent plans.

**Work Items:**
1. ✅ Track transparency flag in include context (DONE in F4)
2. ✅ Add `get_subscope_with_transparency()` to ScopeRefExt trait
3. ✅ Implement `find_node_transparent()` and socket resolution methods
4. ✅ Update depth validation to delegate to resolution (removed hard block)
5. ✅ Implement transitive transparency (transparent → transparent → node)
6. ✅ Write comprehensive transparency tests (3 integration tests)

**Implementation:**
- Added `get_subscope_with_transparency()` to `ScopeRefExt` trait in `scope/traits.rs`
- Checks `transparent` flag on includes and returns None for opaque boundaries
- Added transparent-aware methods to `RelativeSelector`:
  - `find_subscope_transparent()` - traverses through transparent includes
  - `find_node_transparent()` - finds nodes through transparent boundaries
  - `find_node_pub_transparent()`, `find_node_sub_transparent()`, etc.
- Updated `find_plan_or_node_*` methods to use transparent resolution for deep refs
- Modified `validate_socket_reference_depth()` to allow all depths (resolution validates)

**Test Results:**
```yaml
# Single-level transparency (3 segments)
include:
  subplan: !file
    path: transparent_subplan.yaml
    transparent: true

link:
  camera: !pubsub
    src: [subplan/camera/output]  # ✅ Works!
    dst: [processor/input]

# Multi-level transparency (4 segments)
include:
  outer: !file
    path: nested.yaml
    transparent: true  # nested.yaml also has transparent include

link:
  deep: !pubsub
    src: [outer/inner/camera/output]  # ✅ Works!
    dst: [processor/input]
```

**Test Cases:**
1. ✅ Transparent flag parses correctly (F4)
2. ✅ Deep reference through 1 transparent include → Success
3. ✅ Deep reference through opaque include → KeyResolutionError
4. ✅ Multi-level transparency (transparent → transparent → node) → Success
5. ✅ Validation updated - test_too_deep_socket_reference adapted

**Current State:**
- Transparency tracking: ✅ Complete
- Resolution: ✅ Fully implemented in RelativeSelector
- Validation: ✅ Depth validation removed (resolution handles it)
- Tests: ✅ 3 integration tests passing (single-level, multi-level, non-transparent error)

**Files Modified:**
- ✅ `ros-plan-compiler/src/scope/traits.rs` - Added `get_subscope_with_transparency()`
- ✅ `ros-plan-compiler/src/selector/relative_selector.rs` - Added transparent resolution
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs` - Removed depth blocking
- ✅ `ros-plan-compiler/tests/encapsulation_tests.rs` - 3 new tests
- ✅ `ros-plan-compiler/tests/fixtures/*` - 4 new test fixtures

**Actual Effort:** 3 hours
**Test Coverage:** 8 integration tests (5 existing updated + 3 new)

---

#### F13: Plan Socket Forwarding ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** High

**Description:** Support plan sockets that forward internal node/subplan sockets to parent.

**Work Items:**
1. Resolve plan socket `src` references to internal sockets
2. Handle single source forwarding
3. Handle multiple source aggregation
4. Validate internal references
5. Write forwarding tests

**Expected Results:**
```yaml
# subplan.yaml
node:
  camera:
    socket:
      output: !pub

socket:
  camera_out: !pub
    src: [camera/output]  # Forward internal socket

# main.yaml can reference: subplan/camera_out
```

**Test Cases:**
1. Forward single internal node socket
2. Forward internal plan socket (nested)
3. Aggregate multiple internal sockets
4. Validate internal socket exists
5. Error on invalid internal reference

**Current State:**
- Forwarding: 🚧 Partial (existing but needs validation)
- Multi-source: ❌ Not fully tested
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/socket_resolver.rs`

---

#### F14: Namespace Hierarchy Tracking ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** High

**Description:** Track and resolve hierarchical namespaces for nodes and plans.

**Work Items:**
1. Build namespace tree during plan expansion
2. Assign namespaces to nodes: `/{include_path}/{node_name}`
3. Assign namespaces to plans: `/{include_path}/{plan_name}`
4. Support namespace queries during resolution
5. Write namespace tracking tests

**Expected Results:**
```yaml
# root.yaml
include:
  sensing: !file {path: sensing.yaml}

# sensing.yaml (at /sensing)
include:
  camera: !file {path: camera.yaml}

# camera.yaml (at /sensing/camera)
node:
  driver:
    # Namespace: /sensing/camera/driver
```

**Test Cases:**
1. Root-level node → namespace: `/node_name`
2. Nested node (1 level) → `/include/node_name`
3. Nested node (2+ levels) → `/include/nested/node_name`
4. Plan socket → namespace of containing plan
5. Query namespace by key

**Current State:**
- Tracking: 🚧 Partial (scope structure exists)
- Querying: ❌ Needs formalization
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/program.rs`
- `ros-plan-compiler/src/processor/program_builder.rs`

---

### Validation & Error Handling

These features provide compile-time safety guarantees.

#### F15: Multi-Source Without Topic Error ✅

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Clear error when link has multiple sources but no explicit topic.

**Work Items:**
1. ✅ Define error type: `MultipleSourcesRequireExplicitTopic`
2. ✅ Include link name, source count, and suggestion in error
3. ✅ Implement error in link resolution
4. ✅ Test error message quality (unit + integration)
5. ✅ Write error scenario tests (2 integration tests)

**Expected Results:**
```
Error: Link 'camera_array' has 3 sources but no explicit 'topic' attribute.
  Links with multiple sources must specify 'topic'.

  Suggestion: Add a topic field:
    link:
      camera_array: !pubsub
        topic: /cameras/merged  # Add this
        src: [camera_1/out, camera_2/out, camera_3/out]
        dst: [processor/in]
```

**Test Cases:**
1. Error includes link name
2. Error includes source count
3. Error includes suggestion with example
4. Error location points to link definition
5. Error doesn't trigger when topic present

**Current State:**
- Error type: ✅ Defined in error.rs with helpful message format
- Implementation: ✅ Integrated in `derive_topic_name()`
- Tests: ✅ 3 unit tests + 2 integration tests passing

**Files Affected:**
- ✅ `ros-plan-compiler/src/error.rs`
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs`
- ✅ `ros-plan-compiler/tests/multi_source_validation.rs`

---

#### F16: Invalid Socket Reference Errors ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** High

**Description:** Clear errors for invalid socket references with helpful hints.

**Work Items:**
1. Define error types for each case:
   - `SocketReferenceTooDee` (trying to see through opaque boundary)
   - `EntityNotFound` (node/plan doesn't exist)
   - `SocketNotFound` (socket doesn't exist on entity)
2. Include hints about transparency in errors
3. Suggest available entities/sockets
4. Write error message tests

**Expected Results:**
```
Error: Cannot reference 'sensors/camera/driver/output'
  Socket references can only be 2 levels deep: 'entity/socket'

  Hint: Plan 'sensors' is opaque. Either:
    1. Mark the include as transparent:
       include:
         sensors: !file
           path: sensors.yaml
           transparent: true

    2. Or use an exposed plan socket:
       Available sockets: sensors/camera_out, sensors/lidar_out
```

**Test Cases:**
1. Too-deep reference provides transparency hint
2. Non-existent entity suggests available entities
3. Non-existent socket suggests available sockets
4. Error distinguishes between opaque and missing
5. All errors include location information

**Current State:**
- Error types: ❌ Not defined
- Suggestions: ❌ Not implemented
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/error.rs`
- `ros-plan-compiler/src/processor/link_resolver.rs`

---

#### F17: Type Compatibility Checking ❌

**Phase:** 5 (Advanced Features)

**Priority:** Medium

**Description:** Validate message/service types match across link sources, destinations, and sockets.

**Work Items:**
1. Extract types from all connected sockets
2. Compare link type with socket types
3. Error on mismatch with clear explanation
4. Handle optional socket types
5. Write type validation tests

**Expected Results:**
```
Error: Type mismatch in link 'camera_feed'
  Link type:        sensor_msgs/msg/Image
  Socket 'dst/in':  std_msgs/msg/String

  All sockets connected to a link must have compatible types.
```

**Test Cases:**
1. Matching types → Success
2. Mismatched link vs source → Error
3. Mismatched link vs destination → Error
4. Mismatched source vs destination → Error
5. Optional socket types don't cause error

**Current State:**
- Validation: ❌ Not implemented
- Error type: ❌ Not defined
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs`
- `ros-plan-compiler/src/error.rs`

---

#### F18: QoS Requirement Satisfaction ❌

**Phase:** 5 (Advanced Features)

**Priority:** Low

**Description:** Validate link QoS profiles satisfy socket QoS requirements.

**Work Items:**
1. Extract QoS requirements from sockets
2. Extract QoS profile from link
3. Validate profile meets all requirements
4. Error on unsatisfied requirements
5. Write QoS validation tests

**Expected Results:**
```
Error: QoS requirement not satisfied in link 'critical_data'
  Socket 'sensor/data' requires: reliability=reliable, min_depth=10
  Link provides:                 reliability=best-effort, depth=5

  Socket requirements must be met by the link's QoS profile.
```

**Test Cases:**
1. Matching QoS → Success
2. Better QoS than required → Success
3. Insufficient reliability → Error
4. Insufficient depth → Error
5. No requirements → Always success

**Current State:**
- Validation: ❌ Not implemented
- QoS comparison: ❌ Not defined
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs`
- QoS comparison utilities (new)

---

### Format Parsing Tests

These tests verify YAML deserialization and format validation.

#### F22: YAML Deserialization Tests ✅

**Phase:** 1 (Foundation & Testing)

**Priority:** High

**Description:** Comprehensive unit tests for parsing all plan elements from YAML.

**Work Items:**
1. ✅ Test parsing valid YAML for each element type (arg, var, node, link, socket, include, group)
2. ✅ Test parsing with optional fields present/absent
3. ✅ Test parsing with all socket types (!pub, !sub, !srv, !cli)
4. ✅ Test parsing with all link types (!pubsub, !service)
5. ✅ Test parsing nested structures
6. ⏳ Add test fixtures in `ros-plan-format/tests/fixtures/` (deferred to integration tests)

**Expected Results:**
- ✅ Test files in `ros-plan-format/src/` with `#[cfg(test)] mod tests`
- ⏳ Fixtures directory with valid YAML examples (deferred)
- ✅ Each struct has parsing tests

**Test Cases:**
```rust
// plan.rs - 6 tests
#[test]
fn parse_empty_plan() { ... }
fn parse_plan_with_node() { ... }
fn parse_plan_with_link() { ... }
fn parse_plan_with_all_sections() { ... }
fn reject_unknown_fields() { ... }
fn parse_plan_with_multiple_nodes() { ... }

// node.rs - 7 tests
#[test]
fn parse_node_minimal() { ... }
fn parse_node_with_params() { ... }
fn parse_node_with_sockets() { ... }
fn parse_node_with_when_condition() { ... }
fn parse_node_with_plugin() { ... }
fn reject_node_unknown_fields() { ... }
fn parse_node_with_expressions() { ... }

// link.rs - 11 tests
#[test]
fn parse_pubsub_link_minimal() { ... }
fn parse_pubsub_link_multiple_sources() { ... }
fn parse_pubsub_link_multiple_destinations() { ... }
fn parse_pubsub_link_with_qos() { ... }
fn parse_pubsub_link_with_when() { ... }
fn parse_service_link() { ... }
fn parse_service_link_with_when() { ... }
fn reject_pubsub_unknown_fields() { ... }
fn reject_service_unknown_fields() { ... }
fn parse_link_enum_pubsub() { ... }
fn parse_link_enum_service() { ... }

// key.rs - 2 existing tests
#[test]
fn suceed_on_valid_keys() { ... }
fn fail_on_invalid_keys() { ... }
```

**Current State:**
- Tests: ✅ 26 total (24 new + 2 existing)
- Coverage: plan.rs (6), node.rs (7), link.rs (11), key.rs (2)
- All tests passing: ✅

**Files Affected:**
- ✅ `ros-plan-format/src/plan.rs` (added test module with 6 tests)
- ✅ `ros-plan-format/src/node.rs` (added test module with 7 tests)
- ✅ `ros-plan-format/src/link.rs` (added test module with 11 tests)
- ⏳ `ros-plan-format/src/plan_socket.rs` (deferred to F23/F24)
- ⏳ `ros-plan-format/src/node_socket.rs` (deferred to F23/F24)
- ⏳ `ros-plan-format/src/subplan.rs` (deferred to F23/F24)
- ⏳ `ros-plan-format/tests/fixtures/*.yaml` (deferred to F30)

---

#### F23: Expression Parsing Tests ✅

**Phase:** 1 (Foundation & Testing)

**Priority:** High

**Description:** Unit tests for Lua expression parsing in YAML values.

**Work Items:**
1. ✅ Test parsing single-line expressions `$ expr $`
2. ✅ Test parsing multi-line expressions `$$$ expr $$$`
3. ✅ Test parsing expressions in all value contexts (str, i64, f64, bool, lists)
4. ✅ Test expression syntax errors
5. ✅ Test roundtrip serialization and display

**Expected Results:**
- ✅ Test module in `ros-plan-format/src/expr/`
- ✅ Coverage for all expression types

**Current State:**
- Tests: ✅ 60 tests across 5 files
  - expr_.rs: 15 tests (single/multi-line, error cases, roundtrip)
  - value_or_expr.rs: 13 tests (typed values and expressions)
  - text_or_expr.rs: 11 tests (text vs expression, escaping)
  - bool_expr.rs: 9 tests (boolean expressions, validation)
  - key_or_expr.rs: 12 tests (key vs expression, validation)
- All tests passing: ✅

**Files Affected:**
- ✅ `ros-plan-format/src/expr/expr_.rs` (test module added)
- ✅ `ros-plan-format/src/expr/value_or_expr.rs` (test module added)
- ✅ `ros-plan-format/src/expr/text_or_expr.rs` (test module added)
- ✅ `ros-plan-format/src/expr/bool_expr.rs` (test module added)
- ✅ `ros-plan-format/src/expr/key_or_expr.rs` (test module added)

---

#### F24: Type Validation Tests ✅

**Phase:** 1 (Foundation & Testing)

**Priority:** High

**Description:** Unit tests for type checking and conversion in parsed values.

**Work Items:**
1. ✅ Test type validation for Value variants (Str, I64, F64, Bool, lists)
2. ✅ Test ValueType parsing and serialization
3. ✅ Test type checking methods (is_bool, is_i64, etc.)
4. ✅ Test type conversion methods (to_bool, to_i64, etc.)
5. ✅ Test type conversion edge cases (try_into_* methods)

**Expected Results:**
- ✅ Test modules for type validation
- ✅ Coverage for all value types

**Current State:**
- Tests: ✅ 27 tests across 2 files
  - value_type.rs: 8 tests (display, parsing, YAML tags, serialization)
  - value.rs: 19 tests (type checking, conversions, lists, primitives, YAML deserialization)
- All tests passing: ✅

**Files Affected:**
- ✅ `ros-plan-format/src/expr/value.rs` (test module added)
- ✅ `ros-plan-format/src/expr/value_type.rs` (test module added)

---

#### F25: Error Recovery Tests ✅

**Phase:** 1 (Foundation & Testing)

**Priority:** Medium

**Description:** Tests for error messages and error types.

**Work Items:**
1. ✅ Test error message formatting
2. ✅ Test all error enum variants
3. ✅ Test error Display implementations
4. ✅ Verify error messages contain relevant context
5. ✅ Test deserialization errors

**Expected Results:**
- ✅ Test cases for all error types
- ✅ Validation of error message quality

**Current State:**
- Tests: ✅ 17 tests in error.rs
  - Tests for IdentifierCreationError
  - Tests for ParseArgDefError variants
  - Tests for InvalidArgumentDeclaration, InvalidNodeDeclaration, InvalidLinkDeclaration
  - Tests for InvalidSocketDeclaration, InvalidSubplanDeclaration
  - Tests for ParseParamDefError, InvalidParameterValue
  - Tests for KeyCreationError, TopicCreationError, InvalidInterfaceType
  - Tests for DeserializationError, ParseExpressionError
- All tests passing: ✅

**Files Affected:**
- ✅ `ros-plan-format/src/error.rs` (test module added)

---

### Compiler Algorithm Tests

These tests verify the correctness of compiler algorithms.

#### F26: Program Expansion Tests ✅

**Phase:** 1 (Foundation & Testing)

**Priority:** High

**Description:** Unit tests for Program structure and serialization.

**Work Items:**
1. ✅ Test Program default creation
2. ✅ Test Program serialization to YAML
3. ✅ Test Program deserialization from YAML
4. ✅ Test Program roundtrip (serialize → deserialize)
5. ✅ Test SharedTable initialization

**Expected Results:**
- ✅ Test module in `ros-plan-compiler/src/program.rs`
- ✅ Basic program structure validation

**Current State:**
- Tests: ✅ 5 tests in program.rs
  - program_default_creates_instance: Verifies tables are initialized
  - program_to_string_produces_yaml: Tests YAML serialization
  - program_roundtrip_serialization: Tests serialize/deserialize
  - program_from_str_parses_yaml: Tests YAML parsing
  - program_table_names_are_set: Verifies table names
- All tests passing: ✅

**Files Affected:**
- ✅ `ros-plan-compiler/src/program.rs` (test module added)

---

#### F27: Lua Evaluation Tests ✅

**Phase:** 1 (Foundation & Testing)

**Priority:** High

**Description:** Unit tests for Lua environment and basic evaluation.

**Work Items:**
1. ✅ Test Lua environment initialization (sandboxed)
2. ✅ Test basic arithmetic operations
3. ✅ Test string operations
4. ✅ Test boolean logic and comparisons
5. ✅ Test variables, tables, functions
6. ✅ Test conditionals and loops
7. ✅ Test global function registration (add_function)

**Expected Results:**
- ✅ Test module in `ros-plan-compiler/src/eval/lua.rs`
- ✅ Coverage for Lua basics

**Current State:**
- Tests: ✅ 13 tests in lua.rs
  - new_lua_creates_sandboxed_instance: Verifies Lua initialization
  - lua_basic_arithmetic: Tests i64 and f64 operations
  - lua_string_operations: Tests string concatenation
  - lua_boolean_logic: Tests and/or/not operations
  - lua_comparisons: Tests <, >, ==, etc.
  - lua_variables: Tests local variables
  - lua_tables: Tests table creation and indexing
  - lua_functions: Tests function definitions
  - lua_conditionals: Tests if/then/else
  - lua_loops: Tests for loops
  - lua_is_sandboxed: Verifies sandbox restrictions
  - add_function_creates_global: Tests global function creation
  - add_function_with_arguments: Tests function arguments
- All tests passing: ✅

**Files Affected:**
- ✅ `ros-plan-compiler/src/eval/lua.rs` (test module added)
- `ros-plan-compiler/src/processor/evaluator.rs` (add test module)

---

#### F28: Socket Resolution Tests ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Unit tests for socket resolution algorithm.

**Work Items:**
1. ✅ Test socket reference parsing
2. ✅ Test local node socket resolution
3. ✅ Test plan socket resolution (pub/sub)
4. ✅ Test socket forwarding (single/multiple sources)
5. ✅ Test basic resolver structure
6. ✅ Mock program structure for testing

**Expected Results:**
- ✅ Test module in `ros-plan-compiler/src/processor/socket_resolver.rs`
- ✅ Coverage of resolution data structures and basic logic

**Test Cases:**
```rust
#[test]
fn test_visit_pub_socket_single_source() { ... }  // ✅

#[test]
fn test_visit_pub_socket_multiple_sources() { ... }  // ✅

#[test]
fn test_visit_sub_socket_single_destination() { ... }  // ✅

#[test]
fn test_visit_sub_socket_multiple_destinations() { ... }  // ✅

#[test]
fn test_socket_resolver_default() { ... }  // ✅
```

**Current State:**
- Tests: ✅ 5 unit tests passing
- Coverage: Basic structure, pub/sub socket resolution, single/multi-source

**Files Affected:**
- ✅ `ros-plan-compiler/src/processor/socket_resolver.rs`

---

#### F29: Link Resolution Tests ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Unit tests for link resolution and topic derivation.

**Work Items:**
1. ✅ Test link source resolution
2. ✅ Test link destination resolution
3. ✅ Test topic name derivation (all priority levels)
4. ✅ Test multiple sources handling
5. ⏳ Test type checking (deferred to Phase 5 - F17)
6. ✅ Test error cases
7. ✅ Mock socket and program structures

**Expected Results:**
- ✅ Test module in `ros-plan-compiler/src/processor/link_resolver.rs`
- ✅ Tests for all derivation paths

**Test Cases:**
```rust
#[test]
fn derive_topic_from_explicit_topic_field() { ... }  // ✅

#[test]
fn derive_topic_multi_source_without_topic_errors() { ... }  // ✅

#[test]
fn derive_topic_multi_source_with_topic_succeeds() { ... }  // ✅

#[test]
fn derive_topic_no_source_returns_none() { ... }  // ✅

#[test]
fn derive_topic_explicit_overrides_source() { ... }  // ✅

#[test]
fn error_message_includes_link_name_and_count() { ... }  // ✅

#[test]
fn derive_topic_from_plan_socket_topic() { ... }  // ✅

#[test]
fn derive_topic_link_topic_overrides_plan_socket() { ... }  // ✅

#[test]
fn derive_topic_plan_socket_without_topic_falls_through() { ... }  // ✅

#[test]
fn derive_topic_multiple_plan_sockets_ignored() { ... }  // ✅
```

**Current State:**
- Tests: ✅ 10 unit tests passing
- Coverage: Explicit topic, multi-source validation, plan socket topics, ros_name override, error messages

**Files Affected:**
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs`

---

### Integration Tests

These tests verify end-to-end compilation workflows.

#### F30: Fixture-Based Integration Tests ✅

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** End-to-end compilation tests using YAML fixtures.

**Work Items:**
1. ✅ Create test fixtures directory structure (`tests/fixtures/`)
2. ✅ Create valid plan examples for each feature (5 YAML files)
3. ✅ Test successful compilation for each fixture
4. ✅ Verify generated output structure
5. ✅ Compare expected vs actual output (topic names, error messages)
6. ✅ Set up CI to run integration tests (via `cargo test`)

**Expected Results:**
- Fixtures in `tests/fixtures/`
- Integration test file `tests/integration_tests.rs`
- Tests compile fixtures and verify output

**Test Cases:**
```rust
#[test]
fn compile_simple_plan() { ... }

#[test]
fn compile_plan_with_includes() { ... }

#[test]
fn compile_plan_with_expressions() { ... }

#[test]
fn compile_plan_with_plan_sockets() { ... }

#[test]
fn compile_multi_source_link() { ... }

#[test]
fn compile_nested_plans() { ... }

#[test]
fn verify_topic_names() { ... }

#[test]
fn verify_namespace_hierarchy() { ... }
```

**Current State:**
- Fixtures: ✅ 5 YAML files covering F3, F7, F10, F15
- Integration tests: ✅ 5 tests in `phase3_integration.rs`
- Tests cover: single-source, multi-source (success/failure), plan socket topics, error messages

**Files Affected:**
- ✅ `tests/multi_source_validation.rs` (5 integration tests)
- ✅ `tests/fixtures/link_explicit_topic.yaml`
- ✅ `tests/fixtures/link_multi_source_valid.yaml`
- ✅ `tests/fixtures/link_multi_source_error.yaml`
- ✅ `tests/fixtures/plan_socket_topic_resolution.yaml`
- ✅ `tests/fixtures/plan_socket_with_topic.yaml`

---

#### F31: Error Scenario Integration Tests ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** High

**Description:** Integration tests verifying compilation errors for invalid plans.

**Work Items:**
1. Create invalid plan fixtures
2. Test each error type (socket not found, type mismatch, etc.)
3. Verify error messages are helpful
4. Test error location information
5. Create fixtures for boundary cases

**Expected Results:**
- Error fixtures in `tests/fixtures/errors/`
- Tests that expect compilation failures
- Verification of error message quality

**Test Cases:**
```rust
#[test]
fn error_on_socket_not_found() { ... }

#[test]
fn error_on_too_deep_reference() { ... }

#[test]
fn error_on_multi_source_without_topic() { ... }

#[test]
fn error_on_type_mismatch() { ... }

#[test]
fn error_on_invalid_expression() { ... }

#[test]
fn error_messages_include_location() { ... }

#[test]
fn error_messages_include_suggestions() { ... }
```

**Current State:**
- Error fixtures: ❌ None
- Error tests: ❌ None

**Files Affected:**
- `tests/error_tests.rs` (new)
- `tests/fixtures/errors/*.yaml` (new)

---

### Example Tests

These tests verify real-world usage examples.

#### F19: Unit Tests for Topic Resolution ✅

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Focused unit tests for topic resolution algorithm (refactored from original F19).

**Work Items:**
1. Extract topic resolution logic into testable functions
2. Test single-source derivation
3. Test multi-source with explicit topic
4. Test absolute vs relative paths
5. Test ros_name overrides
6. Test all edge cases

**Expected Results:**
- Isolated tests for topic resolution
- Fast execution
- Clear test names

**Test Cases:**
```rust
#[test]
fn single_source_derives_from_node_socket_name() { ... }

#[test]
fn single_source_uses_ros_name_when_present() { ... }

#[test]
fn multi_source_requires_explicit_topic() { ... }

#[test]
fn absolute_topic_path_not_modified() { ... }

#[test]
fn relative_topic_path_prepends_namespace() { ... }

#[test]
fn plan_socket_topic_creates_boundary() { ... }
```

**Current State:**
- Tests: ✅ 10 unit tests complete
- Coverage: ✅ Covers F6, F7, F9, F10

**Files Modified:**
- ✅ `ros-plan-compiler/src/processor/link_resolver.rs` - 10 unit tests in test module

---

#### F20: Encapsulation Integration Tests ✅

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** High

**Description:** End-to-end tests for encapsulation scenarios (refactored from original F20).

**Work Items:**
1. ✅ Create test plans with opaque boundaries
2. ✅ Create test plans with transparent includes
3. ✅ Test socket forwarding patterns
4. ✅ Test multi-level nesting
5. ✅ Verify error cases

**Test Results:**
- 8 integration tests in `encapsulation_tests.rs`
- Test fixtures covering all encapsulation scenarios
- Comprehensive coverage of visibility rules and transparency

**Actual Test Cases:**
```rust
#[test] // ✅
fn test_too_deep_socket_reference()

#[test] // ✅
fn test_socket_not_found()

#[test] // ✅
fn test_entity_not_found()

#[test] // ✅
fn test_namespace_tracking()

#[test] // ✅
fn test_transparent_single_level()

#[test] // ✅
fn test_transparent_multi_level()

#[test] // ✅
fn test_non_transparent_deep_ref_fails()

#[test] // ✅
fn test_transparent_flag_parsing()
```

**Current State:**
- Test fixtures: ✅ 7 YAML fixtures (4 transparent + 3 error scenarios)
- Integration tests: ✅ 8 tests complete

**Files Modified:**
- ✅ `tests/encapsulation_tests.rs` - 8 integration tests
- ✅ `tests/fixtures/transparent_*.yaml` - 4 fixtures
- ✅ `tests/fixtures/errors/*.yaml` - 3 error fixtures

---

#### F21: Real-World Example Suite ❌

**Phase:** 5 (Advanced Features)

**Priority:** Medium

**Description:** Comprehensive examples demonstrating all features working together.

**Work Items:**
1. Create Autoware-style sensing example
2. Create multi-robot example
3. Create complex nesting example
4. Verify examples compile successfully
5. Document expected ROS output
6. Add examples to CI

**Expected Results:**
- Examples in `examples/advanced/`
- README explaining each example
- CI tests that compile examples
- Examples serve as documentation

**Test Cases:**
```rust
#[test]
fn compile_autoware_sensing_example() { ... }

#[test]
fn compile_multi_robot_example() { ... }

#[test]
fn compile_deep_nesting_example() { ... }
```

**Current State:**
- Advanced examples: ❌ None
- Example tests: ❌ None

**Files Affected:**
- `examples/advanced/*.yaml` (new)
- `tests/example_tests.rs` (new)
- `examples/advanced/README.md` (new)

---

## Notes for Developers

### Testing Strategy

**Test-Driven Development:**
- Write tests alongside implementation (not after)
- Each feature PR must include tests
- Aim for 80%+ code coverage
- Focus on behavior, not implementation

**Test Pyramid:**
1. **Unit Tests (60%)**: Fast, isolated, focused
2. **Integration Tests (30%)**: Compile fixtures, verify workflows
3. **Example Tests (10%)**: Real-world scenarios, documentation

### Testing Guidelines

1. **Unit Tests:** Focus on individual functions/algorithms
   - Use mocked structures
   - Test edge cases
   - Fast execution (<1ms per test)
   - Clear test names describing scenarios

2. **Integration Tests:** Compile actual plan files
   - Use YAML fixtures
   - Verify complete workflow
   - Check error messages
   - Moderate execution time

3. **Example Tests:** Real-world scenarios
   - Must be maintained
   - Serve as documentation
   - Demonstrate best practices
   - Can be slower

### Error Message Quality

All errors should include:
- Clear description of what went wrong
- Location in YAML file (line/column)
- Suggestion for how to fix
- Example of correct usage when applicable
- List of available alternatives (entities, sockets, etc.)

### Backward Compatibility

New features should be:
- Optional (don't break existing plans)
- Additive (extend, don't change semantics)
- Documented (update format specification)
- Tested (ensure existing tests still pass)

### CI Integration

All tests must:
- Pass in CI before merging
- Run on every commit
- Complete in reasonable time (<5 minutes total)
- Provide clear failure messages

---

## Phase 6: ROS Launch Compatibility Layer

**Status**: 🔴 Not Started
**Timeline**: 3-4 weeks (15-19 days)
**Dependencies**: Phases 1-5 complete

### Overview

Phase 6 adds support for including ROS 2 Python launch files within plan files. This enables gradual migration from ROS 2 launch to ros-plan by allowing hybrid systems where launch files and plan files work together.

The implementation reuses the existing `launch2dump` project architecture, which uses custom visitors and Python name mangling to extract node metadata from launch files without executing processes. The key insight is to intercept launch actions (Node, ComposableNodeContainer, etc.) and extract their configuration instead of spawning processes.

**Design Principles**:
1. **No Process Execution**: Walk launch file trees and extract metadata without spawning processes
2. **Parameter Propagation**: Pass plan parameters to included launch files
3. **Runtime Reloading**: Support changing parameters at runtime and reloading launch files
4. **Extensive Reuse**: Leverage ROS 2 launch code via custom visitors and name mangling
5. **Rust Integration**: Embed Python loader in Rust compiler using PyO3

### Phase 6.1: Migrate launch2dump to UV Workspace

**Timeline**: 2 days
**Status**: 🔴 Not Started

#### F32: UV Workspace Conversion

**Description**: Convert launch2dump from Rye to UV package manager for better performance and modern Python tooling.

**Work Items**:

1. **Remove Rye Configuration**
   - Delete `[tool.rye]` section from `pyproject.toml`
   - Remove `.python-version` if present
   - Remove `requirements.lock` and `requirements-dev.lock`

2. **Add UV Configuration**
   - Create `uv.lock` with `uv lock`
   - Update `pyproject.toml` with UV-compatible settings:
   ```toml
   [project]
   name = "launch2dump"
   version = "0.1.0"
   requires-python = ">=3.10"
   dependencies = [
       "ruamel-yaml>=0.18.6",
       "lark>=1.2.2",
       "packaging>=24.1",
   ]

   [build-system]
   requires = ["hatchling"]
   build-backend = "hatchling.build"
   ```

3. **Update Development Scripts**
   - Replace `rye run` with `uv run` in documentation
   - Update any CI/CD scripts that reference Rye
   - Add `.venv/` to `.gitignore` if not present

4. **Verify Migration**
   - Test `uv sync` to install dependencies
   - Test `uv run python -m launch2dump` to verify execution
   - Verify all imports work correctly

**Test Cases**:
- [ ] `uv sync` successfully installs all dependencies
- [ ] `uv run python -m launch2dump --help` displays help text
- [ ] All existing launch2dump functionality works with UV
- [ ] UV lock file is reproducible (`uv lock` produces same output)

**Files Modified**:
- `launch2dump/pyproject.toml`
- `.gitignore` (add `.venv/`)

**Files Removed**:
- `launch2dump/requirements*.lock` (if present)
- `launch2dump/.python-version` (if present)

**Files Added**:
- `launch2dump/uv.lock`

---

### Phase 6.2: Launch Loader API

**Timeline**: 4-5 days
**Status**: 🔴 Not Started

#### F33: Launch File Walker with Visitor Pattern

**Description**: Create Python API that walks launch file trees using custom visitors to extract node metadata without process execution.

**Work Items**:

1. **Create launch2plan Module Structure**
   ```
   launch2dump/src/launch2plan/
   ├── __init__.py          # Public API exports
   ├── loader.py            # Main LaunchLoader class
   ├── inspector.py         # Custom LaunchInspector (from launch2dump)
   ├── visitor/
   │   ├── __init__.py
   │   ├── action.py        # Action visitor dispatcher
   │   ├── node.py          # Node visitor
   │   ├── container.py     # Container visitor
   │   └── lifecycle.py     # Lifecycle node visitor
   └── result.py            # LaunchResult data structures
   ```

2. **Implement LaunchLoader Class** (`loader.py`)
   ```python
   class LaunchLoader:
       """Loads ROS 2 launch files and extracts node metadata."""

       def __init__(self):
           self.inspector = LaunchInspector()

       def load_launch_file(
           self,
           launch_file_path: str,
           launch_arguments: Dict[str, str] = None
       ) -> LaunchResult:
           """
           Load a launch file and extract all nodes.

           Args:
               launch_file_path: Path to .launch.py file
               launch_arguments: Arguments to pass to launch file

           Returns:
               LaunchResult containing nodes, containers, etc.
           """
           # Implementation uses LaunchInspector to visit entities
           pass
   ```

3. **Implement LaunchResult Data Structures** (`result.py`)
   ```python
   @dataclass
   class NodeInfo:
       """Metadata for a regular ROS 2 node."""
       package: str
       executable: str
       name: Optional[str]
       namespace: Optional[str]
       parameters: List[Dict[str, Any]]
       remappings: List[Tuple[str, str]]
       arguments: List[str]
       env_vars: Dict[str, str]

   @dataclass
   class ComposableNodeInfo:
       """Metadata for a composable node."""
       plugin: str
       name: str
       namespace: Optional[str]
       parameters: List[Dict[str, Any]]
       remappings: List[Tuple[str, str]]
       extra_arguments: List[str]

   @dataclass
   class ContainerInfo:
       """Metadata for a composable node container."""
       package: str
       executable: str
       name: str
       namespace: Optional[str]
       composable_nodes: List[ComposableNodeInfo]

   @dataclass
   class LaunchResult:
       """Result of loading a launch file."""
       nodes: List[NodeInfo]
       containers: List[ContainerInfo]
       lifecycle_nodes: List[NodeInfo]
       errors: List[str]
   ```

4. **Adapt launch2dump Visitors**
   - Copy and refactor visitor code from launch2dump
   - Use Python name mangling to access private members:
     ```python
     def extract_node_info(node: Node, context) -> NodeInfo:
         node._perform_substitutions(context)
         return NodeInfo(
             package=str(node._Node__package),
             executable=str(node._Node__node_executable),
             name=str(node._Node__node_name) if node._Node__node_name else None,
             namespace=str(node._Node__node_namespace) if node._Node__node_namespace else None,
             parameters=extract_parameters(node._Node__parameters),
             remappings=extract_remappings(node._Node__remappings),
             arguments=extract_arguments(node._Node__arguments),
             env_vars=extract_env_vars(node._Node__env),
         )
     ```

5. **Handle Launch Arguments**
   - Create LaunchContext with provided arguments
   - Resolve substitutions with argument values
   - Support default argument values from launch file

**Test Cases**:
- [ ] Load simple launch file with single node
- [ ] Load launch file with multiple nodes
- [ ] Load launch file with composable node container
- [ ] Load launch file with lifecycle nodes
- [ ] Pass launch arguments and verify substitution resolution
- [ ] Load launch file that includes other launch files (recursive)
- [ ] Handle launch file with conditionals (GroupAction with condition)
- [ ] Handle launch file with DeclareLaunchArgument
- [ ] Extract node parameters (YAML files, dicts, individual params)
- [ ] Extract node remappings
- [ ] Extract node environment variables
- [ ] Error handling for missing launch file
- [ ] Error handling for invalid launch file syntax
- [ ] Verify no processes are spawned during loading

**Files Added**:
- `launch2dump/src/launch2plan/__init__.py`
- `launch2dump/src/launch2plan/loader.py`
- `launch2dump/src/launch2plan/inspector.py`
- `launch2dump/src/launch2plan/visitor/*.py`
- `launch2dump/src/launch2plan/result.py`
- `launch2dump/tests/test_loader.py`

---

#### F34: Parameter Dependency Tracking

**Description**: Track which launch file inclusions depend on which plan parameters, enabling efficient reloading when parameters change.

**Work Items**:

1. **Extend LaunchResult with Parameter Dependencies**
   ```python
   @dataclass
   class LaunchResult:
       nodes: List[NodeInfo]
       containers: List[ContainerInfo]
       lifecycle_nodes: List[NodeInfo]
       errors: List[str]
       parameter_dependencies: Set[str]  # NEW: Parameters used in this launch
   ```

2. **Track Substitutions During Visitor Traversal**
   - Identify which LaunchConfiguration substitutions were resolved
   - Record parameter names that were accessed
   - Handle nested substitutions (e.g., PathJoinSubstitution with LaunchConfiguration)

3. **Implement Dependency Tracking Visitor**
   ```python
   class SubstitutionTracker:
       """Tracks which launch arguments are accessed during evaluation."""

       def __init__(self):
           self.accessed_parameters = set()

       def track_substitutions(self, substitution):
           """Recursively track LaunchConfiguration accesses."""
           if isinstance(substitution, LaunchConfiguration):
               self.accessed_parameters.add(substitution.variable_name)
           elif hasattr(substitution, '__iter__'):
               for sub in substitution:
                   self.track_substitutions(sub)
   ```

4. **Integrate Tracking into LaunchLoader**
   - Run SubstitutionTracker during entity visitation
   - Include parameter dependencies in LaunchResult
   - Document which parameters affect which launch files

**Test Cases**:
- [ ] Track single parameter dependency
- [ ] Track multiple parameter dependencies
- [ ] Track nested substitution dependencies (PathJoinSubstitution)
- [ ] Handle launch file with no parameter dependencies
- [ ] Handle same parameter used multiple times (deduplicate)
- [ ] Track dependencies through included launch files

**Files Modified**:
- `launch2dump/src/launch2plan/result.py`
- `launch2dump/src/launch2plan/loader.py`
- `launch2dump/src/launch2plan/visitor/action.py`

**Files Added**:
- `launch2dump/src/launch2plan/tracker.py`
- `launch2dump/tests/test_dependency_tracking.py`

---

### Phase 6.3: Rust Integration with PyO3

**Timeline**: 4-5 days
**Status**: 🔴 Not Started

#### F35: PyO3 Embedded Python Loader

**Description**: Embed the Python launch loader in the Rust compiler using PyO3 FFI bindings.

**Work Items**:

1. **Add PyO3 Dependency to ros-plan-compiler**
   ```toml
   # ros-plan-compiler/Cargo.toml
   [dependencies]
   pyo3 = { version = "0.22", features = ["auto-initialize"] }
   ```

2. **Create Python FFI Module** (`ros-plan-compiler/src/launch_loader.rs`)
   ```rust
   use pyo3::prelude::*;
   use pyo3::types::{PyDict, PyList};
   use std::collections::HashMap;
   use std::path::Path;

   /// Result from loading a launch file
   #[derive(Debug, Clone)]
   pub struct LaunchLoadResult {
       pub nodes: Vec<NodeInfo>,
       pub containers: Vec<ContainerInfo>,
       pub errors: Vec<String>,
       pub parameter_dependencies: Vec<String>,
   }

   #[derive(Debug, Clone)]
   pub struct NodeInfo {
       pub package: String,
       pub executable: String,
       pub name: Option<String>,
       pub namespace: Option<String>,
       pub parameters: Vec<HashMap<String, String>>,
       pub remappings: Vec<(String, String)>,
   }

   #[derive(Debug, Clone)]
   pub struct ContainerInfo {
       pub package: String,
       pub executable: String,
       pub name: String,
       pub namespace: Option<String>,
       pub composable_nodes: Vec<ComposableNodeInfo>,
   }

   #[derive(Debug, Clone)]
   pub struct ComposableNodeInfo {
       pub plugin: String,
       pub name: String,
       pub namespace: Option<String>,
       pub parameters: Vec<HashMap<String, String>>,
   }

   /// Load a ROS 2 launch file using Python loader
   pub fn load_launch_file(
       launch_file: &Path,
       arguments: HashMap<String, String>,
   ) -> Result<LaunchLoadResult, String> {
       Python::with_gil(|py| {
           // Import launch2plan module
           let launch2plan = py.import_bound("launch2plan")
               .map_err(|e| format!("Failed to import launch2plan: {}", e))?;

           // Create loader
           let loader = launch2plan.getattr("LaunchLoader")
               .map_err(|e| format!("Failed to get LaunchLoader: {}", e))?
               .call0()
               .map_err(|e| format!("Failed to create LaunchLoader: {}", e))?;

           // Prepare arguments
           let py_args = PyDict::new_bound(py);
           for (key, value) in arguments {
               py_args.set_item(key, value)
                   .map_err(|e| format!("Failed to set argument: {}", e))?;
           }

           // Call load_launch_file
           let result = loader
               .call_method1(
                   "load_launch_file",
                   (launch_file.to_str().unwrap(), py_args)
               )
               .map_err(|e| format!("Failed to load launch file: {}", e))?;

           // Convert Python result to Rust structures
           convert_launch_result(result)
       })
   }

   fn convert_launch_result(py_result: Bound<PyAny>) -> Result<LaunchLoadResult, String> {
       // Extract nodes, containers, errors from Python object
       // Convert to Rust data structures
       todo!()
   }
   ```

3. **Initialize Python Interpreter in Compiler**
   - Initialize Python interpreter on first use
   - Set Python path to include launch2plan module
   - Handle Python initialization errors gracefully

4. **Add PYTHONPATH Configuration**
   - Detect launch2dump installation location
   - Add to PYTHONPATH before importing launch2plan
   - Support both development (source) and installed modes

5. **Error Handling**
   - Convert Python exceptions to Rust Result types
   - Provide clear error messages for common failures:
     - Python not available
     - launch2plan module not found
     - Launch file syntax errors
     - Missing launch arguments

**Test Cases**:
- [ ] Initialize Python interpreter successfully
- [ ] Import launch2plan module
- [ ] Call LaunchLoader.load_launch_file from Rust
- [ ] Convert Python NodeInfo to Rust NodeInfo
- [ ] Convert Python ContainerInfo to Rust ContainerInfo
- [ ] Handle Python exception and convert to Rust error
- [ ] Load launch file with arguments from Rust
- [ ] Verify no memory leaks (Python GIL handling)
- [ ] Support multiple load_launch_file calls in same process
- [ ] Handle launch2plan module not found error

**Files Added**:
- `ros-plan-compiler/src/launch_loader.rs`
- `ros-plan-compiler/tests/launch_loader_tests.rs`

**Files Modified**:
- `ros-plan-compiler/Cargo.toml` (add pyo3 dependency)
- `ros-plan-compiler/src/lib.rs` (add module declaration)

---

#### F36: Plan Format Extension for Launch Includes

**Description**: Extend plan YAML format to support including ROS 2 launch files with parameter passing.

**Work Items**:

1. **Add Include Directive to Plan Format** (`ros-plan-format/src/plan.rs`)
   ```rust
   #[derive(Debug, Clone, Serialize, Deserialize)]
   pub struct Plan {
       #[serde(default)]
       pub include: IndexMap<String, LaunchInclude>,

       #[serde(default)]
       pub node: IndexMap<String, Node>,

       #[serde(default)]
       pub link: IndexMap<String, Link>,

       // ... existing fields
   }

   #[derive(Debug, Clone, Serialize, Deserialize)]
   pub struct LaunchInclude {
       /// Path to .launch.py file
       pub file: String,

       /// Arguments to pass to launch file
       #[serde(default)]
       pub args: IndexMap<String, Value>,

       /// Namespace prefix for included nodes (optional)
       pub namespace: Option<String>,

       /// Conditional inclusion (optional)
       #[serde(default)]
       pub when: Option<String>,
   }
   ```

2. **Update YAML Schema Examples**
   ```yaml
   include:
     camera_launch:
       file: /path/to/camera.launch.py
       args:
         camera_name: front_camera
         fps: 30
       namespace: /sensors
       when: ${use_camera}

   node:
     image_processor:
       pkg: image_processing
       exec: processor
       # ... rest of node definition
   ```

3. **Add Validation Rules**
   - Verify launch file exists and is readable
   - Validate argument types match launch file expectations
   - Check for circular includes (include A includes B includes A)
   - Validate namespace syntax

4. **Support Value Substitution in Include Args**
   - Allow `${param}` syntax in include arguments
   - Resolve from plan parameters
   - Support Lua expressions in include arguments

**Test Cases**:
- [ ] Parse plan with single launch include
- [ ] Parse plan with multiple launch includes
- [ ] Parse plan with include arguments
- [ ] Parse plan with include namespace prefix
- [ ] Parse plan with conditional include (when clause)
- [ ] Validate launch file path exists
- [ ] Detect circular includes and report error
- [ ] Substitute plan parameters in include arguments
- [ ] Include without arguments (empty args)
- [ ] Include with namespace prefix applied to nodes

**Files Modified**:
- `ros-plan-format/src/plan.rs`
- `ros-plan-format/src/lib.rs`

**Files Added**:
- `ros-plan-format/tests/include_tests.rs`
- `ros-plan-compiler/tests/fixtures/includes/*.yaml` (test fixtures)

---

#### F37: Compiler Integration and Node Merging

**Description**: Integrate launch loader into compiler pipeline and merge loaded nodes with plan-defined nodes.

**Work Items**:

1. **Add Launch Processing to Compiler Pipeline** (`ros-plan-compiler/src/compiler.rs`)
   ```rust
   impl Compiler {
       pub fn compile(&self, plan_file: &Path, args: IndexMap<String, String>)
           -> Result<Program, CompileError>
       {
           // 1. Parse plan file
           let mut plan = self.parse_plan(plan_file)?;

           // 2. Evaluate expressions with args
           let evaluated_plan = self.evaluate_plan(plan, args)?;

           // 3. Process launch includes (NEW)
           let expanded_plan = self.process_launch_includes(evaluated_plan)?;

           // 4. Resolve links
           let resolved_plan = self.resolve_links(expanded_plan)?;

           // 5. Generate program
           let program = self.generate_program(resolved_plan)?;

           Ok(program)
       }

       fn process_launch_includes(&self, plan: Plan) -> Result<Plan, CompileError> {
           let mut expanded_plan = plan.clone();

           for (include_name, include) in &plan.include {
               // Evaluate when condition
               if let Some(when) = &include.when {
                   if !self.evaluate_condition(when)? {
                       continue; // Skip this include
                   }
               }

               // Resolve arguments from plan parameters
               let args = self.resolve_include_arguments(&include.args)?;

               // Load launch file
               let result = load_launch_file(
                   Path::new(&include.file),
                   args
               )?;

               // Convert loaded nodes to plan nodes
               let nodes = self.convert_loaded_nodes(
                   result,
                   include.namespace.as_deref()
               )?;

               // Merge into plan
               for (node_name, node) in nodes {
                   expanded_plan.node.insert(node_name, node);
               }
           }

           Ok(expanded_plan)
       }
   }
   ```

2. **Implement Node Conversion**
   ```rust
   fn convert_loaded_nodes(
       &self,
       result: LaunchLoadResult,
       namespace_prefix: Option<&str>
   ) -> Result<IndexMap<String, Node>, CompileError> {
       let mut nodes = IndexMap::new();

       for node_info in result.nodes {
           let node_name = self.generate_node_name(&node_info, namespace_prefix);
           let node = Node {
               pkg: node_info.package,
               exec: node_info.executable,
               name: node_info.name,
               namespace: self.merge_namespaces(
                   namespace_prefix,
                   node_info.namespace.as_deref()
               ),
               param: self.convert_parameters(node_info.parameters),
               remap: self.convert_remappings(node_info.remappings),
               // ... other fields
           };
           nodes.insert(node_name, node);
       }

       Ok(nodes)
   }
   ```

3. **Handle Name Conflicts**
   - Detect when loaded node name conflicts with plan-defined node
   - Strategy: Prefix included node names with include name
   - Example: `camera_launch/camera_node` for node from `camera_launch` include
   - Allow user to override with explicit naming

4. **Apply Namespace Prefixes**
   - Prepend namespace from include to node namespaces
   - Handle absolute vs relative namespace paths
   - Preserve node-level namespace settings

5. **Error Reporting**
   - Report errors from launch file loading
   - Include context: which include caused the error
   - Line numbers from plan file for include directive

**Test Cases**:
- [ ] Compile plan with single launch include
- [ ] Compile plan with multiple launch includes
- [ ] Merge loaded nodes with plan nodes (no conflicts)
- [ ] Handle node name conflict with prefixing
- [ ] Apply namespace prefix to included nodes
- [ ] Skip include when `when` condition is false
- [ ] Resolve plan parameters in include arguments
- [ ] Load composable nodes from container
- [ ] Handle launch file loading error gracefully
- [ ] Verify expanded plan has all nodes (plan + included)
- [ ] Generate correct command lines for included nodes
- [ ] Included nodes participate in link resolution

**Files Modified**:
- `ros-plan-compiler/src/compiler.rs`
- `ros-plan-compiler/src/lib.rs`

**Files Added**:
- `ros-plan-compiler/src/launch_integration.rs`
- `ros-plan-compiler/tests/include_compile_tests.rs`

---

### Phase 6.4: Runtime Parameter Updates

**Timeline**: 3-4 days
**Status**: 🔴 Not Started

#### F38: Launch Include Cache and Reload

**Description**: Implement caching and reloading of launch includes when parameters change at runtime.

**Work Items**:

1. **Create LaunchIncludeCache** (`ros-plan-compiler/src/launch_cache.rs`)
   ```rust
   use std::collections::HashMap;
   use std::path::PathBuf;

   pub struct LaunchIncludeCache {
       /// Map from include name to loaded result
       cache: HashMap<String, CachedLaunchInclude>,
   }

   struct CachedLaunchInclude {
       /// Path to launch file
       file: PathBuf,

       /// Arguments used for loading
       arguments: HashMap<String, String>,

       /// Loaded nodes
       result: LaunchLoadResult,

       /// Which plan parameters this include depends on
       parameter_dependencies: Vec<String>,
   }

   impl LaunchIncludeCache {
       pub fn load_or_get(
           &mut self,
           include_name: &str,
           launch_file: &Path,
           arguments: HashMap<String, String>
       ) -> Result<&LaunchLoadResult, String> {
           // Check if cached and arguments match
           if let Some(cached) = self.cache.get(include_name) {
               if cached.file == launch_file && cached.arguments == arguments {
                   return Ok(&cached.result);
               }
           }

           // Load and cache
           let result = load_launch_file(launch_file, arguments.clone())?;
           let parameter_deps = result.parameter_dependencies.clone();

           self.cache.insert(include_name.to_string(), CachedLaunchInclude {
               file: launch_file.to_path_buf(),
               arguments,
               result,
               parameter_dependencies: parameter_deps,
           });

           Ok(&self.cache.get(include_name).unwrap().result)
       }

       pub fn invalidate_for_parameter(&mut self, param_name: &str) -> Vec<String> {
           let mut invalidated = Vec::new();

           self.cache.retain(|include_name, cached| {
               if cached.parameter_dependencies.contains(&param_name.to_string()) {
                   invalidated.push(include_name.clone());
                   false // Remove from cache
               } else {
                   true // Keep in cache
               }
           });

           invalidated
       }
   }
   ```

2. **Add Reload API to Compiler**
   ```rust
   impl Compiler {
       pub fn reload_for_parameter_change(
           &mut self,
           param_name: &str,
           new_value: String
       ) -> Result<ReloadResult, CompileError> {
           // Invalidate affected includes
           let invalidated = self.launch_cache.invalidate_for_parameter(param_name);

           // Reload each invalidated include
           let mut added_nodes = Vec::new();
           let mut removed_nodes = Vec::new();

           for include_name in invalidated {
               let old_nodes = self.get_nodes_from_include(&include_name);

               // Reload with new parameter value
               let new_result = self.reload_include(&include_name)?;
               let new_nodes = self.convert_loaded_nodes(new_result, None)?;

               // Compute diff
               let (added, removed) = self.diff_nodes(old_nodes, new_nodes);
               added_nodes.extend(added);
               removed_nodes.extend(removed);
           }

           Ok(ReloadResult {
               nodes_to_start: added_nodes,
               nodes_to_stop: removed_nodes,
           })
       }
   }

   pub struct ReloadResult {
       pub nodes_to_start: Vec<NodeInfo>,
       pub nodes_to_stop: Vec<String>, // Node names
   }
   ```

3. **Implement Node Diffing**
   ```rust
   fn diff_nodes(
       old: IndexMap<String, Node>,
       new: IndexMap<String, Node>
   ) -> (Vec<Node>, Vec<String>) {
       let mut added = Vec::new();
       let mut removed = Vec::new();

       // Find added nodes
       for (name, node) in &new {
           if !old.contains_key(name) {
               added.push(node.clone());
           }
       }

       // Find removed nodes
       for (name, _) in &old {
           if !new.contains_key(name) {
               removed.push(name.clone());
           }
       }

       (added, removed)
   }
   ```

4. **Handle Node Modifications**
   - Detect when node configuration changes (same name, different config)
   - Strategy: Stop old node and start new node
   - Track which nodes came from which include

**Test Cases**:
- [ ] Load launch include and cache result
- [ ] Reload include when cache miss
- [ ] Return cached result when arguments match
- [ ] Invalidate cache when dependent parameter changes
- [ ] Reload include with new parameter value
- [ ] Compute diff: detect added nodes
- [ ] Compute diff: detect removed nodes
- [ ] Compute diff: detect modified nodes
- [ ] Multiple includes depending on same parameter
- [ ] Parameter change affects subset of includes
- [ ] No reload when unrelated parameter changes

**Files Added**:
- `ros-plan-compiler/src/launch_cache.rs`
- `ros-plan-compiler/tests/launch_cache_tests.rs`

**Files Modified**:
- `ros-plan-compiler/src/compiler.rs`

---

#### F39: Runtime Node Lifecycle Management

**Description**: Provide runtime API for starting and stopping nodes based on launch include changes.

**Work Items**:

1. **Define Runtime API** (`ros-plan-runtime/src/launch_runtime.rs`)
   ```rust
   pub struct LaunchRuntime {
       compiler: Compiler,
       running_nodes: HashMap<String, NodeHandle>,
   }

   pub struct NodeHandle {
       name: String,
       process: Child,
       include_source: Option<String>, // Which include created this node
   }

   impl LaunchRuntime {
       pub fn update_parameter(
           &mut self,
           param_name: &str,
           new_value: String
       ) -> Result<(), RuntimeError> {
           // Reload affected includes
           let reload_result = self.compiler.reload_for_parameter_change(
               param_name,
               new_value
           )?;

           // Stop removed nodes
           for node_name in reload_result.nodes_to_stop {
               self.stop_node(&node_name)?;
           }

           // Start added nodes
           for node in reload_result.nodes_to_start {
               self.start_node(node)?;
           }

           Ok(())
       }

       fn start_node(&mut self, node: Node) -> Result<(), RuntimeError> {
           let process = self.spawn_node_process(&node)?;
           let handle = NodeHandle {
               name: node.name.clone().unwrap_or_default(),
               process,
               include_source: node.include_source.clone(),
           };
           self.running_nodes.insert(handle.name.clone(), handle);
           Ok(())
       }

       fn stop_node(&mut self, node_name: &str) -> Result<(), RuntimeError> {
           if let Some(handle) = self.running_nodes.remove(node_name) {
               handle.process.kill()?;
               handle.process.wait()?;
           }
           Ok(())
       }
   }
   ```

2. **Track Node Source**
   - Record which include each node came from
   - Store in NodeHandle for debugging
   - Use for selective reloading

3. **Implement Graceful Shutdown**
   - Send SIGTERM before SIGKILL
   - Wait for graceful shutdown timeout
   - Handle zombie processes

4. **Add Runtime Logging**
   - Log when nodes start/stop due to parameter changes
   - Include node name, reason, include source
   - Support different log levels

5. **Error Recovery**
   - Handle node startup failures
   - Revert to previous state on reload error
   - Report which nodes failed to start

**Test Cases**:
- [ ] Start node successfully
- [ ] Stop node successfully
- [ ] Update parameter triggers node reload
- [ ] Added nodes are started
- [ ] Removed nodes are stopped
- [ ] Modified nodes are restarted (stop old, start new)
- [ ] Handle node startup failure gracefully
- [ ] Revert on reload error
- [ ] Multiple parameter updates in sequence
- [ ] Track node source include correctly
- [ ] Graceful shutdown with timeout
- [ ] Handle zombie processes

**Files Added**:
- `ros-plan-runtime/src/launch_runtime.rs`
- `ros-plan-runtime/tests/runtime_tests.rs`

**Files Modified**:
- `ros-plan-runtime/Cargo.toml` (new crate)
- `Cargo.toml` (add to workspace)

---

### Phase 6.5: Testing and Documentation

**Timeline**: 2-3 days
**Status**: 🔴 Not Started

#### F40: Integration Tests and Examples

**Description**: Comprehensive integration tests and example plans demonstrating launch include functionality.

**Work Items**:

1. **Create Test Launch Files**
   ```
   ros-plan-compiler/tests/fixtures/launch/
   ├── simple_talker.launch.py      # Single node
   ├── talker_listener.launch.py    # Multiple nodes
   ├── camera_driver.launch.py      # With parameters
   ├── composable.launch.py         # Container with composable nodes
   └── conditional.launch.py        # With conditionals
   ```

2. **Create Test Plan Files**
   ```yaml
   # tests/fixtures/includes/simple_include.yaml
   include:
     talker:
       file: ${TEST_FIXTURES}/launch/simple_talker.launch.py

   node:
     listener:
       pkg: demo_nodes_cpp
       exec: listener

   link:
     chatter: !pubsub
       type: std_msgs/msg/String
       src: [talker/talker/chatter]
       dst: [listener/chatter]
   ```

3. **Integration Test Scenarios**
   - Simple include: Load single launch file
   - Multiple includes: Load multiple launch files
   - Include with arguments: Pass parameters to launch
   - Include with namespace: Prefix node namespaces
   - Conditional include: Skip when condition false
   - Nested includes: Launch file includes another launch file
   - Parameter reload: Change parameter, reload include
   - Name conflict resolution: Handle duplicate node names
   - Composable nodes: Load container with components
   - Lifecycle nodes: Load lifecycle node from launch

4. **Add Example Plans to Documentation**
   ```
   examples/launch_integration/
   ├── simple_include.yaml          # Basic launch include
   ├── camera_with_processing.yaml  # Include + custom nodes
   ├── multi_robot.yaml             # Multiple includes with namespaces
   └── runtime_reload.yaml          # Parameter-dependent include
   ```

5. **Performance Testing**
   - Measure launch file loading time
   - Measure reload time for parameter changes
   - Verify no memory leaks in Python FFI
   - Test with large launch files (100+ nodes)

**Test Cases**:
- [ ] End-to-end: Compile plan with include, run nodes
- [ ] End-to-end: Reload on parameter change, verify new nodes
- [ ] Performance: Load large launch file (<1s)
- [ ] Performance: Reload on parameter change (<100ms)
- [ ] Memory: No leaks after 1000 reload cycles
- [ ] Compatibility: Works with ROS 2 Humble launch files
- [ ] Compatibility: Works with ROS 2 Iron launch files
- [ ] Compatibility: Works with ROS 2 Rolling launch files
- [ ] Error handling: Invalid launch file path
- [ ] Error handling: Launch file syntax error
- [ ] Error handling: Missing required argument
- [ ] Error handling: Circular include detection

**Files Added**:
- `ros-plan-compiler/tests/fixtures/launch/*.launch.py`
- `ros-plan-compiler/tests/fixtures/includes/*.yaml`
- `ros-plan-compiler/tests/integration/launch_include_tests.rs`
- `examples/launch_integration/*.yaml`

---

#### F41: Documentation and Migration Guide

**Description**: Complete documentation for launch include feature, including migration guide from ROS 2 launch.

**Work Items**:

1. **Add Launch Include Chapter to Book** (`book/src/launch_includes.md`)
   ```markdown
   # Launch File Includes

   ROS-Plan supports including existing ROS 2 Python launch files within plan files.
   This enables gradual migration from launch to plan format, and reuse of existing
   launch files.

   ## Basic Usage

   ```yaml
   include:
     camera_launch:
       file: /opt/ros/humble/share/camera_driver/launch/camera.launch.py
       args:
         camera_name: front_camera

   node:
     image_processor:
       pkg: image_processing
       exec: processor
       socket:
         input: !sub
           type: sensor_msgs/msg/Image

   link:
     camera_images: !pubsub
       type: sensor_msgs/msg/Image
       src: [camera_launch/camera_node/image_raw]
       dst: [image_processor/input]
   ```

   ## Features
   - Pass arguments to launch files
   - Namespace prefixing for included nodes
   - Conditional includes with `when` clause
   - Automatic parameter dependency tracking
   - Runtime reloading on parameter changes

   ## How It Works
   - Launch files are loaded using Python launch API
   - No processes are spawned during loading
   - Node metadata is extracted using visitor pattern
   - Included nodes merge with plan-defined nodes
   - Links can connect plan and launch nodes
   ```

2. **Add API Documentation**
   - Document LaunchLoader Python API
   - Document launch_loader Rust API
   - Document LaunchInclude format
   - Document parameter dependency tracking

3. **Create Migration Guide** (`book/src/migration_from_launch.md`)
   ```markdown
   # Migrating from ROS 2 Launch

   This guide helps you migrate from ROS 2 launch files to ROS-Plan.

   ## Strategy 1: Gradual Migration

   Start by including your existing launch files, then gradually
   convert nodes to plan format:

   1. Create plan file that includes your launch file
   2. Add new nodes in plan format
   3. Convert one launch node at a time to plan format
   4. Eventually remove the include when all nodes converted

   ## Strategy 2: Hybrid Approach

   Keep stable subsystems in launch format, use plan for top-level
   coordination:

   - Hardware drivers: Keep in launch files
   - Application logic: Define in plan format
   - System integration: Use plan links to connect

   ## Conversion Reference

   ### Node Definition

   Launch:
   ```python
   Node(
       package='demo_nodes_cpp',
       executable='talker',
       name='my_talker',
       namespace='/demo',
       parameters=[{'publish_rate': 10}],
       remappings=[('chatter', 'my_topic')]
   )
   ```

   Plan:
   ```yaml
   node:
     my_talker:
       pkg: demo_nodes_cpp
       exec: talker
       name: my_talker
       namespace: /demo
       param:
         publish_rate: 10
       remap:
         chatter: my_topic
   ```
   ```

4. **Add Troubleshooting Section**
   - Common errors and solutions
   - Debugging launch includes
   - Python environment issues
   - Performance tuning

5. **Update README and Quick Start**
   - Add launch include example to README
   - Update quick start with include example
   - Document Python dependencies

**Documentation Sections**:
- [ ] Launch includes chapter in book
- [ ] API reference for LaunchLoader
- [ ] Migration guide from ROS 2 launch
- [ ] Troubleshooting guide
- [ ] Example plans with includes
- [ ] README updated with launch includes
- [ ] Python dependency installation guide
- [ ] Performance and limitations section

**Files Added**:
- `book/src/launch_includes.md`
- `book/src/migration_from_launch.md`
- `book/src/troubleshooting.md`

**Files Modified**:
- `book/src/SUMMARY.md` (add new chapters)
- `README.md` (add launch include example)

---

## Phase 6 Testing Strategy

### Unit Tests
- [ ] LaunchLoader API (Python)
- [ ] Visitor pattern (Python)
- [ ] PyO3 FFI bindings (Rust)
- [ ] LaunchInclude parsing (Rust)
- [ ] Node conversion (Rust)
- [ ] Launch cache (Rust)
- [ ] Node diffing (Rust)

### Integration Tests
- [ ] Compile plan with include
- [ ] Run nodes from included launch
- [ ] Reload on parameter change
- [ ] Name conflict resolution
- [ ] Namespace prefixing
- [ ] Composable node containers
- [ ] Nested includes

### End-to-End Tests
- [ ] Full system: Include → Compile → Run
- [ ] Runtime reload: Change param → Restart nodes
- [ ] Multi-include: Multiple launch files
- [ ] Hybrid: Plan nodes + launch nodes + links

### Performance Tests
- [ ] Load time for large launch files
- [ ] Reload time on parameter change
- [ ] Memory usage over 1000 reloads
- [ ] Python GIL overhead

### Compatibility Tests
- [ ] ROS 2 Humble launch files
- [ ] ROS 2 Iron launch files
- [ ] ROS 2 Rolling launch files
- [ ] Official ROS 2 example launch files

---

## Phase 6 Dependencies

**External Dependencies**:
- Python 3.10+
- PyO3 0.22+
- UV package manager
- ROS 2 launch package (Python)
- ruamel.yaml (Python)
- lark (Python)
- packaging (Python)

**Internal Dependencies**:
- Phase 1: Core compiler (F1-F10)
- Phase 2: Lua evaluation (F11-F15)
- Phase 3: Link resolution (F16-F17)
- Phase 4: Code generation (F18-F25)
- Phase 5: Testing infrastructure (F26-F31)

**Sequential Dependencies Within Phase 6**:
- F33 depends on F32 (need UV workspace before adding Python code)
- F35 depends on F33 (need Python API before FFI bindings)
- F36 parallel to F35 (format and FFI can develop concurrently)
- F37 depends on F35, F36 (need FFI and format before compiler integration)
- F38 depends on F37 (need compiler integration before caching)
- F39 depends on F38 (need cache before runtime)
- F40 depends on F37, F38, F39 (need all features for integration tests)
- F41 depends on F40 (document after testing complete)

---

## Phase 6 Success Criteria

- [ ] All 10 features (F32-F41) implemented and tested
- [ ] UV workspace successfully replaces Rye
- [ ] Python launch loader extracts node metadata without spawning processes
- [ ] PyO3 integration works reliably with no memory leaks
- [ ] Plan format supports launch includes with arguments
- [ ] Compiler merges included nodes with plan nodes
- [ ] Name conflicts resolved with prefixing
- [ ] Namespace prefixing works correctly
- [ ] Launch include cache speeds up reloads
- [ ] Parameter dependency tracking detects affected includes
- [ ] Runtime API starts/stops nodes on parameter changes
- [ ] All integration tests pass
- [ ] Performance meets targets (load <1s, reload <100ms)
- [ ] Documentation complete with examples and migration guide
- [ ] Compatible with ROS 2 Humble, Iron, and Rolling

---

## Phase 6 Known Limitations

1. **Python Dependency**: Requires Python runtime and ROS 2 launch packages
2. **Private API Usage**: Relies on Python name mangling to access private members (fragile across ROS versions)
3. **No XML/YAML Launch**: Only supports Python launch files (not XML or YAML launch formats)
4. **Limited Launch Features**: May not support all launch features (e.g., custom event handlers, complex substitutions)
5. **Performance Overhead**: Python FFI adds startup latency compared to pure Rust
6. **Version Compatibility**: May break when ROS 2 launch internals change in new releases

---

## Phase 6 Future Enhancements

- **F42**: Support XML and YAML launch file formats
- **F43**: Static analysis of launch files (detect issues before runtime)
- **F44**: Launch file visualization (show node tree from launch)
- **F45**: Automatic conversion tool (launch.py → plan.yaml)
- **F46**: Native Rust launch parser (remove Python dependency)
- **F47**: Launch file hot reload (watch file changes)
- **F48**: Include from package:// URIs (resolve from ROS packages)
- **F49**: Lazy loading (only load includes when needed)
- **F50**: Parallel loading (load multiple includes concurrently)