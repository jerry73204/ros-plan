# Implementation Roadmap

This page tracks the implementation status of the node/link connection design features. Each feature includes work items, expected results, test cases, and current implementation status.

## Overview

**Legend:**
- ❌ Not Started
- 🚧 In Progress
- ✅ Complete

**Progress Summary:**
- Format Extensions: 0/5 complete
- Topic Resolution: 0/5 complete
- Plan Encapsulation: 0/5 complete
- Validation & Errors: 0/4 complete
- Format Parsing Tests: 4/4 complete (F22 ✅, F23 ✅, F24 ✅, F25 ✅)
- Compiler Algorithm Tests: 2/4 complete (F26 ✅, F27 ✅)
- Integration Tests: 0/2 complete
- Example Tests: 0/2 complete

**Total:** 6/31 features complete (19%)

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
- 148 total tests (up from 2 baseline)
- Format tests: plan.rs (6), node.rs (7), link.rs (11), key.rs (2 existing)
- Expression tests: expr_.rs (15), value_or_expr.rs (13), text_or_expr.rs (11), bool_expr.rs (9), key_or_expr.rs (12)
- Type tests: value_type.rs (8), value.rs (19)
- Error tests: error.rs (17)
- Compiler tests: program.rs (5), lua.rs (13)
- All tests passing via `make test`

---

### Phase 2: Core Topic Resolution (2-3 weeks)

**Goal:** Implement basic single-source topic derivation with comprehensive tests.

**Features:** F1, F2, F6, F9, F28, F29

**Deliverables:**
- Node socket `ros_name` attribute
- Link `topic` attribute (basic)
- Single-source topic derivation algorithm
- Unit tests for topic resolution
- Integration tests for basic scenarios

**Dependencies:** Phase 1 (testing infrastructure)

**Timeline:** 2-3 weeks

---

### Phase 3: Multi-Source & Validation (2 weeks)

**Goal:** Support multiple publishers and add comprehensive validation.

**Features:** F3, F7, F10, F15, F30

**Deliverables:**
- Plan socket `topic` attribute
- Multi-source validation and errors
- Plan socket topic resolution
- Integration test suite with fixtures

**Dependencies:** Phase 2 (core resolution)

**Timeline:** 2 weeks

---

### Phase 4: Encapsulation & Transparency (2-3 weeks)

**Goal:** Implement plan boundaries, socket visibility, and transparent includes.

**Features:** F4, F11, F12, F13, F14, F16, F31

**Deliverables:**
- Include `transparent` flag
- Socket reference validation
- Transparent resolution algorithm
- Namespace hierarchy tracking
- Error scenario integration tests

**Dependencies:** Phase 3 (validation)

**Timeline:** 2-3 weeks

---

### Phase 5: Advanced Features & Polish (1-2 weeks)

**Goal:** Add optional enhancements and comprehensive examples.

**Features:** F5, F8, F17, F18, F19, F20, F21

**Deliverables:**
- Empty src/dst support
- Absolute/relative path resolution
- Type compatibility checking
- QoS validation
- Real-world example suite

**Dependencies:** Phase 4 (encapsulation)

**Timeline:** 1-2 weeks

---

## Feature Catalog

### Format Extensions

These features require changes to the YAML schema and data structures in `ros-plan-format`.

#### F1: Node Socket `ros_name` Attribute ❌

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Allow node sockets to override their name for ROS topic derivation.

**Work Items:**
1. Add `ros_name: Option<String>` field to `NodePubCfg`, `NodeSubCfg`, `NodeSrvCfg`, `NodeCliCfg`
2. Update YAML deserialization to parse `ros_name`
3. Document field in format specification
4. Write parsing tests

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
1. Parse YAML with `ros_name` field
2. Verify topic derivation uses `ros_name` when present
3. Verify topic derivation uses socket name when `ros_name` absent
4. Test with all socket types (pub, sub, srv, cli)

**Current State:**
- Fields: ❌ Not present in `NodePubCfg` etc.
- Tests: ❌ None

**Files Affected:**
- `ros-plan-format/src/node_socket.rs`

---

#### F2: Link `topic` Attribute ❌

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Allow links to specify explicit ROS topic names.

**Work Items:**
1. Add `topic: Option<TopicName>` field to `PubSubLinkCfg`
2. Support both absolute (`/tf`) and relative (`diagnostics`) paths
3. Update YAML deserialization
4. Document field in format specification
5. Write parsing tests

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
1. Parse link with absolute topic path
2. Parse link with relative topic path
3. Parse link without topic (existing behavior)
4. Verify topic resolution with explicit topic
5. Test empty src/dst with explicit topic

**Current State:**
- Field: ❌ Not present in `PubSubLinkCfg`
- Tests: ❌ None

**Files Affected:**
- `ros-plan-format/src/link.rs`

---

#### F3: Plan Socket `topic` Attribute ❌

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Allow plan sockets to specify topic names when aggregating multiple sources.

**Work Items:**
1. Add `topic: Option<TopicName>` field to `PlanPubCfg`
2. Allow `src: Vec<KeyOrExpr>` to have multiple entries
3. Update YAML deserialization
4. Document multi-source aggregation pattern
5. Write parsing tests

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
- Field: ❌ Not present in `PlanPubCfg`
- Multi-src: ✅ Already `Vec<KeyOrExpr>`
- Tests: ❌ None

**Files Affected:**
- `ros-plan-format/src/plan_socket.rs`

---

#### F4: Include `transparent` Flag ❌

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

#### F6: Single-Source Topic Derivation ❌

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Derive ROS topic name from single source socket's namespace and ros_name.

**Work Items:**
1. Implement topic resolution for links with one source
2. Handle node sockets vs plan sockets
3. Use `socket.ros_name ?? socket.name` for topic component
4. Build full path: `/{namespace}/{topic_component}`
5. Write comprehensive unit tests

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
1. Derive topic from node socket name
2. Derive topic with ros_name override
3. Derive topic from plan socket
4. Derive topic with plan socket ros_name
5. Handle multi-level namespaces correctly

**Current State:**
- Algorithm: ❌ Not implemented
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs` (new or existing)
- `ros-plan-compiler/src/context/link.rs`

---

#### F7: Multi-Source Explicit Topic Requirement ❌

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Require explicit topic when link has multiple sources, provide clear error otherwise.

**Work Items:**
1. Implement validation: `len(src) > 1 && topic.is_none()` → Error
2. Create descriptive error message
3. Suggest adding `topic` field in error
4. Test error reporting
5. Write error scenario tests

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
- Validation: ❌ Not implemented
- Error type: ❌ Not defined
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs`
- `ros-plan-compiler/src/error.rs`

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

#### F9: Socket `ros_name` Override Resolution ❌

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Use socket's `ros_name` field when deriving topic names.

**Work Items:**
1. Query socket for `ros_name` field
2. Fall back to socket name if not present
3. Integrate with topic derivation algorithm
4. Test with node and plan sockets
5. Write unit tests for override logic

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
1. Derive topic with ros_name present
2. Derive topic with ros_name absent
3. Test plan socket ros_name
4. Test node socket ros_name
5. Verify precedence: explicit topic > ros_name > socket name

**Current State:**
- Query logic: ❌ Not implemented
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs`
- `ros-plan-compiler/src/context/node_socket.rs`
- `ros-plan-compiler/src/context/plan_socket.rs`

---

#### F10: Plan Socket Topic Resolution ❌

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Resolve topics for plan sockets with explicit topic field and multiple sources.

**Work Items:**
1. Handle plan socket `topic` field
2. Create topic at plan boundary
3. Map multiple internal sources to single topic
4. Integrate with link resolution
5. Write plan socket resolution tests

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
- Algorithm: ❌ Not implemented
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/socket_resolver.rs`
- `ros-plan-compiler/src/processor/link_resolver.rs`

---

### Plan Encapsulation

These features enforce visibility boundaries between plans.

#### F11: Socket Reference Validation ❌

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

#### F12: Transparent Include Resolution ❌

**Phase:** 4 (Encapsulation & Transparency)

**Priority:** Medium

**Description:** Allow deep socket references when include is marked `transparent: true`.

**Work Items:**
1. Track transparency flag in include context
2. Implement recursive socket resolution
3. Stop at opaque boundaries
4. Validate transitive visibility
5. Write transparency tests

**Expected Results:**
```yaml
include:
  sensors: !file
    path: sensors.yaml
    transparent: true

link:
  deep: !pubsub
    src: [sensors/camera/driver/output]  # OK: sensors is transparent
    dst: [processor/input]
```

**Test Cases:**
1. Deep reference through transparent include → Success
2. Deep reference through opaque include → Error
3. Multi-level transparency (transitive) → Success
4. Mixed transparent/opaque boundaries
5. Verify error when trying to access through opaque plan

**Current State:**
- Transparency tracking: ❌ Not implemented
- Resolution: ❌ Not implemented
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/context/include.rs`
- `ros-plan-compiler/src/processor/link_resolver.rs`

---

#### F13: Plan Socket Forwarding ❌

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

#### F14: Namespace Hierarchy Tracking ❌

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

#### F15: Multi-Source Without Topic Error ❌

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** Clear error when link has multiple sources but no explicit topic.

**Work Items:**
1. Define error type: `MultipleSoursRequireExplicitTopic`
2. Include link name, source count, and suggestion in error
3. Implement error in link resolution
4. Test error message quality
5. Write error scenario tests

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
- Error type: ❌ Not defined
- Implementation: ❌ None
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/error.rs`
- `ros-plan-compiler/src/processor/link_resolver.rs`

---

#### F16: Invalid Socket Reference Errors ❌

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

#### F28: Socket Resolution Tests ❌

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Unit tests for socket resolution algorithm.

**Work Items:**
1. Test socket reference parsing
2. Test local node socket resolution
3. Test plan socket resolution
4. Test socket forwarding
5. Test error cases (socket not found, invalid reference)
6. Mock program structure for testing

**Expected Results:**
- Test module in `ros-plan-compiler/src/processor/socket_resolver.rs`
- Comprehensive coverage of resolution logic

**Test Cases:**
```rust
#[test]
fn resolve_node_socket() { ... }

#[test]
fn resolve_plan_socket() { ... }

#[test]
fn resolve_forwarded_socket() { ... }

#[test]
fn error_on_socket_not_found() { ... }

#[test]
fn error_on_invalid_reference_format() { ... }

#[test]
fn resolve_in_nested_scope() { ... }
```

**Current State:**
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/socket_resolver.rs` (add test module)

---

#### F29: Link Resolution Tests ❌

**Phase:** 2 (Core Topic Resolution)

**Priority:** High

**Description:** Unit tests for link resolution and topic derivation.

**Work Items:**
1. Test link source resolution
2. Test link destination resolution
3. Test topic name derivation
4. Test multiple sources handling
5. Test type checking
6. Test error cases
7. Mock socket and program structures

**Expected Results:**
- Test module in `ros-plan-compiler/src/processor/link_resolver.rs`
- Tests for all derivation paths

**Test Cases:**
```rust
#[test]
fn resolve_link_sources() { ... }

#[test]
fn resolve_link_destinations() { ... }

#[test]
fn derive_topic_from_single_source() { ... }

#[test]
fn require_explicit_topic_for_multi_source() { ... }

#[test]
fn use_ros_name_override() { ... }

#[test]
fn error_on_invalid_socket_reference() { ... }

#[test]
fn error_on_type_mismatch() { ... }
```

**Current State:**
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs` (add test module)

---

### Integration Tests

These tests verify end-to-end compilation workflows.

#### F30: Fixture-Based Integration Tests ❌

**Phase:** 3 (Multi-Source & Validation)

**Priority:** High

**Description:** End-to-end compilation tests using YAML fixtures.

**Work Items:**
1. Create test fixtures directory structure
2. Create valid plan examples for each feature
3. Test successful compilation for each fixture
4. Verify generated output structure
5. Compare expected vs actual output
6. Set up CI to run integration tests

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
- Fixtures: ❌ None
- Integration tests: ❌ None

**Files Affected:**
- `tests/integration_tests.rs` (new)
- `tests/fixtures/*.yaml` (new)

---

#### F31: Error Scenario Integration Tests ❌

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

#### F19: Unit Tests for Topic Resolution ❌

**Phase:** 5 (Advanced Features)

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
- Tests: ❌ None

**Files Affected:**
- `ros-plan-compiler/src/processor/link_resolver.rs` (test module)

---

#### F20: Encapsulation Integration Tests ❌

**Phase:** 5 (Advanced Features)

**Priority:** High

**Description:** End-to-end tests for encapsulation scenarios (refactored from original F20).

**Work Items:**
1. Create test plans with opaque boundaries
2. Create test plans with transparent includes
3. Test socket forwarding patterns
4. Test multi-level nesting
5. Verify error cases

**Expected Results:**
- Test plans in `tests/fixtures/encapsulation/`
- Comprehensive coverage of visibility rules

**Test Cases:**
```rust
#[test]
fn compile_opaque_plan_with_exposed_sockets() { ... }

#[test]
fn compile_transparent_plan_with_deep_references() { ... }

#[test]
fn error_on_reference_through_opaque_boundary() { ... }

#[test]
fn multi_source_plan_socket_aggregates_topics() { ... }
```

**Current State:**
- Test fixtures: ❌ None
- Integration tests: ❌ None

**Files Affected:**
- `tests/encapsulation_tests.rs` (new)
- `tests/fixtures/encapsulation/*.yaml` (new)

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
