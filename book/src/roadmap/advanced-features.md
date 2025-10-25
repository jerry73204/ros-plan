# Advanced Runtime Features (Future Work)

**Status:** 🔴 Not Started (Future Work)

**Goal:** Advanced runtime features and enhancements

---

## F66: Terminal UI (TUI)

**Description:** Interactive terminal UI for runtime control and monitoring using `ratatui`

**Features:**
- Real-time node status display
- Interactive parameter updates
- Log viewing
- Node selection and control

---

## F67: Runtime Metrics and Monitoring

**Description:** Detailed metrics collection and monitoring

**Features:**
- CPU/memory usage per node
- Topic throughput monitoring
- Node restart rate tracking
- Performance dashboards

---

## F68: Node Dependency Ordering

**Description:** Start and stop nodes in correct dependency order

**Features:**
- Infer dependencies from topic connections
- Topological sort for startup order
- Graceful shutdown in reverse order

---

##F69: Health Checks and Recovery

**Description:** Advanced health checking and automatic recovery

**Features:**
- Custom health check scripts
- Detect hanging nodes (not just crashes)
- Automatic recovery strategies
- Alerting and notifications

---

## F70: XML/YAML Launch File Support

**Description:** Support loading XML and YAML launch files in addition to Python

**Features:**
- Parse XML launch files
- Parse YAML launch files
- Unified LaunchLoadResult format
- Extend launch2dump for all formats
