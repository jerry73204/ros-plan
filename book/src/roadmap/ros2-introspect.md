# ros2-introspect (Python)

**Location:** `python/ros2-introspect/`

**Purpose:** Discover ROS 2 node interfaces (publishers, subscriptions, services, clients) without middleware

**Status:** ✅ Production Ready
- **Tests:** 9 passing
- **Features:** All core features complete
- **Coverage:** Node introspection, interface discovery, dependency checking

---

## Features

### Introspection Capabilities

- Discover publishers and subscriptions with message types
- Discover services and clients with service types
- No middleware required (uses rmw_introspect_cpp RMW implementation)
- Dependency checking with clear error messages

### Key Features

- **Dependency Validation:** `check_rmw_introspect_available()` validates environment before introspection
- **Error Guidance:** Clear error messages guide users to fix missing dependencies
- **Fast Execution:** No node spawning, direct interface discovery
- **Test Coverage:** 9 tests covering introspection scenarios

### Dependencies

- Requires workspace to be built and sourced
- Requires rmw_introspect_cpp RMW implementation
