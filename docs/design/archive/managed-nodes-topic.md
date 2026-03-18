# Managed Nodes Topic Design

Design for exposing play_launch's managed node state via a ROS 2 topic, with a
custom message package and a CLI tool for users.

## Problem

When play_launch starts a large deployment (e.g., Autoware with ~61 processes),
`ros2 node list` returns stale or incomplete results because the ros2 daemon's
DDS discovery cannot keep up with the graph churn.

play_launch already knows the authoritative state of every node it manages
(name, namespace, PID, loaded status) with zero DDS discovery lag -- but this
information is only available via its HTTP web API, which has port conflict issues
when running multiple instances.

## Goal

Expose play_launch's managed node state over ROS 2 RMW (DDS) so that:

1. Multiple play_launch instances coexist without port conflicts
2. Consumers discover publishers through standard DDS mechanisms
3. The existing ListNodes verification flow is unaffected
4. Rich monitoring data is available (not just node names)
5. A dedicated CLI tool gives users immediate, authoritative node listings

## Why DDS Discovery Works Here

The daemon's problem is discovering **61 nodes joining simultaneously**. The
play_launch management node is fundamentally different:

- Created **before** spawning any application nodes
- A **single, stable** DDS participant -- discovered quickly
- Stays up for the entire session -- no churn

Timeline:

```
t=0s     play_launch starts, creates management node + advertises topic
t=0.5s   DDS discovers play_launch's topic (1 participant, fast)
t=1s     play_launch begins spawning 61 application nodes
t=1-10s  DDS graph churns (daemon struggles here)
         BUT: play_launch's topic is already discovered
         Subscriber receives authoritative state instantly
```

## Compatibility with ListNodes Verification

The managed-nodes topic and the ListNodes verification operate at different
layers and do not conflict:

```
                        play_launch's rclrs node
                       ┌──────────────────────────┐
OUTBOUND (new)         │                          │  OUTBOUND (existing)
publish topic          │   Actor State Machine     │  LoadNode service call
/play_launch/managed   │                          │  → container process
◄────────────────────  │  Pending                 │  ────────────────────►
                       │  Running {pid}            │
                       │  Loading {started_at}     │  INBOUND (existing)
                       │  Loaded {unique_id}       │  ListNodes service call
                       │  ...                      │  → container process
                       └──────────────────────────┘  ────────────────────►
```

- **ListNodes**: play_launch is a **client** calling the container process's
  `/_container/list_nodes` service. Internal verification: "did the container
  actually load this node?"
- **Managed-nodes topic**: play_launch is a **publisher** announcing its state
  machine's current view. External announcement: "here's what I manage and their
  states."

The topic publishes whatever the actor state machine says. ListNodes is one of
the inputs that updates that state machine. They are sequential:

```
LoadNode call sent         → state = Loading  → topic publishes Loading
LoadNode response received → state = Loaded   → topic publishes Loaded
  OR
LoadNode times out (30s)   → ListNodes query
ListNodes finds the node   → state = Loaded   → topic publishes Loaded
ListNodes doesn't find it  → state = Loading  → topic still shows Loading
```

No changes to the ListNodes verification flow are needed.

## Coexistence with ros2 Daemon

The daemon and play_launch's topic serve different scopes:

| Source | Scope | Accuracy | Latency |
|--------|-------|----------|---------|
| play_launch topic | Only nodes it manages | Authoritative | Zero |
| ros2 daemon | ALL nodes on DDS graph | Best-effort | Seconds under load |
| Both merged | Complete picture | Authoritative where available | Hybrid |

They coexist without modification to either:

- `ros2 node list` continues to query the daemon (all nodes, eventually
  consistent)
- `play_launch list` subscribes to the managed-nodes topic (managed nodes,
  authoritative, instant)
- A future merge mode in `play_launch list --all` could combine both sources

---

## Architecture

```
┌─────────────────────┐         ┌─────────────────────┐
│ play_launch A       │         │ play_launch B       │
│  Actor state machine│         │  Actor state machine│
│         │           │         │         │           │
│         ▼           │         │         ▼           │
│  ManagedNodes topic │         │  ManagedNodes topic │
└────────┬────────────┘         └────────┬────────────┘
         │ publish                       │ publish
         ▼                               ▼
    /play_launch/managed_nodes  (DDS multicast)
         │
         │ subscribe (transient_local)
         ▼
┌─────────────────────┐
│ play_launch list    │
│  (CLI tool)         │
│  Aggregates by      │
│  instance_id        │
└─────────────────────┘
```

Multiple play_launch instances publish to the **same topic**. Each message
carries an `instance_id` so the subscriber can aggregate and deduplicate. DDS
multicast handles publisher discovery -- no manual port configuration needed.

---

## Message Package: `play_launch_msgs`

A standard ROS 2 CMake message package, placed at `src/play_launch_msgs/`.
colcon-cargo-ros2 generates Rust bindings automatically (same as
`composition_interfaces`, `diagnostic_msgs`, etc.).

### Package Structure

```
src/play_launch_msgs/
├── package.xml
├── CMakeLists.txt
└── msg/
    ├── ManagedNodes.msg
    ├── MemberStatus.msg
    └── HealthSummary.msg
```

### `msg/MemberStatus.msg`

```
# Member type
uint8 NODE=0
uint8 CONTAINER=1
uint8 COMPOSABLE_NODE=2

# State (process-based members: Node, Container)
uint8 PENDING=0
uint8 RUNNING=1
uint8 RESPAWNING=2
uint8 STOPPED=3
uint8 FAILED=4

# State (composable node members)
uint8 UNLOADED=10
uint8 LOADING=11
uint8 LOADED=12
uint8 UNLOADING=13
uint8 BLOCKED=14

# Block reason (valid when state=BLOCKED)
uint8 BLOCK_CONTAINER_NOT_STARTED=0
uint8 BLOCK_CONTAINER_STOPPED=1
uint8 BLOCK_CONTAINER_FAILED=2
uint8 BLOCK_SHUTDOWN=3

# --- Identity ---
string name                     # Actor name (e.g., "NODE 'talker'")
uint8 member_type
string package                  # ROS package (e.g., "demo_nodes_cpp")
string executable               # Binary name (e.g., "talker")
string namespace                # ROS namespace (e.g., "/perception")
string node_name                # ROS node name (e.g., "talker")
string exec_name                # Executable display name

# --- State ---
uint8 state
uint32 pid                      # Valid when state=RUNNING; 0 otherwise
uint64 unique_id                # Valid when state=LOADED; 0 otherwise
uint32 respawn_attempt          # Valid when state=RESPAWNING; 0 otherwise
string error_message            # Valid when state=FAILED; empty otherwise
uint8 block_reason              # Valid when state=BLOCKED; 0 otherwise

# --- Composable node fields ---
string target_container         # Parent container name; empty if not composable
bool auto_load                  # Auto-load when container starts

# --- Configuration ---
bool respawn_enabled
float64 respawn_delay           # Seconds; 0.0 if not set

# --- Monitoring ---
uint64 stderr_size              # Bytes of stderr output
uint64 stderr_last_modified     # Unix timestamp (seconds); 0 if never written
```

### `msg/HealthSummary.msg`

```
# Aggregate counts across all managed members

# Process-level
uint32 processes_running
uint32 processes_stopped

# Regular nodes
uint32 nodes_running
uint32 nodes_stopped
uint32 nodes_failed
uint32 nodes_total

# Containers
uint32 containers_running
uint32 containers_stopped
uint32 containers_failed
uint32 containers_total

# Composable nodes
uint32 composable_loaded
uint32 composable_failed
uint32 composable_pending
uint32 composable_total

# Noise indicator
uint32 noisy                    # Members with stderr > 10KB
```

### `msg/ManagedNodes.msg`

```
# Snapshot of all members managed by a play_launch instance.
# Published on /play_launch/managed_nodes with transient_local QoS.

string instance_id              # Unique per play_launch process (UUID)
builtin_interfaces/Time stamp
MemberStatus[] members
HealthSummary health
```

### `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>play_launch_msgs</name>
  <version>0.1.0</version>
  <description>Message definitions for play_launch managed node status</description>
  <maintainer email="jerry73204@gmail.com">Jerry Lin</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(play_launch_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManagedNodes.msg"
  "msg/MemberStatus.msg"
  "msg/HealthSummary.msg"
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### Build Integration

`play_launch` declares the dependency on `play_launch_msgs`:

**`src/play_launch/package.xml`** -- add:

```xml
<depend>play_launch_msgs</depend>
```

**`src/play_launch/Cargo.toml`** -- add:

```toml
play_launch_msgs = "*"
```

colcon resolves the build order: `play_launch_msgs` (CMake, generates .msg →
C/Python/Rust bindings) builds before `play_launch` (ament_cargo, consumes Rust
bindings).

---

## Publisher Side (play_launch)

### Files to Modify

| File | Change |
|------|--------|
| `src/play_launch/package.xml` | Add `<depend>play_launch_msgs</depend>` |
| `src/play_launch/Cargo.toml` | Add `play_launch_msgs = "*"` |
| `src/play_launch/src/commands/replay.rs` | Spawn publisher task alongside other background tasks |
| `src/play_launch/src/member_actor/coordinator.rs` | Expose state snapshot method for publisher |

### New File

| File | Purpose |
|------|---------|
| `src/play_launch/src/ros/managed_nodes_publisher.rs` | Async task: listens for state events, publishes `ManagedNodes` |

### Publisher Task Lifecycle

```
1. Generate instance_id (UUID)
2. Create publisher:
   - Topic: /play_launch/managed_nodes
   - Type:  play_launch_msgs::msg::ManagedNodes
   - QoS:   transient_local, reliable, keep_last(1)
3. Publish initial snapshot (all members in current state)
4. Loop:
   a. Wait for StateEvent from broadcast channel
   b. Debounce: collect events for 200ms window
   c. Read current state via MemberHandle::list_members()
   d. Convert Vec<MemberSummary> → Vec<MemberStatus>
   e. Compute HealthSummary
   f. Publish ManagedNodes message
5. On shutdown: publisher dropped, DDS handles cleanup
```

### QoS Profile

```rust
let qos = rclrs::QoSProfile {
    durability: rclrs::QoSDurabilityPolicy::TransientLocal,
    reliability: rclrs::QoSReliabilityPolicy::Reliable,
    history: rclrs::QoSHistoryPolicy::KeepLast,
    depth: 1,
    ..Default::default()
};
```

- **Transient local**: Late subscribers receive the most recent snapshot
  immediately upon discovery. No need to wait for the next state change.
- **Reliable**: Guarantees delivery (node state is important, not high-frequency
  sensor data).
- **Depth 1**: Only the latest snapshot matters.

### Debouncing

During rapid state changes (e.g., 15 containers starting simultaneously),
debounce publications to at most one every 200ms. On receiving a `StateEvent`,
start a 200ms timer. If more events arrive within the window, reset the timer.
Publish a single snapshot when the timer fires.

This bounds topic bandwidth during startup bursts while preserving near-instant
updates during steady state.

### Conversion: `MemberSummary` → `MemberStatus`

The publisher converts between Rust internal types and generated message types:

```rust
fn to_member_status(m: &MemberSummary) -> play_launch_msgs::msg::MemberStatus {
    let mut msg = play_launch_msgs::msg::MemberStatus::default();
    msg.name = m.name.clone();
    msg.member_type = match m.member_type {
        MemberType::Node => MemberStatus::NODE,
        MemberType::Container => MemberStatus::CONTAINER,
        MemberType::ComposableNode => MemberStatus::COMPOSABLE_NODE,
    };
    msg.package = m.package.clone().unwrap_or_default();
    msg.executable = m.executable.clone();
    msg.namespace = m.namespace.clone().unwrap_or_default();
    msg.node_name = m.node_name.clone().unwrap_or_default();
    msg.exec_name = m.exec_name.clone().unwrap_or_default();

    match &m.state {
        MemberState::Pending => msg.state = MemberStatus::PENDING,
        MemberState::Running { pid } => {
            msg.state = MemberStatus::RUNNING;
            msg.pid = *pid;
        }
        MemberState::Respawning { attempt } => {
            msg.state = MemberStatus::RESPAWNING;
            msg.respawn_attempt = *attempt;
        }
        MemberState::Stopped => msg.state = MemberStatus::STOPPED,
        MemberState::Failed { error } => {
            msg.state = MemberStatus::FAILED;
            msg.error_message = error.clone();
        }
        MemberState::Unloaded => msg.state = MemberStatus::UNLOADED,
        MemberState::Loading => msg.state = MemberStatus::LOADING,
        MemberState::Loaded { unique_id } => {
            msg.state = MemberStatus::LOADED;
            msg.unique_id = *unique_id;
        }
        MemberState::Unloading => msg.state = MemberStatus::UNLOADING,
        MemberState::Blocked { reason } => {
            msg.state = MemberStatus::BLOCKED;
            msg.block_reason = match reason {
                BlockReason::ContainerNotStarted => MemberStatus::BLOCK_CONTAINER_NOT_STARTED,
                BlockReason::ContainerStopped => MemberStatus::BLOCK_CONTAINER_STOPPED,
                BlockReason::ContainerFailed => MemberStatus::BLOCK_CONTAINER_FAILED,
                BlockReason::Shutdown => MemberStatus::BLOCK_SHUTDOWN,
            };
        }
    }

    msg.target_container = m.target_container.clone().unwrap_or_default();
    msg.auto_load = m.auto_load.unwrap_or(false);
    msg.respawn_enabled = m.respawn_enabled.unwrap_or(false);
    msg.respawn_delay = m.respawn_delay.unwrap_or(0.0);
    msg.stderr_size = m.stderr_size;
    msg.stderr_last_modified = m.stderr_last_modified.unwrap_or(0);

    msg
}
```

---

## CLI Tool: `play_launch list`

A new subcommand on the existing `play_launch` binary. Subscribes to the
managed-nodes topic, aggregates across all play_launch instances, and prints
a formatted table.

### Usage

```
$ play_launch list
Instance abc123 (61 members: 45 running, 15 loaded, 1 failed):
  /perception/filter                      Node           Running   PID 12345
  /perception/container                   Container      Running   PID 12400
  /perception/traffic_light_classifier    ComposableNode Loaded    uid 42
  /planning/route_planner                 Node           Failed    exit code 1
  ...

Instance def456 (8 members: 8 running):
  /sensing/lidar_driver                   Node           Running   PID 23456
  ...
```

### Options

```
play_launch list [OPTIONS]

Options:
  --timeout <SECS>       Discovery timeout in seconds [default: 2]
  --state <STATE>        Filter by state: running, stopped, failed, loaded, ...
  --type <TYPE>          Filter by type: node, container, composable
  --instance <ID>        Show only a specific instance
  --watch                Continuous mode: print updates as they arrive
  --no-header            Omit instance headers (for scripting)
  --count                Print member counts only
```

### Implementation

**New files:**

| File | Purpose |
|------|---------|
| `src/play_launch/src/commands/list.rs` | `handle_list()` implementation |
| `src/play_launch/src/cli/options.rs` | Add `List(ListArgs)` variant to `Command` enum |

**Subscriber flow:**

```
1. Create rclrs Context + Node ("play_launch_list")
2. Subscribe to /play_launch/managed_nodes
   - QoS: transient_local, reliable, keep_last(10)
   - depth=10 to buffer messages from multiple publishers
3. Spin for --timeout seconds (default 2s)
   - On each message: store in HashMap<instance_id, ManagedNodes>
   - Transient local delivers latest from each publisher on discovery
4. Format and print aggregated results
5. Exit (or continue in --watch mode)
```

The 2-second default timeout accommodates DDS discovery of the play_launch
publisher nodes. Since these are stable, long-lived participants, discovery
typically completes within 0.5s. The extra headroom handles slower networks or
high system load.

**Watch mode** (`--watch`):

Instead of exiting after the timeout, keep spinning. On each new message, clear
the terminal and reprint the full table. This provides a live dashboard in the
terminal.

### State Display Format

| State | Display | Extra Info |
|-------|---------|------------|
| `PENDING` | `Pending` | |
| `RUNNING` | `Running` | `PID {pid}` |
| `RESPAWNING` | `Respawning` | `attempt {n}` |
| `STOPPED` | `Stopped` | |
| `FAILED` | `Failed` | `{error_message}` |
| `UNLOADED` | `Unloaded` | |
| `LOADING` | `Loading` | |
| `LOADED` | `Loaded` | `uid {unique_id}` |
| `UNLOADING` | `Unloading` | |
| `BLOCKED` | `Blocked` | `{block_reason}` |

---

## Alternatives Considered

### `std_msgs/String` with JSON Payload

Publish JSON-serialized state as a plain string. Zero custom message definitions,
schema evolves freely, but no compile-time type safety for consumers, and
`ros2 topic echo` shows a raw JSON blob instead of structured fields. JSON also
cannot be recorded/replayed cleanly with standard ROS 2 bag tooling
(plotjuggler, rqt_bag).

### HTTP Web API (Current)

play_launch already serves `/api/nodes` on its web UI port. Works for a single
instance but causes port conflicts with multiple instances. Not discoverable
via ROS 2 mechanisms.

---

## Summary of Changes

### New Package

| Path | Type | Purpose |
|------|------|---------|
| `src/play_launch_msgs/` | CMake package | `.msg` definitions + rosidl generation |

### Modified Files (play_launch)

| File | Change |
|------|--------|
| `src/play_launch/package.xml` | Add `<depend>play_launch_msgs</depend>` |
| `src/play_launch/Cargo.toml` | Add `play_launch_msgs = "*"` |
| `src/play_launch/src/main.rs` | Add `Command::List` match arm |
| `src/play_launch/src/cli/options.rs` | Add `List(ListArgs)` to `Command` enum |
| `src/play_launch/src/commands/mod.rs` | Add `handle_list()` |
| `src/play_launch/src/commands/replay.rs` | Spawn publisher task |
| `src/play_launch/src/member_actor/coordinator.rs` | Expose snapshot for publisher |

### New Files (play_launch)

| File | Purpose |
|------|---------|
| `src/play_launch/src/ros/managed_nodes_publisher.rs` | Publisher task (state events → topic) |
| `src/play_launch/src/commands/list.rs` | `play_launch list` subscriber + formatter |
