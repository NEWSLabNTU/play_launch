# RCL Interception Design

## Overview

`libplay_launch_interception.so` is an LD_PRELOAD library that hooks RCL
functions to observe ROS 2 communication without modifying user code. It is
injected by play_launch into all managed node processes.

The library uses a compiled-in plugin architecture. Each plugin receives
callbacks at publisher/subscriber init and on every publish/take. Plugins
write events to a zero-copy SPSC shared memory ring buffer consumed by
play_launch.

Roadmap: `docs/roadmap/phase-29-rcl_interception.md`

## Architecture

```
  Node process (LD_PRELOAD)
  ┌──────────────────────────────────────────────────┐
  │                                                  │
  │  rcl_publisher_init / rcl_publish                │
  │       │                                          │
  │       v                                          │
  │  libplay_launch_interception.so                  │
  │  ┌──────────────────────────────────────────┐    │
  │  │ Runtime (OnceLock, lazy init)             │    │
  │  │   originals: 4 fn pointers (dlsym)       │    │
  │  │   plugins: Vec<Box<dyn InterceptionPlugin>>│   │
  │  │     ├── FrontierPlugin (stamp tracking)  │    │
  │  │     ├── StatsPlugin (counts, rates)      │    │
  │  │     └── (future plugins)                 │    │
  │  └──────────────────┬───────────────────────┘    │
  │                     │ SPSC ring buffer            │
  │                     │ (memfd shared memory)       │
  └─────────────────────┼────────────────────────────┘
                        │ eventfd wakeup
                        v
  play_launch process
  ┌──────────────────────────────────────────────────┐
  │ InterceptionListener (tokio task)                │
  │   polls eventfd via AsyncFd                      │
  │   reads InterceptionEvent from Consumer          │
  │   dispatches by EventKind                        │
  └──────────────────────────────────────────────────┘
```

## Hooked Functions

Four RCL functions are intercepted. Init hooks are cold path (once per
endpoint). Publish/take hooks are hot path (every message).

| Hook                    | Path | Action                                                                 |
|-------------------------|------|------------------------------------------------------------------------|
| `rcl_publisher_init`    | Cold | Call original → register in registry → dispatch `on_publisher_init`    |
| `rcl_publish`           | Hot  | Lookup registry → read stamp → dispatch `on_publish` → call original   |
| `rcl_subscription_init` | Cold | Call original → register in registry → dispatch `on_subscription_init` |
| `rcl_take`              | Hot  | Call original → if success, lookup → read stamp → dispatch `on_take`   |

Original function pointers resolved lazily via `OnceLock<Option<Runtime>>`.
Two resolution strategies for Python support:
1. `dlsym(RTLD_NEXT)` — C++ nodes (global symbol scope)
2. `dlopen("librcl.so", RTLD_NOLOAD)` + `dlsym(handle)` — Python nodes
   (rclpy loads librcl with RTLD_LOCAL)

## Plugin Architecture

All plugins are compiled into the `.so`. No dynamic loading, no C ABI.

```rust
pub(crate) trait InterceptionPlugin: Send + Sync {
    fn on_publisher_init(&self, handle: usize, topic: &str, topic_hash: u64, stamp_offset: Option<usize>);
    fn on_subscription_init(&self, handle: usize, topic: &str, topic_hash: u64, stamp_offset: Option<usize>);
    fn on_publish(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);
    fn on_take(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);
}
```

Each plugin's constructor checks activation env vars and returns
`Option<Self>`. Inactive plugins are not instantiated.

### Current plugins

**FrontierPlugin** — per-topic timestamp frontier tracking. Writes
`InterceptionEvent` to SPSC ring when frontier advances. State:
`AtomicU64` per topic (CAS max-update, ~40-50ns per publish).

**StatsPlugin** — writes every publish/take event to SPSC ring with
monotonic timestamp. play_launch computes per-topic counts, rates,
inter-message latency, and pub-to-take ratio.

### Inert mode

The library is inert (hooks pass through with zero extra work) when:
- `librcl.so` not found (non-ROS process), or
- No plugins activated (no relevant env vars set)

## IPC: SPSC Shared Memory Ring Buffer

`spsc_shm` crate — generic, zero-dep (only `libc`), reusable.

```rust
#[repr(C)]
pub struct InterceptionEvent {  // 40 bytes
    pub kind: EventKind,        // u8
    pub _pad: [u8; 3],
    pub topic_hash: u64,
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub handle: u64,
    pub monotonic_ns: u64,
}
```

Setup: play_launch creates `memfd` + `eventfd` per child process, passes
fds via env vars (`PLAY_LAUNCH_INTERCEPTION_SHM_FD`,
`PLAY_LAUNCH_INTERCEPTION_EVENT_FD`). Producer (in child) and Consumer
(in play_launch) share the memory-mapped ring. Lock-free, single-writer
single-reader.

## Graph Discovery

### Current: ROS graph API (Phase 25)

`GraphSnapshot` is built by polling DDS discovery via `rclrs` node methods:

```rust
// graph_builder.rs
ros_node.get_topic_names_and_types()?;
ros_node.get_publishers_info_by_topic(topic)?;
ros_node.get_subscriptions_info_by_topic(topic)?;
```

These call `rcl_get_topic_names_and_types`,
`rcl_get_publishers_info_by_topic`, etc. under the hood.

**Characteristics:**
- Polling — periodic snapshots at a configurable interval
- Covers topics, publishers, subscribers, QoS profiles
- O(topics) DDS queries per snapshot
- Detection latency = polling interval
- Services and actions require separate API calls
  (`get_service_names_and_types`, etc.)

This is used by Phase 30 (Topic Manifest) v1 for runtime audit: the
executor diffs `GraphSnapshot` against the expected graph from `record.json`.

### Future: Interception-based graph discovery

The Phase 29 interception layer already hooks `rcl_publisher_init` and
`rcl_subscription_init` — the exact points where the graph changes. These
hooks receive the topic name, type support, and node handle. A new
`GraphPlugin` can report graph changes as events through the existing SPSC
pipeline.

```
Current (polling):
  play_launch ──[timer]──→ rcl graph API → snapshot → diff

Future (event-driven):
  rcl_publisher_init hook ──→ GraphPlugin.on_publisher_init()
                              ──→ SPSC event → play_launch
                              ──→ immediate diff against expected graph
```

**Advantages over polling:**
- Instant detection — deviations caught at endpoint creation time,
  before the first message
- No polling overhead — event-driven via existing SPSC ring
- Complete coverage — hooks fire for every endpoint, including dynamic
  ones created after startup
- Services and actions — hookable via `rcl_service_init`,
  `rcl_client_init`, `rcl_action_server_init`, `rcl_action_client_init`

**New hooks needed** (not yet intercepted):

| Hook | Purpose |
|------|---------|
| `rcl_service_init` | Detect service server creation |
| `rcl_client_init` | Detect service client creation |
| `rcl_action_server_init` | Detect action server creation |
| `rcl_action_client_init` | Detect action client creation |

These follow the same pattern as publisher/subscriber hooks: call original,
register in registry, dispatch to plugins.

**New plugin: `GraphPlugin`**

```rust
pub(crate) struct GraphPlugin { producer: Producer<InterceptionEvent> }

impl InterceptionPlugin for GraphPlugin {
    fn on_publisher_init(&self, _handle: usize, topic: &str,
                         topic_hash: u64, _stamp_offset: Option<usize>) {
        self.producer.push(InterceptionEvent {
            kind: EventKind::PublisherInit,
            topic_hash,
            ..
        });
    }
    fn on_subscription_init(&self, _handle: usize, topic: &str,
                            topic_hash: u64, _stamp_offset: Option<usize>) {
        self.producer.push(InterceptionEvent {
            kind: EventKind::SubscriptionInit,
            topic_hash,
            ..
        });
    }
    fn on_publish(&self, ..) {}   // no-op for graph plugin
    fn on_take(&self, ..) {}      // no-op for graph plugin
}
```

The play_launch consumer receives `PublisherInit`/`SubscriptionInit` events
and immediately diffs against the expected graph. No polling needed.

Note: `PublisherInit`/`SubscriptionInit` event kinds already exist in the
`EventKind` enum (used by StatsPlugin). The `GraphPlugin` reuses them.

## Graph Audit in the Interception Layer

Beyond event-driven detection, the interception layer can perform audit
**inside the hooked process** — enabling blocking enforcement.

### Warn-only (v2)

The `GraphPlugin` receives the expected graph (via shared memory or
serialized in an env var). On `rcl_publisher_init`, it checks whether the
topic is in the expected graph. If not, it writes a warning event to the
SPSC ring. play_launch logs the deviation. The publisher is still created.

### Blocking enforcement (v3)

The hook can return `RCL_RET_ERROR` from `rcl_publisher_init` to prevent
unauthorized publishers from being created. The node receives an error
from `rcl_publisher_init` and handles it (or crashes, depending on the
node's error handling).

```rust
// In the rcl_publisher_init hook (future)
fn rcl_publisher_init(pub_: *mut rcl_publisher_t, node: *const rcl_node_t,
                      type_support: *const .., topic: *const c_char,
                      opts: *const ..) -> rcl_ret_t {
    let ret = (originals.rcl_publisher_init)(pub_, node, type_support, topic, opts);
    if ret != RCL_RET_OK { return ret; }

    let topic_str = /* extract topic name */;
    if let Some(graph) = expected_graph() {
        if !graph.allows_publisher(topic_str) {
            warn!("Blocked unauthorized publisher on {}", topic_str);
            // Undo: could call rcl_publisher_fini, or just return error
            return RCL_RET_ERROR;
        }
    }
    ret
}
```

**Risks of blocking:**
- Nodes may not handle `rcl_publisher_init` failure gracefully
- Debug/diagnostic publishers would be blocked unless allowlisted
- Requires careful testing per node

This is why blocking is a future goal, not a v1 feature. The warn-only
path (v2) provides most of the value without the risk.

## Migration Path

| Version | Graph discovery | Audit location | Enforcement |
|---------|----------------|----------------|-------------|
| v1 (Phase 30) | Polling via `GraphSnapshot` | play_launch executor | Warn-only |
| v2 (future) | `GraphPlugin` via SPSC events | play_launch consumer | Warn-only, instant |
| v3 (future) | `GraphPlugin` | In-process (interception .so) | Blocking |

Each version builds on the previous. The `InterceptionPlugin` trait and
SPSC pipeline are already in place from Phase 29 — adding `GraphPlugin` is
a new plugin module, not an architectural change.
