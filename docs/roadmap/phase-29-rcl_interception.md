# Phase 29: RCL Interception & Frontier Tracking

**Status**: In progress (interceptor complete, play_launch integration planned)
**Priority**: Medium (Observability / Data Flow)
**Dependencies**: Phase 19 (Isolated Container — LD_PRELOAD inheritance), external `rcl_interception_sys` crate

---

## Overview

A Rust LD_PRELOAD interceptor library that hooks `rcl_publish` and `rcl_take` to extract `header.stamp` from every published/received message. Compiled-in plugins process intercepted events:

- **FrontierPlugin** — per-topic timestamp frontier tracking (highest stamp seen)
- **StatsPlugin** — per-topic message counts, rates, and inter-message latency

play_launch optionally injects this library into all managed nodes via LD_PRELOAD, giving transparent data flow visibility without modifying user code. Communication between the interceptor and play_launch uses a zero-copy shared memory SPSC ring buffer.

This is the same LD_PRELOAD pattern proven by CARET (Tier4) in production Autoware, reimplemented in Rust with a trait-based plugin architecture for internal extensibility.

### Why this matters

In a multi-node pipeline (sensors -> perception -> planning -> control):

- **Pipeline stalls**: a node's frontier stops advancing while upstream continues
- **Dropped frames**: frontier jumps forward (gap in sequence)
- **End-to-end latency**: difference between sensor frontier and control frontier
- **Backpressure**: subscriber frontier lagging far behind publisher frontier on the same topic
- **Throughput**: per-topic publish/take rates reveal bottlenecks
- **Jitter**: inter-message latency variance shows scheduling instability

### Architecture

```
+-----------------------------------------------------+
|  Node process (any ROS 2 node, unmodified)           |
|                                                      |
|  rcl_publish(pub, msg, alloc)                        |
|       |                                              |
|       v                                              |
|  +----------------------------------------------+   |
|  | libplay_launch_interception.so  (LD_PRELOAD)  |   |
|  |                                               |   |
|  |  Runtime (OnceLock, lazy init)                |   |
|  |    originals: 4 rcl fn pointers (dlsym)       |   |
|  |    plugins: Vec<Box<dyn InterceptionPlugin>>  |   |
|  |                                               |   |
|  |  trait InterceptionPlugin                     |   |
|  |    +-- FrontierPlugin (frontier tracking)     |   |
|  |    +-- StatsPlugin (message statistics)       |   |
|  |                                               |   |
|  |  Both plugins write InterceptionEvent         |   |
|  |  to shared memory SPSC ring buffer            |   |
|  +------------------+----------------------------+   |
|                     | shared memory (memfd)            |
|                     | signaling (eventfd)              |
+---------------------+--------------------------------+
                      |
                      v
+-----------------------------------------------------+
|  play_launch (replay process)                        |
|                                                      |
|  InterceptionListener (tokio task)                   |
|  - polls eventfd via AsyncFd                         |
|  - reads InterceptionEvent from Consumer             |
|  - computes frontier state + message stats           |
|  - logs to play_log/<ts>/interception/               |
|  - exposes via web UI SSE                            |
+-----------------------------------------------------+
```

### Crate layout

| Crate | Type | Location | Purpose |
|---|---|---|---|
| `rcl_interception_sys` | lib | `src/vendor/rcl_interception_sys/` | Bindgen FFI types + hand-written opaque/fn types |
| `play_launch_interception` | cdylib | `src/play_launch_interception/` | LD_PRELOAD .so: hooks, registry, introspection, compiled-in plugins |
| `spsc_shm` | lib | `src/spsc_shm/` | Shared memory SPSC ring buffer + typed event protocol |

`play_launch_interception` depends on `rcl_interception_sys` and `spsc_shm` via path. Neither crate is in the workspace `Cargo.toml` — they are built standalone, no colcon dependency. The interception crate requires a distro feature (`humble` or `jazzy`).

`spsc_shm` has **zero external dependencies** (only `libc` for `memfd_create`/`mmap`/`eventfd`). It is consumed by both `play_launch_interception` (producer) and `play_launch` (consumer via path dependency in the main workspace).

---

## Design

### Hooked functions

Four RCL functions are intercepted. Init hooks are **cold path** (once per publisher/subscriber). Publish/take hooks are **hot path** (every message).

| Hook | Signature | Path | Action |
|---|---|---|---|
| `rcl_publisher_init` | `(pub, node, type_support, topic, opts) -> rcl_ret_t` | Cold | Call original -> register pub in registry -> dispatch `on_publisher_init` to plugins |
| `rcl_publish` | `(pub, ros_message, alloc) -> rcl_ret_t` | **Hot** | Lookup registry -> read stamp -> dispatch `on_publish` to plugins -> call original |
| `rcl_subscription_init` | `(sub, node, type_support, topic, opts) -> rcl_ret_t` | Cold | Call original -> register sub in registry -> dispatch `on_subscription_init` to plugins |
| `rcl_take` | `(sub, ros_message, msg_info, alloc) -> rcl_ret_t` | Hot | Call original -> if success, lookup registry -> read stamp -> dispatch `on_take` to plugins |

Original function pointers resolved lazily on first hook invocation via `OnceLock<Option<Runtime>>`.

### Lazy resolution (Python/rclpy support)

Python nodes load `librcl.so` lazily via `_rclpy_pybind11.so` with `RTLD_LOCAL`, so `dlsym(RTLD_NEXT)` can't find it at init time. Resolution strategy on first hook invocation:

1. `RTLD_NEXT` — works for C++ nodes (global symbol scope)
2. `dlopen("librcl.so", RTLD_NOLOAD)` + `dlsym(handle)` — works for Python nodes (local scope, already loaded)

### Type introspection

To read `header.stamp` from an opaque `void* ros_message`, the interceptor resolves the byte offset at registration time (cold path) using rosidl introspection:

1. `type_support->func(type_support, "rosidl_typesupport_introspection_c")` -> introspection handle
2. Cast `handle->data` to `MessageMembers*`
3. Walk `members[0..member_count_]`, find member with `name_ == "header"` and `type_id_ == 18` (ROS_TYPE_MESSAGE)
4. **stamp_offset** = `header.offset_` (stamp is at offset 0 within Header)
5. Falls back to `rosidl_typesupport_introspection_cpp` identifier for C++ nodes

Messages without `header.stamp` (e.g. `std_msgs/String`) get `stamp_offset = None` and are skipped.

### Plugin architecture (internal, dev-only)

All plugins are compiled into the interception `.so`. No dynamic loading, no C ABI, no manifest files.

```
play_launch_interception/src/
  lib.rs              -- Runtime (OnceLock), 4 hooks, lazy dlsym
  plugin.rs           -- trait InterceptionPlugin + Stamp type
  plugin_dispatch.rs  -- iterates Vec<Box<dyn InterceptionPlugin>>
  plugins/
    mod.rs
    frontier.rs       -- FrontierPlugin (frontier tracking)
    stats.rs          -- StatsPlugin (message statistics)
  registry.rs         -- pub/sub handle -> (topic_hash, stamp_offset)
  introspection.rs    -- rosidl stamp offset resolution
```

**Trait definition** (`plugin.rs`):

```rust
pub(crate) struct Stamp { pub sec: i32, pub nanosec: u32 }

pub(crate) trait InterceptionPlugin: Send + Sync {
    fn on_publisher_init(&self, handle: usize, topic: &str, topic_hash: u64, stamp_offset: Option<usize>);
    fn on_subscription_init(&self, handle: usize, topic: &str, topic_hash: u64, stamp_offset: Option<usize>);
    fn on_publish(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);
    fn on_take(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);
}
```

**Runtime initialization** (`lib.rs`):

```rust
struct Runtime {
    originals: Originals,
    plugins: Vec<Box<dyn InterceptionPlugin>>,
}

static RUNTIME: OnceLock<Option<Runtime>> = OnceLock::new();

fn build_plugins() -> Vec<Box<dyn InterceptionPlugin>> {
    let mut plugins: Vec<Box<dyn InterceptionPlugin>> = Vec::new();
    if let Some(frontier) = FrontierPlugin::new() { plugins.push(Box::new(frontier)); }
    if let Some(stats) = StatsPlugin::new() { plugins.push(Box::new(stats)); }
    plugins
}
```

Each plugin's constructor checks its activation env var and returns `Option<Self>`.

### Frontier plugin

`FrontierPlugin` tracks per-topic timestamp frontiers and writes events to the SPSC ring buffer when a frontier advances.

**Activation**: `PLAY_LAUNCH_INTERCEPTION_SHM_FD` env var (shared memory fd).

**State**: `RwLock<HashMap<u64, &'static AtomicU64>>` — per-topic frontier as packed `(sec, nanosec)` in `AtomicU64`, updated via CAS max-update (~40-50ns per publish).

**Behavior**:
- `on_publisher_init`: pre-creates frontier entry if message has stamp
- `on_publish`: CAS-update frontier, write `EventKind::Publish` to ring only if frontier advanced
- `on_take`: always writes `EventKind::Take` to ring (reports all takes)
- `on_subscription_init`: no-op

### Stats plugin

`StatsPlugin` writes every publish/take event to the SPSC ring buffer with a monotonic timestamp for rate and latency computation.

**Activation**: same `PLAY_LAUNCH_INTERCEPTION_SHM_FD` env var (shared with frontier).

**Behavior**:
- `on_publisher_init` / `on_subscription_init`: write `EventKind::PublisherInit` / `EventKind::SubscriptionInit` with topic name hash
- `on_publish`: write `EventKind::Publish` with stamp + `clock_gettime(MONOTONIC)`
- `on_take`: write `EventKind::Take` with stamp + `clock_gettime(MONOTONIC)`

play_launch computes derived metrics from the event stream:
- Per-topic publish/take count
- Message rate (events per second, windowed)
- Inter-message latency (delta between consecutive monotonic timestamps on same topic)
- Publish-to-take ratio (detects message loss)

### Inert mode

The library is inert (hooks pass through with zero extra work) when:
- `librcl.so` not found via either dlsym strategy (non-ROS process), OR
- originals resolved but no plugins activated (no relevant env vars set)

No `#[ctor]` — initialization is fully lazy via `OnceLock` on first hook invocation.

### Thread safety

| Component | Mechanism | Contention |
|---|---|---|
| Registry (pub/sub metadata) | `parking_lot::RwLock<HashMap<usize, Record>>` | Cold path only |
| Frontier per topic | `AtomicU64` with CAS | Lock-free, ~40-50ns |
| Ring buffer write | `Producer::push()` — atomic `write_idx` | Lock-free, SPSC (single writer per process) |
| Original fn pointers | `OnceLock<Option<Runtime>>` | Initialized once, immutable after |

---

## IPC: `spsc_shm` crate

A minimal, zero-dep (beyond `libc`) crate providing a typed SPSC ring buffer over shared memory. Usable by any Rust project — not specific to play_launch.

### Event type

```rust
#[repr(C)]
pub struct InterceptionEvent {  // 40 bytes
    pub kind: EventKind,        // u8
    pub _pad: [u8; 3],
    pub topic_hash: u64,
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub handle: u64,            // publisher/subscription pointer
    pub monotonic_ns: u64,      // clock_gettime(CLOCK_MONOTONIC)
}

#[repr(u8)]
pub enum EventKind {
    PublisherInit = 0,
    SubscriptionInit = 1,
    Publish = 2,
    Take = 3,
}
```

Note: `InterceptionEvent` is defined in `play_launch_interception` (or a thin protocol module), not in `spsc_shm`. The `spsc_shm` crate is generic over any `#[repr(C)]` + `Copy` type `T`.

### Ring buffer layout

```
+-------------------------------------------+
|  Header (64 bytes, cache-line aligned)    |
|    write_idx: AtomicU64  (producer owns)  |
|    read_idx:  AtomicU64  (consumer owns)  |
|    capacity:  u64                         |
|    _pad: [u8; 40]                         |
+-------------------------------------------+
|  Slots[0..capacity]: T                    |
|    each slot is size_of::<T>() bytes      |
+-------------------------------------------+
```

### SPSC Lamport queue (~200 lines)

- **Producer**: `if write_idx - read_idx < capacity` -> write slot at `write_idx % capacity`, `write_idx.store(w + 1, Release)`
- **Consumer**: `if read_idx < write_idx` -> read slot at `read_idx % capacity`, `read_idx.store(r + 1, Release)`
- **Overflow**: producer returns `Err(Full)` — caller decides to drop (best-effort)

### Setup

```rust
// play_launch (parent):
let (memfd, eventfd) = spsc_shm::create::<InterceptionEvent>(capacity)?;
// pass fds to child via env vars

// child (.so):
let producer = unsafe { spsc_shm::Producer::<InterceptionEvent>::from_fd(shm_fd)? };
producer.push(event);                          // no syscall
libc::eventfd_write(event_fd, 1);              // wakeup (optional batching)

// play_launch (consumer):
let consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_fd(shm_fd)? };
while let Some(event) = consumer.pop() { ... } // no syscall
```

### Fd passing

play_launch creates `memfd_create` + `eventfd` before spawning each child, passes via env vars:
- `PLAY_LAUNCH_INTERCEPTION_SHM_FD=/proc/self/fd/N`
- `PLAY_LAUNCH_INTERCEPTION_EVENT_FD=/proc/self/fd/M`

Fds inherited across `fork()+exec()` (clear `FD_CLOEXEC`). Per-process SPSC — one ring buffer per child, no cross-process contention.

### Evaluated alternatives

- [iceoryx2](https://docs.rs/iceoryx2/latest/iceoryx2/) — full IPC middleware (~180K SLoC, 15+ sub-crates). Too heavy for LD_PRELOAD cdylib. Risk of symbol conflicts with DDS middleware.
- [shmem-ipc](https://lib.rs/crates/shmem-ipc) — right architecture (memfd + eventfd + SPSC) but unmaintained (last release Nov 2022), outdated deps (zerocopy 0.3, memfd 0.4).
- [ipmpsc](https://crates.io/crates/ipmpsc) — requires serde + bincode, too heavy for LD_PRELOAD context.
- The core SPSC ring buffer is ~200 lines. A focused, generic implementation is simpler and more maintainable than vendoring an unmaintained crate.

---

## Configuration

Interception is configured in the play_launch YAML config file (`--config <PATH>`). No CLI flags — all settings under an `interception` section.

```yaml
# Example config
interception:
  enabled: true           # default: true
  frontier: true          # enable frontier tracking (default: true)
  stats: true             # enable message statistics (default: true)
  ring_capacity: 65536    # SPSC ring buffer capacity per child (default: 65536)
```

When `interception.enabled` is true, play_launch:
1. Creates memfd + eventfd per child process
2. Injects `LD_PRELOAD` and fd env vars into each spawned node
3. Spawns an `InterceptionListener` tokio task to consume events

Individual plugins (`frontier`, `stats`) can be toggled independently. The interception `.so` activates only the plugins whose env vars are set by play_launch.

When not using a config file, defaults apply: interception enabled with all plugins active.

---

## Work Items

### Phase A: Interceptor (complete)

#### 29.0: `rcl_interception_sys` — FFI types (bindgen)

- [x] Bindgen-generated structs from C headers (`rosidl_message_type_support_t`, `MessageMembers`, `builtin_interfaces__msg__Time`)
- [x] Hand-written opaque types and function pointer aliases
- [x] Compile-time layout assertions, feature-gated distro selection (`humble`/`jazzy`)
- [x] Pre-generated bindings checked in (no ROS install needed to build)

#### 29.1: Crate scaffold — `play_launch_interception`

- [x] cdylib, deps: `rcl_interception_sys` (path), `libc`, `parking_lot`
- [x] `OnceLock<Option<Runtime>>` lazy init with RTLD_NEXT + RTLD_NOLOAD fallback
- [x] Verify: `LD_PRELOAD=...so /bin/true` (no crash)

#### 29.2: Introspection — `find_stamp_offset()`

- [x] C and C++ introspection identifier fallback
- [x] Unit tests (3 tests: header found, no header, empty members)

#### 29.3: Registry

- [x] `PUB_REGISTRY` / `SUB_REGISTRY` with `LazyLock<RwLock<HashMap>>`
- [x] `lookup_publisher` / `lookup_subscription` (stamp-only) + `_full` variants
- [x] Unit tests (5 tests)

#### 29.4: Plugin architecture

- [x] `trait InterceptionPlugin` + `Stamp` type
- [x] Dispatch functions iterating `&[Box<dyn InterceptionPlugin>]`
- [x] `build_plugins()` in `lib.rs`, no C ABI / dlopen / manifest

#### 29.5: Frontier plugin (socket-based, initial)

- [x] `FrontierPlugin::new()` — activated by `PLAY_LAUNCH_INTERCEPTION_SOCKET` env var
- [x] Per-topic `AtomicU64` frontier with CAS max-update
- [x] 17-byte `FrontierEvent` over Unix datagram socket (non-blocking, best-effort)
- [x] Unit tests (3) + integration tests over real sockets (7)

#### 29.6: Hooks

- [x] 4 `#[unsafe(no_mangle)]` hook functions dispatching to plugins
- [x] Inert when no originals found or plugins list empty

#### 29.7: Manual validation

- [x] Tested with talker, cam2image, Python nodes
- [x] Bug fixes: C++ introspection fallback, Python lazy resolution

### Phase B: IPC crate + plugins on shared memory

#### 29.8: `spsc_shm` crate

- [ ] Create `src/spsc_shm/Cargo.toml` — lib (rlib), dep: `libc` only, own `[workspace]`, `.gitignore` for `/target/`
- [ ] Ring buffer header layout: `write_idx: AtomicU64`, `read_idx: AtomicU64`, `capacity: u64` (cache-line padded to 64 bytes)
- [ ] `create::<T>(capacity) -> Result<(OwnedFd, OwnedFd)>` — `memfd_create` + `ftruncate` + `eventfd`
- [ ] `Producer::<T>::from_fd(shm_fd) -> Result<Self>` — mmap, validate header, write-side handle
- [ ] `Consumer::<T>::from_fd(shm_fd) -> Result<Self>` — mmap, validate header, read-side handle
- [ ] `Producer::push(&self, item: &T) -> Result<(), Full>` — write slot, advance `write_idx`
- [ ] `Consumer::pop(&self) -> Option<T>` — read slot, advance `read_idx`
- [ ] `T: Copy + repr(C)` bound on the generic parameter
- [ ] Unit tests: SPSC correctness (sequential + concurrent), overflow returns `Full`, empty returns `None`
- [ ] Standalone smoke test: create ring, push N events from spawned child, read N events in parent

#### 29.9: `InterceptionEvent` type

- [ ] Define `InterceptionEvent` (40 bytes, `#[repr(C)]`, `Copy`) and `EventKind` enum in a shared module
- [ ] Place in `spsc_shm` as a feature-gated submodule, or in a thin `play_launch_interception_event` crate, or directly in `play_launch_interception` and re-export — decide based on dependency hygiene
- [ ] FNV-1a hash function: single source of truth (move from `registry.rs`, import in both sides)

#### 29.10: Migrate frontier plugin to shared memory

- [ ] Add `spsc_shm` dependency to `play_launch_interception`
- [ ] `FrontierPlugin` constructor: accept `Producer<InterceptionEvent>` (shared among plugins via `Arc`)
- [ ] Replace `send_event()` (socket sendto) with `producer.push()` (atomic store)
- [ ] Write `EventKind::Publish` / `EventKind::Take` with stamp fields
- [ ] Keep `PLAY_LAUNCH_INTERCEPTION_SOCKET` fallback for standalone use / debugging
- [ ] Update integration tests to verify events via ring buffer

#### 29.11: Stats plugin

- [ ] Create `plugins/stats.rs` — `StatsPlugin` implementing `InterceptionPlugin`
- [ ] Shares the same `Arc<Producer<InterceptionEvent>>` as frontier plugin
- [ ] `on_publish` / `on_take`: write event with `clock_gettime(CLOCK_MONOTONIC)` in `monotonic_ns` field
- [ ] `on_publisher_init` / `on_subscription_init`: write init events
- [ ] Activated by same env var as frontier (both use the shared ring buffer)
- [ ] Unit tests

### Phase C: play_launch integration

#### 29.12: Config file — interception settings

- [ ] Add `InterceptionConfig` to `RuntimeConfig` in `config.rs`:
  ```rust
  #[derive(Debug, Clone, Deserialize)]
  pub struct InterceptionConfig {
      #[serde(default = "default_true")]
      pub enabled: bool,
      #[serde(default = "default_true")]
      pub frontier: bool,
      #[serde(default = "default_true")]
      pub stats: bool,
      #[serde(default = "default_ring_capacity")]
      pub ring_capacity: usize,  // default: 65536
  }
  ```
- [ ] Add `interception: InterceptionConfig` field to `RuntimeConfig` and `ResolvedRuntimeConfig`
- [ ] Flow through to replay/run commands

#### 29.13: Shared memory setup + LD_PRELOAD injection

- [ ] Resolve path to `libplay_launch_interception.so` at startup (relative to binary, then system path)
- [ ] Before spawning each child: `spsc_shm::create::<InterceptionEvent>(ring_capacity)` -> `(memfd, eventfd)`
- [ ] Clear `FD_CLOEXEC` on both fds so they survive exec
- [ ] In `to_command()`, inject env vars:
  - `LD_PRELOAD` -> interception .so path
  - `PLAY_LAUNCH_INTERCEPTION_SHM_FD=/proc/self/fd/N`
  - `PLAY_LAUNCH_INTERCEPTION_EVENT_FD=/proc/self/fd/M`
  - Plugin activation env vars based on config (`frontier`, `stats`)
- [ ] Composable nodes inherit env from container process automatically

#### 29.14: `InterceptionListener` — consumer tokio task

- [ ] Create `src/play_launch/src/interception/mod.rs`
- [ ] `InterceptionListener` — async tokio task per child, polls eventfd via `AsyncFd`
- [ ] Reads `InterceptionEvent` from `Consumer`, dispatches to:
  - Frontier aggregator: `HashMap<u64, FrontierState>` (latest stamp per topic hash)
  - Stats aggregator: per-topic counters, rate windows, latency tracking
- [ ] Spawn from `replay.rs` following the monitoring/diagnostics pattern
- [ ] Graceful shutdown on replay exit

#### 29.15: Logging + web UI

- [ ] On replay completion, write to `play_log/<ts>/interception/`:
  - `frontier_summary.json` — last stamp + event count per topic
  - `stats_summary.json` — total counts, avg rate, avg latency per topic
- [ ] Log final state at `info!` level
- [ ] Expose live stats via web UI SSE endpoint (optional, follow existing pattern)

#### 29.16: Build integration

- [ ] Justfile recipe: `build-interception` — builds `spsc_shm` + `play_launch_interception`
- [ ] Add to `just build` (inert when unused)
- [ ] Bundle script: add `libplay_launch_interception.so` to ARTIFACTS
- [ ] Verify play_launch locates the .so at runtime

#### 29.17: Integration tests

- [ ] Test with `demo_nodes_cpp` talker/listener: verify events flow through ring buffer
- [ ] Verify frontier + stats summaries written after replay
- [ ] Verify inert mode (interception disabled in config) — no LD_PRELOAD, no events
- [ ] Verify config toggles: frontier-only, stats-only, both

#### 29.18: Documentation

- [ ] Update `CLAUDE.md` — interception config, crate layout, env vars
- [ ] Update `docs/roadmap/README.md`
- [ ] Document config file `interception` section with examples

---

## Verification

```bash
# Build interceptor + IPC crate
cd src/spsc_shm && cargo test
cd src/play_launch_interception && cargo build --release --features humble && cargo test --features humble

# Manual smoke test — standalone (socket fallback)
PLAY_LAUNCH_INTERCEPTION_SOCKET=/tmp/frontier.sock \
  LD_PRELOAD=target/release/libplay_launch_interception.so \
  ros2 run demo_nodes_cpp talker

# play_launch integration (after Phase C)
just build
play_launch replay record.json --config my_config.yaml
# config: interception: { enabled: true, frontier: true, stats: true }
# -> interception/frontier_summary.json + stats_summary.json in play_log/

# Autoware validation
just run-autoware --config autoware_config.yaml
# -> works with 46 nodes + 15 containers
```

---

## Key design decisions

1. **Trait-based compiled-in plugins** — `InterceptionPlugin` trait for internal code organization. All implementations compiled into the cdylib. No dynamic loading, no C ABI, no manifest files, no user-facing plugin API. Future plugins are added as new `plugins/*.rs` modules.

2. **Lazy initialization** — `OnceLock<Option<Runtime>>` instead of `#[ctor]`. Defers dlsym resolution to first hook invocation. Enables Python/rclpy support (librcl.so loaded with RTLD_LOCAL after process init).

3. **Separate sys crate with bindgen** — `rcl_interception_sys` uses bindgen for struct types from C headers. Feature-gated per distro (`humble`/`jazzy`).

4. **Generic SPSC crate** — `spsc_shm` is a standalone, reusable crate for any `#[repr(C)] + Copy` type. Not tied to play_launch. Zero deps beyond `libc`.

5. **Zero-copy shared memory** — `memfd_create` + `mmap` SPSC ring buffer. No syscall per event, no kernel copy. `eventfd` for async wakeup. Per-process SPSC avoids contention.

6. **Unified ring buffer for all plugins** — frontier and stats share one `Producer` per child process. Both write `InterceptionEvent` to the same ring. play_launch's consumer dispatches by `EventKind`. Simpler than per-plugin channels.

7. **Config file, not CLI** — interception settings live in the YAML config file alongside monitoring, diagnostics, and container settings. No CLI flags for individual plugins. Keeps the CLI surface clean.

8. **Best-effort reporting** — ring buffer overflow drops events silently. Frontiers are monotonically increasing. Stats tolerate gaps (rates computed from received events). Hot path never blocks on I/O.

9. **Not in workspace** — interception crates built standalone to avoid colcon dependency graph. Links to ROS only at runtime via `dlsym`.
