# Phase 29: RCL Interception & Frontier Tracking

**Status**: Planned
**Priority**: Medium (Observability / Data Flow)
**Dependencies**: Phase 19 (Isolated Container — LD_PRELOAD inheritance), external `rcl_interception_sys` crate

---

## Overview

Build a Rust LD_PRELOAD interceptor library that hooks `rcl_publish` and `rcl_take` to extract `header.stamp` from every published/received message and track per-topic **timestamp frontiers** — the highest stamp seen on each topic. play_launch optionally injects this library into all managed nodes, giving transparent data flow visibility without modifying user code.

This is the same LD_PRELOAD pattern proven by CARET (Tier4) in production Autoware, reimplemented in Rust with a narrower scope: frontier tracking only, reporting via a lightweight Unix datagram socket side-channel.

### Why frontiers matter

In a multi-node pipeline (sensors → perception → planning → control), knowing the latest timestamp each node has produced reveals:

- **Pipeline stalls**: a node's frontier stops advancing while upstream continues
- **Dropped frames**: frontier jumps forward (gap in sequence)
- **End-to-end latency**: difference between sensor frontier and control frontier
- **Backpressure**: subscriber frontier lagging far behind publisher frontier on the same topic

### Architecture

```
┌─────────────────────────────────────────────────────┐
│  Node process (any ROS 2 node, unmodified)          │
│                                                     │
│  rcl_publish(pub, msg, alloc)                       │
│       │                                             │
│       ▼                                             │
│  ┌──────────────────────────────────────┐           │
│  │ libplay_launch_interception.so       │           │
│  │  (LD_PRELOAD)                        │           │
│  │                                      │           │
│  │  1. Read stamp at cached offset      │           │
│  │  2. CAS-update AtomicU64 frontier    │           │
│  │  3. sendto() 17-byte event           │           │
│  │  4. Call real rcl_publish via dlsym   │           │
│  └──────────────┬───────────────────────┘           │
│                 │ Unix datagram socket               │
└─────────────────┼───────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────┐
│  play_launch (replay process)                       │
│                                                     │
│  frontier/ module                                   │
│  - tokio task reading socket                        │
│  - aggregates per-topic frontier state              │
│  - logs to play_log/<ts>/frontier/                  │
└─────────────────────────────────────────────────────┘
```

### Crate layout

| Crate                      | Type   | Location                                         | Purpose                                                    |
|----------------------------|--------|--------------------------------------------------|------------------------------------------------------------|
| `rcl_interception_sys`     | lib    | `src/vendor/rcl_interception_sys/` (submodule)   | Bindgen-generated FFI types + hand-written opaque/fn types  |
| `play_launch_interception` | cdylib | `src/play_launch_interception/`                  | LD_PRELOAD .so with hooks, introspection, frontier, socket |

`play_launch_interception` depends on `rcl_interception_sys` via path. Neither crate is in the workspace `Cargo.toml` — they are built standalone, no colcon dependency. Both require a distro feature (`humble` or `jazzy`).

---

## Design

### Hooked functions

Four RCL functions are intercepted. Init hooks are **cold path** (once per publisher/subscriber). Publish/take hooks are **hot path** (every message).

| Hook                    | Signature                                            | Path    | Action                                                                         |
|-------------------------|------------------------------------------------------|---------|--------------------------------------------------------------------------------|
| `rcl_publisher_init`    | `(pub, node, type_support, topic, opts) → rcl_ret_t` | Cold    | Call original → register pub in registry with topic name + stamp offset        |
| `rcl_publish`           | `(pub, ros_message, alloc) → rcl_ret_t`              | **Hot** | Read stamp at cached offset → CAS-update frontier → send event → call original |
| `rcl_subscription_init` | `(sub, node, type_support, topic, opts) → rcl_ret_t` | Cold    | Call original → register sub in registry with topic name + stamp offset        |
| `rcl_take`              | `(sub, ros_message, msg_info, alloc) → rcl_ret_t`    | Hot     | Call original → if success, read stamp → send event                            |

Original function pointers resolved via `dlsym(RTLD_NEXT, ...)` at `#[ctor]` init time.

### Type introspection

To read `header.stamp` from an opaque `void* ros_message`, the interceptor resolves the byte offset at registration time (cold path) using rosidl introspection:

1. `type_support→func(type_support, "rosidl_typesupport_introspection_c")` → introspection handle
2. Cast `handle→data` to `MessageMembers*`
3. Walk `members[0..member_count_]`, find member with `name_ == "header"` and `type_id_ == 18` (ROS_TYPE_MESSAGE)
4. Recurse into `header.members_` → find `stamp` → its `offset_` within Header is 0
5. **stamp_offset** = `header.offset_ + 0`

At publish/take time: `stamp = *(builtin_interfaces__msg__Time*)(msg + stamp_offset)`

Messages without `header.stamp` (e.g. `std_msgs/String`) get `stamp_offset = None` and are skipped.

### Frontier state

Per-publisher `AtomicU64` storing packed `(sec: i32, nanosec: u32)`:

```rust
fn pack(sec: i32, nanosec: u32) -> u64 {
    ((sec as u64) << 32) | (nanosec as u64)
}
```

Updated via CAS loop — only advances forward (max). ~40-50ns per publish on hot path.

### Side-channel protocol

Unix datagram socket. Path from `PLAY_LAUNCH_INTERCEPTION_SOCKET` env var. Non-blocking, best-effort (dropped datagrams are acceptable — frontiers are idempotent).

```rust
#[repr(C, packed)]
struct FrontierEvent {    // 17 bytes total
    topic_hash: u64,      // FNV-1a hash of topic name
    stamp_sec: i32,
    stamp_nanosec: u32,
    event_type: u8,       // 0 = publish, 1 = take
}
```

### Inert mode

When `PLAY_LAUNCH_INTERCEPTION_SOCKET` is not set, the library sets an `INERT` flag at init. All 4 hooks immediately delegate to the original function with zero additional work. This makes it safe to leave LD_PRELOAD set even when frontier tracking is disabled.

### Thread safety

| Component                   | Mechanism                                        | Contention                     |
|-----------------------------|--------------------------------------------------|--------------------------------|
| Registry (pub/sub metadata) | `parking_lot::RwLock<HashMap<usize, TopicInfo>>` | Cold path only — no contention |
| Frontier per topic          | `AtomicU64` with CAS                             | Lock-free, ~40-50ns            |
| Socket send                 | `sendto()` is thread-safe on Unix datagrams      | No lock needed                 |
| Original fn pointers        | `static` initialized once at `#[ctor]`           | Immutable after init           |

---

## Work Items

### 29.0: `rcl_interception_sys` — FFI types (bindgen)

Struct types generated by bindgen from actual C headers, checked into `src/bindings/<distro>.rs`.
Opaque handle types and function pointer aliases are hand-written (shared across distros).

- [x] `rosidl_message_type_support_t` — bindgen-generated from `rosidl_runtime_c/message_type_support_struct.h`
- [x] `MessageMembers` / `MessageMember` — bindgen-generated from `rosidl_typesupport_introspection_c/message_introspection.h`
- [x] `builtin_interfaces__msg__Time` — bindgen-generated from `builtin_interfaces/msg/detail/time__struct.h`
- [x] Compile-time layout assertions (auto-generated by bindgen `layout_tests`)
- [x] Opaque handle types (hand-written `opaque.rs`): `rcl_node_t`, `rcl_publisher_t`, `rcl_subscription_t`, `rcl_publisher_options_t`, `rcl_subscription_options_t`, `rmw_publisher_allocation_t`, `rmw_subscription_allocation_t`, `rmw_message_info_t`
- [x] Function pointer type aliases (hand-written `fn_types.rs`): `FnRclPublisherInit`, `FnRclPublish`, `FnRclSubscriptionInit`, `FnRclTake`, `rcl_ret_t`
- [x] Convenience aliases (`BuiltinTime`, `MessageMember`, `MessageMembers`), `ROS_TYPE_MESSAGE`, `TYPESUPPORT_INTROSPECTION_C_IDENTIFIER`
- [x] Feature-gated distro selection: `humble` or `jazzy` (mandatory, enforced by `compile_error!`)
- [x] `generate-bindings` feature for developer regeneration workflow (optional bindgen dep)
- [x] `build.rs` discovers ROS include dirs from `AMENT_PREFIX_PATH` (4 required packages)
- [x] Pre-generated `src/bindings/humble.rs` checked in (no ROS install needed to build)

### 29.1: Crate scaffold — `play_launch_interception`

- [x] `src/play_launch_interception/Cargo.toml` — cdylib, deps: `rcl_interception_sys` (path), `libc`, `parking_lot`, `ctor`
- [x] `humble`/`jazzy` features forwarding to `rcl_interception_sys`
- [x] `src/play_launch_interception/src/lib.rs` — `#[ctor]` init, env var check, `INERT` flag, eager `dlsym` for 4 originals
- [x] Uses `FnRcl*` types from sys crate (no hand-written function pointer aliases)
- [x] Verify `.so` builds and can be loaded without crashing: `LD_PRELOAD=...so /bin/true`

### 29.2: Introspection — `find_stamp_offset()`

- [x] `introspection.rs` — `find_stamp_offset(type_support: *const rosidl_message_type_support_t) → Option<usize>`
- [x] Resolve introspection typesupport via `func` pointer
- [x] Walk `MessageMembers` → find "header" member with `type_id_ == 18`
- [x] Return `header.offset_` (stamp is at offset 0 within Header)
- [x] Return `None` for messages without `header` field
- [x] Unit tests with mock `MessageMembers` structs (3 tests: header found, no header, empty members)

### 29.3: Registry

- [x] `registry.rs` — separate `PUB_REGISTRY` / `SUB_REGISTRY` (`LazyLock<RwLock<HashMap<usize, Record>>>`)
- [x] `PubEntry` / `SubEntry`: topic hash (FNV-1a), stamp offset; PubEntry also holds `&'static AtomicU64` frontier ref
- [x] `register_publisher(ptr, topic, stamp_offset)` / `register_subscription(ptr, topic, stamp_offset)`
- [x] `lookup_publisher(ptr) → Option<PubEntry>` / `lookup_subscription(ptr) → Option<SubEntry>` — returns `None` for messages without stamp
- [x] Unit tests (4 tests: FNV-1a determinism, pub round-trip, pub-without-stamp, sub round-trip)

### 29.4: Frontier state

- [x] `frontier.rs` — `pack(sec, nanosec) → u64`, `update(frontier, sec, nanosec) → bool` CAS max-update
- [x] Global frontier map: `LazyLock<RwLock<HashMap<u64, &'static AtomicU64>>>` with `Box::leak` for process-lifetime entries
- [x] `get_or_create(topic_hash) → &'static AtomicU64` — read-lock fast path, write-lock slow path
- [x] Unit tests (3 tests: pack ordering, update advances, get_or_create identity)

### 29.5: Channel — Unix datagram sender

- [x] `channel.rs` — `init()` opens unbound socket, `connect()`s to `PLAY_LAUNCH_INTERCEPTION_SOCKET` path, sets non-blocking
- [x] `FrontierEvent` struct (17 bytes, `#[repr(C, packed)]`)
- [x] `send(event: &FrontierEvent)` — non-blocking `send()`, silently drops on error (best-effort)
- [x] No-op when channel not initialized
- [x] Unit test (1 test: event size is 17 bytes)

### 29.6: Hooks

- [x] 4 `#[no_mangle] pub unsafe extern "C" fn` in `lib.rs` matching exact rcl signatures
- [x] `rcl_publisher_init`: call original → on success, resolve introspection → register in registry
- [x] `rcl_publish`: lookup registry → if stamp_offset present, read stamp, update frontier, send event → call original
- [x] `rcl_subscription_init`: call original → on success, resolve introspection → register
- [x] `rcl_take`: call original → on success and stamp_offset present, read stamp, send event
- [x] All hooks pass through directly when `INERT` flag is set
- [x] `#[ctor] init()` calls `channel::init()` to determine INERT state

### 29.7: Manual validation ✅

- [x] Build: `cd src/play_launch_interception && cargo build --release --features humble`
- [x] Test with talker: `LD_PRELOAD=...so ros2 run demo_nodes_cpp talker` — hooks activate, String messages correctly skipped (no header.stamp)
- [x] Test with cam2image: `LD_PRELOAD=...so ros2 run image_tools cam2image --ros-args -p burger_mode:=true` — 24+ PUB events received with monotonically advancing timestamps
- [x] Write `scripts/frontier_listener.py` — reads and prints 17-byte FrontierEvent datagrams from Unix socket
- [x] Verify events arrive with correct topic hash and advancing timestamps — confirmed: all events from `/image` topic, stamps match wall-clock time
- [x] Test inert mode: `LD_PRELOAD=...so ros2 run demo_nodes_cpp talker` (no env var) — no "active" message, no events, no crash
- [x] **Bug fix**: added C++ introspection fallback (`rosidl_typesupport_introspection_cpp`) in `find_stamp_offset()` — C++ nodes use `rosidl_typesupport_cpp` type support, so `rosidl_typesupport_introspection_c` identifier alone returns NULL. Now tries C first, falls back to C++.
- **Known limitation**: LD_PRELOAD + Python/rclpy nodes fails with "error creating node" — dlsym(RTLD_NEXT) can't find rcl symbols because librcl.so is loaded lazily by rclpy's `_rclpy_pybind11.so`. Works correctly with C++ nodes (librcl.so is a direct dependency). This is acceptable since Autoware nodes are C++.

### 29.8: play_launch config & CLI

- [ ] Add `FrontierTrackingConfig { enabled: bool }` to `RuntimeConfig` in `src/play_launch/src/cli/config.rs`
- [ ] Add `FrontierTracking` variant to `Feature` enum in `src/play_launch/src/cli/options.rs`
- [ ] Wire `--enable frontier-tracking` to set config
- [ ] Add `interception_lib_path: Option<PathBuf>` and `interception_socket_path: Option<PathBuf>` to `NodeCommandLine`

### 29.9: LD_PRELOAD injection in node_cmdline.rs

- [ ] In `to_command()` (after line 343 — `command.envs(&self.env)`), inject:
  - `LD_PRELOAD` → path to `libplay_launch_interception.so`
  - `PLAY_LAUNCH_INTERCEPTION_SOCKET` → socket path
- [ ] Only inject when frontier tracking is enabled
- [ ] Composable nodes inherit env from container process automatically — no special handling needed

### 29.10: Frontier listener module

- [ ] Create `src/play_launch/src/frontier/mod.rs`
- [ ] `FrontierListener` — async tokio task binding Unix datagram socket
- [ ] Parse incoming `FrontierEvent` datagrams (17 bytes each)
- [ ] Maintain `HashMap<u64, FrontierState>` — latest stamp per topic hash
- [ ] Spawn from `replay.rs` following the monitoring/diagnostics pattern
- [ ] Graceful shutdown on replay exit

### 29.11: Frontier logging

- [ ] On replay completion, write frontier summary to `play_log/<ts>/frontier/summary.json`
- [ ] Format: `{ "topics": { "<topic_hash>": { "last_stamp_sec": N, "last_stamp_nanosec": N, "event_count": N } } }`
- [ ] Log final frontier state at `info!` level

### 29.12: Build integration

- [ ] Justfile recipe: `build-interception` — `cd src/play_launch_interception && cargo build --release`
- [ ] Add to `just build` if frontier feature is enabled (or always build, since it's inert when unused)
- [ ] Bundle script: add `"play_launch_interception/lib/libplay_launch_interception.so:lib/"` to ARTIFACTS in `scripts/bundle_wheel.sh`
- [ ] Verify `play_launch` can locate the .so at runtime (check relative to binary, then system path)

### 29.13: Integration test

- [ ] Test in `tests/` using `demo_nodes_cpp` talker/listener with frontier tracking enabled
- [ ] Verify frontier events are received on the socket
- [ ] Verify `frontier/summary.json` is written after replay completes
- [ ] Verify inert mode (tracking disabled) produces no socket, no events, no errors

### 29.14: Documentation

- [ ] Update `CLAUDE.md` — mention frontier tracking feature, env vars, new crate
- [ ] Update `docs/roadmap/README.md` — add Phase 29 entry
- [ ] Update `docs/guide/parser-features.md` or create `docs/guide/frontier-tracking.md` if warranted
- [ ] Document env vars: `PLAY_LAUNCH_INTERCEPTION_SOCKET`

---

## Verification

```bash
# Build interceptor
cd src/play_launch_interception && cargo build --release

# Manual smoke test (Phase 1)
PLAY_LAUNCH_INTERCEPTION_SOCKET=/tmp/frontier.sock \
  LD_PRELOAD=target/release/libplay_launch_interception.so \
  ros2 run demo_nodes_cpp talker
# → events on socket with advancing timestamps

# Inert mode (no env var → zero overhead)
LD_PRELOAD=target/release/libplay_launch_interception.so \
  ros2 run demo_nodes_cpp talker
# → no crash, no socket activity

# play_launch integration
just build-interception && just build-rust
play_launch replay record.json --enable frontier-tracking
# → frontier/summary.json in play_log/

# Autoware validation
just run-autoware --enable frontier-tracking
# → works with 46 nodes + 15 containers, frontier events for stamped topics
```

---

## Key design decisions

1. **Single interceptor, not a framework** — all frontier logic lives directly in the cdylib. No plugin trait, no dispatch table, no dynamic loading. If future interception needs arise, they are added to this crate directly.

2. **Separate sys crate with bindgen** — `rcl_interception_sys` uses bindgen to generate struct types from actual C headers, checked into `src/bindings/<distro>.rs`. Opaque handles and function pointer aliases are hand-written. Feature-gated per distro (`humble`/`jazzy`).

3. **Unix datagram socket, not DDS** — avoids rcl re-entrancy (calling rcl_publish from inside an rcl_publish hook could deadlock). Datagrams are connectionless and non-blocking.

4. **Best-effort reporting** — dropped socket events are acceptable. Frontiers are monotonically increasing, so any later event subsumes missed ones. This keeps the hot path fast and avoids backpressure on the node.

5. **Inert when unconfigured** — the library checks for `PLAY_LAUNCH_INTERCEPTION_SOCKET` once at init. If absent, all hooks are pure pass-through with no memory allocation, no socket I/O, no overhead.

6. **Not in workspace** — built standalone to avoid colcon dependency graph. Links to ROS only at runtime via `dlsym(RTLD_NEXT, ...)`.
