# Phase 19: Clone-Isolated Component Manager

**Status**: 🔧 In Progress (19.0–19.2 complete, 19.3 partially complete, 19.4–19.5 complete)
**Priority**: High (crash isolation, per-node resource control)
**Dependencies**: Phase 18 complete. ObservableComponentManager merged.

## Overview

Replace `std::thread`-based executor isolation with `clone(CLONE_VM)`-based
process isolation. Each composable node gets its own Linux PID while sharing the
container's address space for zero-copy intra-process communication.

## Design References

- `docs/container-isolation-design.md` — background, rationale, Linux isolation analysis
- `docs/clone-vm-container-design.md` — detailed implementation design

## Implementation Order

```
19.0 Consolidate executable          ✅ complete
  └── 19.1 CloneIsolatedComponentManager  ✅ complete
        ├── 19.2 Death monitor            ✅ complete
        │     └── 19.3 Integration tests  ⏳ partial (19.4b complete)
        └── 19.4 play_launch integration
              ├── 19.4a ComponentEvent sub ✅ complete
              ├── 19.4b Container tests   ✅ complete
              └── 19.4c Exec rewrite      ⏳ planned
        └── 19.5 Event-driven container status  ✅ complete
              └── 19.8 Parallel node loading   ⏳ planned
                    ├── 19.8a C++ async loading  (non-blocking on_load_node)
                    ├── 19.8b Rust parallel dispatch (concurrent in-flight loads)
                    └── 19.8c Integration tests  (TensorRT-like slow loader)
              ├── 19.6 cgroups            (optional)
              └── 19.7 MPK               (optional, experimental)
```

---

## Phase 19.0: Consolidate Container Executable ✅

**Status**: Complete

Merged `component_container.cpp` and `component_container_mt.cpp` into a single
binary with `--use_multi_threaded_executor` CLI flag.

### Work Items

- [x] Rewrite `component_container.cpp` with CLI flag parsing
- [x] Delete `component_container_mt.cpp`
- [x] Update `CMakeLists.txt` to remove MT target

### Files Changed

| File | Action |
|------|--------|
| `src/play_launch_container/src/component_container.cpp` | Rewritten: single main() with CLI flags |
| `src/play_launch_container/src/component_container_mt.cpp` | Deleted |
| `src/play_launch_container/CMakeLists.txt` | Removed `component_container_mt` target |

### Passing Criteria

- [x] Single `component_container` executable builds
- [x] Default mode: single-threaded executor (same as before)
- [x] `--use_multi_threaded_executor` flag selects MT executor
- [x] `thread_num` parameter works in MT mode
- [x] Existing tests pass (326/326)
- [x] `component_container_mt.cpp` deleted

---

## Phase 19.1: CloneIsolatedComponentManager ✅

**Status**: Complete

New C++ class that subclasses `ObservableComponentManager` and uses
`clone(CLONE_VM)` to spawn each node's executor in its own process.

### Work Items

- [x] Create `clone_isolated_component_manager.hpp` header
- [x] Implement `add_node_to_executor()` — clone child with dedicated executor
- [x] Implement `remove_node_from_executor()` — cancel + waitpid + cleanup
- [x] Implement `cleanup_child()` — destructor helper (SIGTERM → SIGKILL)
- [x] Add `--isolated` CLI flag to `component_container.cpp`
- [x] Solve glibc TLS sharing — `_dl_allocate_tls()` + `CLONE_SETTLS`
- [x] Solve TID-in-TLS — runtime offset discovery + `ARCH_GET_FS` in child
- [x] Fix shutdown crash — explicit `exec->remove_node(); node.reset(); rclcpp::shutdown()` ordering
- [x] Change `event_pub_` from private to protected in `ObservableComponentManager`
- [x] Add new source to `CMakeLists.txt`

### Files Changed

| File                                   | Action                                                    |
|----------------------------------------|-----------------------------------------------------------|
| `clone_isolated_component_manager.hpp` | New — class declaration + ChildInfo struct                |
| `clone_isolated_component_manager.cpp` | New — clone(CLONE_VM) + TLS + TID implementation          |
| `observable_component_manager.hpp`     | Modified — `event_pub_` private → protected               |
| `component_container.cpp`              | Modified — `--isolated` flag + explicit shutdown ordering |
| `CMakeLists.txt`                       | Modified — added new source file                          |

### Key Technical Decisions

- **clone() not clone3()**: glibc's `clone()` wrapper handles stack setup and
  TLS passing via `CLONE_SETTLS`. `clone3()` would require a raw syscall wrapper
  with no benefit for our use case.
- **`_dl_allocate_tls` + CLONE_SETTLS**: Without separate TLS, parent and child
  share glibc's per-thread malloc cache (tcache) → double-free. Fresh TLS block
  allocated via glibc internal `_dl_allocate_tls(nullptr)`.
- **TID offset discovery**: `_dl_allocate_tls` zeroes the TLS block, leaving
  `tid=0`. DDS calls `pthread_create` internally, which checks caller's TID →
  EAGAIN. Solved by scanning parent's TLS for matching TID at startup (offset
  720 on glibc 2.35 x86_64), then setting child's TID via `ARCH_GET_FS` +
  memcpy.
- **pidfd via `pidfd_open()`**: Obtained after clone() rather than via
  `CLONE_PIDFD` (which requires clone3). Used for Phase 19.2 death monitor.

### Passing Criteria

- [x] `component_container --isolated` starts and stays alive
- [x] LoadNode creates clone'd child in S state (not zombie)
- [x] Child spins executor and processes callbacks (Talker publishes)
- [x] Cross-node pub/sub works (Listener receives Talker messages)
- [x] Two children alive (S state) after loading 2 components
- [x] UnloadNode kills child cleanly (child count drops 2 → 1)
- [x] Container survives unload
- [x] Clean shutdown on SIGTERM (no crash, no core dump)
- [x] Existing tests pass (326/326)

---

## Phase 19.2: Child Death Monitor ✅

**Status**: Complete

Detect child crashes via `epoll` on pidfds, clean up resources, publish events.

### Work Items

- [x] Create monitor thread (spawned in constructor, joined in destructor)
- [x] Use `epoll_create1()` + `epoll_ctl()` to watch child pidfds
- [x] Register pidfds directly from `add_node_to_executor()` (epoll_ctl is thread-safe)
- [x] On child death: `waitpid()` + `WIFSIGNALED`/`WIFEXITED` for exit info
- [x] Clean up dead child resources (munmap stack, close pidfd, free TLS, delete boot)
- [x] Publish `CRASHED` event (new type, `uint8 CRASHED=3`) via inherited `event_pub_`
- [x] Log crash details with RCLCPP_ERROR (node name, PID, signal name, core dump flag)
- [x] Handle monitor thread shutdown (eventfd wakeup + `monitor_running_` flag)
- [x] Graceful degradation if epoll/eventfd fails (log warning, skip monitor)

### Key Technical Decisions

- **`CRASHED` event type**: Added `uint8 CRASHED=3` to `ComponentEvent.msg` rather
  than reusing `LOAD_FAILED`. Semantically distinct: node was running then died,
  vs. failed to load initially.
- **`waitpid` not `waitid(P_PIDFD)`**: `P_PIDFD` not available in glibc 2.35
  headers. `waitpid` with `WIFSIGNALED`/`WIFEXITED`/`WCOREDUMP` macros gives the
  same information.
- **No notification fd for new pidfds**: `epoll_ctl(EPOLL_CTL_ADD)` is thread-safe,
  so pidfds are registered directly from `add_node_to_executor()` without a
  notification pipe.
- **Race resolution**: `remove_node_from_executor` deregisters pidfd from epoll
  BEFORE killing the child. Both paths lock `children_mutex_`; first to acquire
  handles cleanup, second finds entry gone and returns.

### Files Changed

| File                                      | Action                                                          |
|-------------------------------------------|-----------------------------------------------------------------|
| `play_launch_msgs/msg/ComponentEvent.msg` | Added `uint8 CRASHED=3`                                         |
| `clone_isolated_component_manager.hpp`    | Added monitor members (epoll_fd_, stop_fd_, thread, atomic)     |
| `clone_isolated_component_manager.cpp`    | Monitor implementation (constructor, destructor, loop, handler) |

### Passing Criteria

- [x] Monitor thread starts with container and stops on shutdown
- [x] Crash event published on `~/_container/component_events` topic
- [x] Resources cleaned up (stack freed, pidfd closed, TLS freed)
- [x] Container process itself survives child crash
- [x] Existing tests pass (326/326)

---

## Phase 19.3: Integration Tests

**Status**: Partially complete (19.4b covers basic container tests)

Automated tests for crash isolation and normal operation.

### Work Items

- [x] Create test workspace `tests/fixtures/container_events/` (done in 19.4b)
- [x] Test: Normal load cycle — 2 composable nodes load into container (done in 19.4b)
- [x] Test: Crash isolation — kill isolated child, verify crash detection (done in 19.4b)
- [ ] Create `crash_node.cpp` — composable node that segfaults on timer
- [ ] Create `healthy_node.cpp` — composable node that publishes at 10 Hz
- [ ] Test 19.3.3: Intra-process communication (pub/sub between isolated nodes)
- [ ] Test 19.3.4: Per-node PID visibility (`kill(pid, 0)` checks)

### Passing Criteria

- [x] Basic container load + crash detection tests pass (6 tests)
- [ ] Custom crash_node / healthy_node test cases pass
- [ ] Tests complete within 60s timeout

---

## Phase 19.4: play_launch Integration (Rust)

**Status**: In progress (19.4a + 19.4b complete)

### Phase 19.4a: ComponentEvent Subscription ✅

Added ComponentEvent subscription to `container_actor.rs`. Currently only
handles CRASHED events; LOADED/UNLOADED/LOAD_FAILED are discarded (handled
by service responses).

- [x] Subscribe to `~/_container/component_events` topic
- [x] Bridge ROS callback to tokio channel
- [x] Handle CRASHED → ComposableState::Failed transition
- [x] Log crash with error!() including signal/exit details

### Phase 19.4b: Container Integration Tests ✅

Parser fix + test fixtures + integration tests for container events.

- [x] Add `args` attribute support to `<node_container>` in parser
- [x] Create `tests/fixtures/container_events/` with launch files
- [x] 6 integration tests: dump, parity, args validation, live launch, crash detection
- [x] All 331 tests pass (311 parser + 20 integration)

### Phase 19.4c: Container Exec Rewrite (Planned)

**Goal**: At runtime, transparently replace stock `rclcpp_components` containers
with `play_launch_container`, preserving the ST/MT flavor from the launch file
and unconditionally enabling `--isolated` mode.

**Why unconditional isolation**: Crash isolation and per-node PID visibility
outweigh the risks for non-safety-critical use. The main trade-off is that
`ReentrantCallbackGroup` concurrency within a single node becomes serialized
(each child has its own executor). This doesn't break correctness but may
reduce throughput for nodes that rely on concurrent callback execution.

**Record dump stays faithful**: The parser records the exact `executable` and
`package` from the launch file (e.g., `component_container_mt` /
`rclcpp_components`). The rewrite is a runtime-only concern — record.json
preserves the original intent for analysis.

**Exec rewrite mapping**:

```
record.json executable        →  play_launch_container flags
──────────────────────────────────────────────────────────────
component_container           →  component_container --isolated
component_container_mt        →  component_container --use_multi_threaded_executor --isolated
component_container_isolated  →  component_container --isolated
```

**Why preserve ST/MT flavor**: Autoware's choice is intentional — 4 ST
containers (MRM safety operators, velocity smoother) use single-threaded
executors for deterministic sequential execution; 11 MT containers
(planning, control, perception) use multi-threaded executors for parallel
callback processing. The `--use_multi_threaded_executor` flag must be
forwarded to respect this design.

**C++ container fix — MT executor for isolated children**:

Currently `CloneIsolatedComponentManager::add_node_to_executor()` hardcodes
`SingleThreadedExecutor` for each child (line 215). When the container is
started with `--use_multi_threaded_executor`, only the *parent* executor is
MT — children always get ST. This means the MT flag is silently ignored in
isolated mode.

The upstream `ComponentManagerIsolated` solves this with a template parameter
`ExecutorT`. Our class uses runtime dispatch, so the fix is:

- [ ] Add `bool use_multi_threaded_` member to `CloneIsolatedComponentManager`
- [ ] Accept it as constructor parameter (default `false`)
- [ ] In `add_node_to_executor()`, create `MultiThreadedExecutor` or
  `SingleThreadedExecutor` based on the flag
- [ ] Pass `use_multi_threaded` from `component_container.cpp` to the
  constructor

**Rust runtime — exec rewrite**:

- [ ] Add exec rewrite step in `prepare_container_contexts()` or
  `NodeCommandLine::from_node_record()` — map `rclcpp_components` package
  to `play_launch_container` and normalize executable to `component_container`
- [ ] Translate `component_container_mt` → `--use_multi_threaded_executor` arg
- [ ] Always append `--isolated` flag
- [ ] Add config option to disable isolation (`use_isolated: false`) as escape hatch
- [ ] Add optional `auto_reload_on_crash` config (re-issue LoadNode on crash)
- [ ] Update Autoware smoke test to verify rewritten containers start correctly

**Known risks with unconditional isolation**:

| Risk                                   | Severity   | Mitigation                                        |
|----------------------------------------|------------|---------------------------------------------------|
| Mutex poisoning on child crash         | High       | jemalloc + robust mutexes (future phase)          |
| ReentrantCallbackGroup serialized      | Low-Medium | Each child still gets MT executor if `_mt` flavor |
| Memory overhead (+450 MB for Autoware) | Low        | Acceptable for modern systems                     |
| DDS discovery graph growth (n² pairs)  | Low        | FastDDS Discovery Server                          |

### Passing Criteria

- [x] CRASHED events detected and logged
- [x] Integration tests cover dump, parity, launch, crash detection
- [ ] All `rclcpp_components` containers rewritten to `play_launch_container` at runtime
- [ ] ST/MT flavor preserved via `--use_multi_threaded_executor` flag
- [ ] `--isolated` added unconditionally
- [ ] Autoware smoke test passes with rewritten containers (49/49 processes)
- [ ] Config escape hatch (`use_isolated: false`) works

---

## Phase 19.5: Event-Driven Container Status

**Status**: ✅ Complete

Replace service-response-based composable node tracking with ComponentEvent
subscription as the primary source of truth.

### Motivation

Today `container_actor.rs` tracks composable node status via **two redundant
paths**:

1. **LoadNode/UnloadNode service responses** (primary) — the container actor
   calls the service, awaits the response, and transitions state based on
   `response.success`.
2. **ComponentEvent subscription** (secondary) — `ObservableComponentManager`
   publishes LOADED/LOAD_FAILED/UNLOADED events from within the same service
   handler, but the Rust side only uses CRASHED events; the rest are discarded
   with `_ => {}`.

Additionally, a **ListNodes polling mechanism** exists to detect "stuck"
composable nodes whose service responses were lost:

```
container_actor  →  StateEvent::ListNodesRequested
  →  MemberRunner  →  ListNodesManager
  →  ListNodes service call  →  StateEvent::NodeDiscovered
  →  MemberRunner  →  ControlEvent::DiscoveredLoaded
  →  container_actor  →  transition Loading → Loaded
```

This is a 6-hop round-trip with a 30-second timeout before it even starts.
ComponentEvent can replace it entirely.

### Problems with the current approach

| Problem                    | Detail                                                                                 |
|----------------------------|----------------------------------------------------------------------------------------|
| Service call hangs forever | `call_load_node_service()` has no timeout (line 473: "NO timeout — wait indefinitely") |
| 200ms warmup hack          | Service readiness race requires a hard-coded sleep (line 471)                          |
| ListNodes round-trip       | 6 async hops across 3 components to verify a single node                               |
| DiscoveredLoaded race      | Matches by `Loading` state, not by node name — first match wins (line 966)             |
| Duplicate tracking         | Both service response and ComponentEvent carry LOADED/LOAD_FAILED results              |
| Dead code path             | `response_tx` channel kept for "legacy composable node actors" (line 1702)             |

### Why ComponentEvent is the right primary source

- **Already published**: `ObservableComponentManager::on_load_node()` publishes
  LOADED or LOAD_FAILED synchronously in the service handler, with all the same
  fields (unique_id, full_node_name, error_message).
- **Crash detection proven**: Phase 19.4a already relies on CRASHED events from
  `CloneIsolatedComponentManager` — this just extends the same pattern to
  LOADED/LOAD_FAILED/UNLOADED.
- **QoS guarantees**: Reliable + transient_local with depth 100 means late
  subscribers get history and events survive brief disconnections.
- **No polling**: Events arrive at publish time — no 30-second timeout, no
  rate-limited ListNodes queries, no coordinator round-trip.

### Phase 19.5a: Handle LOADED/LOAD_FAILED events ✅

**Goal**: Transition composable node state from ComponentEvent instead of
service response.

Expand `handle_component_event()` to handle all four event types:

```rust
async fn handle_component_event(&mut self, event: ComponentEvent) {
    match event.event_type {
        ComponentEvent::LOADED => {
            // Find composable in Loading state by matching plugin/package/node_name
            // Transition to Loaded { unique_id }
            // Emit StateEvent::LoadSucceeded
            // If current_load matches, take it (stop awaiting response)
        }
        ComponentEvent::LOAD_FAILED => {
            // Find composable in Loading state by matching plugin/package
            // Transition to Failed { error }
            // Emit StateEvent::LoadFailed
            // If current_load matches, take it
        }
        ComponentEvent::UNLOADED => {
            // Find composable by unique_id
            // Transition to Unloaded
            // Emit StateEvent::Unloaded
            // If current_unload matches, take it
        }
        ComponentEvent::CRASHED => {
            // (existing logic, unchanged)
        }
    }
}
```

**Key matching logic**: LOADED/LOAD_FAILED events don't carry a
pre-assigned node name that maps to our composable entry names. Match by
correlating the `current_load` request's plugin name with the event's
`plugin_name` field. For UNLOADED, match by `unique_id` (already stored in
`ComposableNodeEntry`).

**Service response branch**: Keep the service response select branch but
change it to a **no-op if the event already handled the transition**. Check
`entry.state != Loading` before transitioning. This provides a graceful
fallback for stock `rclcpp_components` containers that don't publish
ComponentEvents.

**Fallback for stock containers**: If `component_event_sub` is `None` (no
subscription created because the topic doesn't exist), fall back to the
current service-response path. This means the refactoring is backward
compatible.

### Phase 19.5b: Add timeout to LoadNode service call ✅

**Goal**: Stop waiting forever for service responses.

Change `call_load_node_service()` to use `tokio::time::timeout`:

```rust
let timeout = Duration::from_secs(self.config.load_service_timeout_secs);
match tokio::time::timeout(timeout, response_future).await {
    Ok(Ok(response)) => Ok(response),
    Ok(Err(e)) => Err(service_call_error(e)),
    Err(_) => Err(eyre!("LoadNode service call timed out after {}s", timeout)),
}
```

Default timeout: 30 seconds (same as current service readiness timeout).

This timeout is now a safety net only — the ComponentEvent should arrive
first. If the service times out but the LOADED event already arrived, the
composable node is already in `Loaded` state and the timeout is harmless.

### Phase 19.5c: Remove ListNodes polling ✅

**Goal**: Eliminate the `check_loading_timeouts()` timer, `ListNodesManager`,
`ListNodesRequested`/`NodeDiscovered`/`DiscoveredLoaded` event types, and
the coordinator forwarding path.

**Prerequisite**: Phase 19.5a must be complete. The stuck-node scenario that
ListNodes was designed to catch is now handled by:
- LOADED/LOAD_FAILED ComponentEvent arrives → immediate state transition
- Service call timeout (Phase 19.5b) → transition to Failed

**Remove**:

| Item                              | File                 | What                          |
|-----------------------------------|----------------------|-------------------------------|
| `check_loading_timeouts()`        | `container_actor.rs` | Periodic timer check          |
| `list_nodes_requested` field      | `container_actor.rs` | Per-node flag                 |
| `handle_discovered_loaded()`      | `container_actor.rs` | DiscoveredLoaded handler      |
| Timeout tick branch               | `container_actor.rs` | `select!` branch in main loop |
| `DiscoveredLoaded` variant        | `events.rs`          | ControlEvent variant          |
| `ListNodesRequested` variant      | `events.rs`          | StateEvent variant            |
| `NodeDiscovered` variant          | `events.rs`          | StateEvent variant            |
| `list_nodes_manager.rs`           | entire file          | ListNodesManager task         |
| `list_nodes_event_tx`             | `coordinator.rs`     | Forwarding channel            |
| `ListNodesSettings`               | `config.rs`          | Configuration struct          |
| `list_nodes_loading_timeout_secs` | `ActorConfig`        | Config field                  |

**Keep `ListNodes` service client creation ability** — it's useful for
on-demand debugging (web UI "refresh" button) even if not used for polling.

### Phase 19.5d: Clean up legacy code ✅

**Goal**: Remove dead paths and simplify the select loop.

- Remove `response_tx` from `LoadRequest` (line 1702 comment: "Legacy: Send
  response via channel for old composable node actors, will be ignored").
- Remove `LoadRequest.response_tx` / `LoadNodeResponse` channel if no
  consumers remain.
- Remove the 200ms warmup sleep once the service response is no longer the
  primary confirmation mechanism (the event will confirm independently).
- Collapse `current_load` / `current_unload` from `Option<CurrentLoad>` to
  just tracking the in-flight composable name (the JoinHandle for the service
  task can still be kept for cancellation).

### File changes summary

| Phase | Files modified                                                                            | Lines (est.) |
|-------|-------------------------------------------------------------------------------------------|--------------|
| 19.5a | `container_actor.rs`, `events.rs`                                                         | +80, -10     |
| 19.5b | `container_actor.rs`                                                                      | +10, -5      |
| 19.5c | `container_actor.rs`, `events.rs`, `coordinator.rs`, `list_nodes_manager.rs`, `config.rs` | +0, -350     |
| 19.5d | `container_actor.rs`, `container_control.rs`                                              | +5, -60      |

Net: **~350 lines removed** from the codebase.

### Testing

Existing tests cover the affected paths:

- `tests/tests/container_events.rs` — dump parity, live launch, crash
  detection (6 tests)
- `tests/tests/sequential_loading.rs` — 5 composable nodes loaded
  sequentially (3 tests + 1 launch)
- `tests/tests/concurrent_loading.rs` — concurrent container loading
- Autoware smoke test — 49 processes, 15 containers, 54 composable nodes

New tests for Phase 19.5:

| Test                            | What                                                                                             |
|---------------------------------|--------------------------------------------------------------------------------------------------|
| `test_event_driven_load`        | Launch container, verify LOADED event drives state (not service response)                        |
| `test_service_timeout_fallback` | Launch with stock `rclcpp_components` container (no events), verify service response still works |

### Risks

| Risk                                             | Mitigation                                                                                                                                                                         |
|--------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Stock containers don't publish ComponentEvents   | Phase 19.5a fallback: service response path remains active when subscription is None                                                                                               |
| Event arrives before service response processing | Check `entry.state != Loading` before service-response transition                                                                                                                  |
| Event lost due to QoS overflow                   | Depth 100 + reliable QoS; 100 is generous for any realistic launch                                                                                                                 |
| Autoware uses stock containers, not ours         | For Autoware containers launched via stock `rclcpp_components`, play_launch uses its own container binary (`play_launch_container`) anyway — the `exec` is rewritten during replay |

---

## Phase 19.8: Parallel Non-Blocking Node Loading

**Status**: Planned
**Dependencies**: Phase 19.5 complete (ComponentEvent as primary source of truth)

### Motivation

In Autoware, some composable nodes trigger TensorRT compilation in their
constructor, blocking for **minutes**. The current architecture serializes
loading at three levels:

1. **C++ `on_load_node()`**: Synchronous service callback — `dlopen()` →
   constructor → `clone()` → respond. A blocking constructor freezes the
   executor, making the container unresponsive to all other service calls
   (LoadNode, UnloadNode, ListNodes).
2. **C++ callback group**: Services use the default
   `MutuallyExclusiveCallbackGroup`, serializing callbacks even with
   `MultiThreadedExecutor`.
3. **Rust `current_load`**: `Option<CurrentLoad>` allows at most one
   in-flight LoadNode request. `start_next_load()` returns immediately
   if `current_load.is_some()`.

With 54 composable nodes across 15 containers in Autoware, a single slow
TensorRT node blocks all subsequent loads into that container.

### Design

**Key insight**: `LoadNode` service clients (launch_ros, ros2 CLI,
play_launch) do **not** expect the node to be operational after the
response. They only use the response for:

- `unique_id` — for future UnloadNode calls and display
- `full_node_name` — for logging and duplicate name detection
- `success` — pass/fail

Node operational readiness is handled asynchronously through DDS discovery.
This means we can **decouple the service response from node construction**.

**Two-phase loading**:

```
Phase 1 (synchronous, microseconds):
  on_load_node() → validate plugin → pre-assign unique_id → respond success

Phase 2 (async worker thread, seconds to minutes):
  worker → create_node_instance() → add_node_to_executor() → ComponentEvent
```

The `unique_id` serves as the correlation token between the immediate
service response and the deferred ComponentEvent.

### Phase 19.8a: C++ Non-Blocking on_load_node

**Goal**: Return from `on_load_node()` immediately after plugin validation,
deferring the actual node construction and `clone()` to a worker thread.

**Override `on_load_node()` in `CloneIsolatedComponentManager`**:

```cpp
void CloneIsolatedComponentManager::on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response)
{
  (void)request_header;

  // Phase 1: Synchronous validation + immediate response
  try {
    auto resources = get_component_resources(request->package_name);
    auto factory = /* find matching factory (same logic as parent) */;
    if (!factory) {
      response->success = false;
      response->error_message = "Plugin not found";
      return;
    }

    auto options = create_node_options(request);
    auto node_id = unique_id_++;

    // Compute full_node_name from request (before actual construction)
    std::string full_node_name;
    if (!request->node_namespace.empty() && !request->node_name.empty()) {
      full_node_name = request->node_namespace + "/" + request->node_name;
    } else if (!request->node_name.empty()) {
      full_node_name = "/" + request->node_name;
    }

    response->success = true;
    response->unique_id = node_id;
    response->full_node_name = full_node_name;

    // Phase 2: Async construction on worker thread
    auto work = [this, factory, options, node_id, request]() {
      try {
        node_wrappers_[node_id] = factory->create_node_instance(options);
        add_node_to_executor(node_id);

        // Publish LOADED event with actual full_node_name
        auto event = play_launch_msgs::msg::ComponentEvent();
        event.stamp = now();
        event.event_type = CE::LOADED;
        event.unique_id = node_id;
        event.full_node_name =
            node_wrappers_[node_id].get_node_base_interface()
                ->get_fully_qualified_name();
        event.package_name = request->package_name;
        event.plugin_name = request->plugin_name;
        event_pub_->publish(event);
      } catch (const std::exception & ex) {
        auto event = play_launch_msgs::msg::ComponentEvent();
        event.stamp = now();
        event.event_type = CE::LOAD_FAILED;
        event.unique_id = node_id;
        event.error_message = ex.what();
        event.package_name = request->package_name;
        event.plugin_name = request->plugin_name;
        event_pub_->publish(event);
      }
    };
    worker_pool_.submit(std::move(work));

  } catch (const ComponentManagerException & ex) {
    response->success = false;
    response->error_message = ex.what();
  }
}
```

**Worker thread pool**: Simple pool (2–4 threads) for concurrent node
construction. Could use `std::thread` with a task queue, or a lightweight
thread pool library.

**Thread safety**: `node_wrappers_`, `loaders_` are accessed from both
the executor thread (service callbacks) and worker threads. Need mutex
protection. `unique_id_` is only incremented in service callbacks
(executor thread), so no contention.

**Why only in `CloneIsolatedComponentManager`**: Non-isolated
`ObservableComponentManager` keeps synchronous loading — its event
publishing is tightly coupled to the service response in the same
callback. Changing it would break stock container compatibility.

**Work items**:

- [ ] Add worker thread pool to `CloneIsolatedComponentManager` (start in
  constructor, join in destructor)
- [ ] Override `on_load_node()` — validate plugin synchronously, respond
  immediately, spawn construction work
- [ ] Add mutex on `node_wrappers_` and `loaders_` for thread safety
- [ ] Worker publishes `ComponentEvent::LOADED` or `LOAD_FAILED` after
  construction
- [ ] Compute approximate `full_node_name` from request fields for
  immediate response
- [ ] Override `on_list_nodes()` to exclude not-yet-constructed nodes
  (entries in `node_wrappers_` not yet populated by worker)

**Key technical decisions**:

- **Plugin validation is synchronous**: `get_component_resources()` (ament
  index lookup) and `create_component_factory()` (dlopen, cached after
  first load) are fast (milliseconds). This catches wrong
  package/plugin errors immediately in the service response.
- **Constructor is async**: `create_node_instance()` is the slow path
  (TensorRT, large model loading). This runs on the worker thread.
- **`unique_id` pre-assignment**: Counter is incremented in the service
  callback (single-threaded executor context). Worker receives the
  assigned ID; no contention.
- **`full_node_name` approximation**: Response carries the expected name
  from request fields. The ComponentEvent carries the actual name from
  `get_fully_qualified_name()`. launch_ros falls back to
  `request.node_name` if response's name is empty. play_launch uses
  ComponentEvent as primary source (Phase 19.5).
- **`on_unload_node()` with pending loads**: If the client tries to unload
  a node that's still being constructed (unique_id assigned but not yet
  in `node_wrappers_`), return an error. The client shouldn't unload
  a node before LOADED event confirms it.

### Phase 19.8b: Rust Parallel Load Dispatch

**Goal**: Remove the `current_load` serialization in `container_actor.rs`.
Allow multiple concurrent LoadNode service calls and use `unique_id` from
the immediate response to correlate with ComponentEvent.

**Replace single in-flight load with concurrent tracking**:

```rust
// Before: at most one load in flight
current_load: Option<CurrentLoad>,

// After: multiple concurrent loads, keyed by composable_name
in_flight_loads: HashMap<String, InFlightLoad>,
```

```rust
struct InFlightLoad {
    request: LoadRequest,
    start_time: Instant,
    task: JoinHandle<Result<LoadNodeResponse>>,
    /// Pre-assigned unique_id from immediate service response (None until
    /// the service task completes and stores it)
    unique_id: Option<u64>,
}
```

**New loading flow**:

1. `pending_loads` queue → `start_all_pending_loads()`: dispatch ALL
   pending loads concurrently (no serialization gate)
2. Each service call returns immediately (microseconds) with
   `success=true` + `unique_id`
3. Service response handler stores `unique_id` in `InFlightLoad` and in
   `ComposableNodeEntry`, but does NOT transition to `Loaded` — state
   stays `Loading`
4. `ComponentEvent::LOADED` arrives with matching `unique_id` →
   transitions from `Loading` to `Loaded`
5. `ComponentEvent::LOAD_FAILED` arrives → transitions to `Failed`

**Service response handling changes**:

```rust
// Before: service response transitions to Loaded
Ok(response) if response.success => {
    if matches!(entry.state, ComposableState::Loading { .. }) {
        entry.state = ComposableState::Loaded { unique_id };
    }
}

// After: service response only stores unique_id, no state transition
Ok(response) if response.success => {
    entry.unique_id = Some(response.unique_id);
    // State stays Loading — wait for ComponentEvent
}
```

**ComponentEvent matching**:

```rust
// Before: match against single current_load
let composable_name = match &self.current_load {
    Some(load) => load.request.composable_name.clone(),
    None => return,
};

// After: match by unique_id across all composable entries
CE::LOADED => {
    let entry = self.composable_nodes.iter_mut()
        .find(|(_, e)| e.unique_id == Some(event.unique_id));
    if let Some((name, entry)) = entry {
        if matches!(entry.state, ComposableState::Loading { .. }) {
            entry.state = ComposableState::Loaded { unique_id: event.unique_id };
        }
    }
}
```

**Poll multiple in-flight tasks**: Replace the single `current_load` task
poll in `select!` with `FuturesUnordered` or iterate over `in_flight_loads`:

```rust
// Before: poll single task
result = async {
    match &mut self.current_load {
        Some(load) => (&mut load.task).await,
        None => std::future::pending().await,
    }
}, if self.current_load.is_some() => { ... }

// After: poll any completed task from in_flight_loads
result = self.in_flight_futures.next(),
    if !self.in_flight_futures.is_empty() => { ... }
```

**Backward compatibility with stock containers**: When the container is a
stock `rclcpp_components` container (no ComponentEvent subscription), the
service response IS the completion signal. Detect this by checking if
`component_event_sub` is `None` and fall back to service-response-driven
state transitions (existing behavior).

**Work items**:

- [ ] Replace `current_load: Option<CurrentLoad>` with
  `in_flight_loads: HashMap<String, InFlightLoad>`
- [ ] Replace `start_next_load()` with `start_all_pending_loads()` —
  dispatch all pending loads concurrently
- [ ] Service response handler: store `unique_id` in composable entry,
  do NOT transition to `Loaded`
- [ ] ComponentEvent LOADED/LOAD_FAILED: match by `unique_id` instead
  of `current_load` composable name
- [ ] Use `FuturesUnordered` to poll multiple in-flight service tasks
- [ ] Remove `start_next_load()` call after each task completion (no
  longer needed — all loads started immediately)
- [ ] Backward compatibility: fall back to service-response transitions
  when ComponentEvent subscription is absent
- [ ] Update `pending_loads: VecDeque<LoadRequest>` flush to also cancel
  in-flight tasks on container shutdown

### Phase 19.8c: Integration Tests

**Goal**: Verify parallel loading works correctly with slow-constructing
nodes.

**Create `slow_loader` test component** (`src/play_launch_container/test/`):

A composable node whose constructor sleeps for a configurable duration
(simulating TensorRT compilation). Use a ROS parameter to control the
delay:

```cpp
class SlowLoader : public rclcpp::Node {
public:
  SlowLoader(const rclcpp::NodeOptions & options) : Node("slow_loader", options) {
    auto delay_ms = declare_parameter("load_delay_ms", 5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    // Publish a "ready" message after construction
    pub_ = create_publisher<std_msgs::msg::String>("~/ready", 10);
    timer_ = create_wall_timer(100ms, [this]() {
      pub_->publish(std_msgs::msg::String().set__data("ready"));
    });
  }
};
```

**Test scenarios**:

| Test                                    | Description                                                                                                                                |
|-----------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| `test_parallel_load_2_slow_nodes`       | Load 2 SlowLoader nodes (3s each) into isolated container. Verify both LOADED events arrive within ~4s (parallel), not ~7s (sequential).   |
| `test_container_responsive_during_load` | Load 1 SlowLoader (5s), immediately load 1 fast node (Talker). Verify Talker's LOADED event arrives within ~1s, not blocked by SlowLoader. |
| `test_load_failure_during_parallel`     | Load 1 SlowLoader + 1 invalid plugin concurrently. Verify LOAD_FAILED for invalid plugin arrives immediately, SlowLoader still completes.  |
| `test_list_nodes_excludes_pending`      | Load 1 SlowLoader, immediately call ListNodes. Verify response doesn't include the still-constructing node.                                |
| `test_unload_during_construction`       | Load SlowLoader (5s), attempt UnloadNode with its pre-assigned unique_id during construction. Verify error response.                       |

**Test fixture**: `tests/fixtures/parallel_loading/` with launch files
using `play_launch_container` with `--isolated`.

**Work items**:

- [ ] Create `SlowLoader` composable node component
- [ ] Register `SlowLoader` in `play_launch_container` CMakeLists
- [ ] Create test launch files in `tests/fixtures/parallel_loading/`
- [ ] Implement 5 integration tests
- [ ] Verify all existing tests still pass

### File Changes Summary

| Phase | Files modified                                  | Lines (est.) |
|-------|-------------------------------------------------|--------------|
| 19.8a | `clone_isolated_component_manager.{hpp,cpp}`    | +150, -5     |
| 19.8b | `container_actor.rs`, `container_control.rs`    | +80, -60     |
| 19.8c | New: `slow_loader.cpp`, launch files, test file | +250         |

### Risks

| Risk                                                                           | Severity | Mitigation                                                                                              |
|--------------------------------------------------------------------------------|----------|---------------------------------------------------------------------------------------------------------|
| Worker thread crashes during construction (shared addr space)                  | High     | Same risk as current clone(CLONE_VM) — accept per earlier discussion                                    |
| Race between worker `node_wrappers_` write and executor `on_list_nodes()` read | Medium   | Mutex on `node_wrappers_`; `on_list_nodes` only returns fully-constructed entries                       |
| `dlopen` not thread-safe for same library                                      | Low      | `loaders_` map caches `ClassLoader` per library path; protect with mutex to avoid double-dlopen         |
| `unique_id` overflow with pre-assignment + failure                             | None     | Failed loads consume a `unique_id` — 2^64 IDs still effectively infinite                                |
| Approximate `full_node_name` differs from actual                               | Low      | launch_ros falls back to `request.node_name`; play_launch uses ComponentEvent (actual name)             |
| Stock containers (non-isolated) break                                          | None     | Only `CloneIsolatedComponentManager` overrides `on_load_node()`; `ObservableComponentManager` unchanged |

---

## Phase 19.6: Per-Node cgroups (Optional)

**Status**: Planned

Assign CPU cgroup limits to each composable node at spawn time.

### Work Items

- [ ] Create cgroup hierarchy under `/sys/fs/cgroup/play_launch/<container>/<node>/`
- [ ] Write `cpu.max` and `cpuset.cpus` from config
- [ ] Open cgroup dir fd and pass to clone via `CLONE_INTO_CGROUP` (requires clone3)
- [ ] Clean up cgroup dirs on container shutdown
- [ ] Add per-node cgroup config to YAML schema

### Passing Criteria

- [ ] Cgroup dirs created on container start
- [ ] Children born into their cgroup (no race window)
- [ ] CPU limits enforced (verify with stress test)
- [ ] Cgroup dirs cleaned up on shutdown

---

## Phase 19.7: MPK Memory Domains (Optional, Experimental)

**Status**: Planned

Use Intel Memory Protection Keys to isolate each node's heap.

### Work Items

- [ ] Detect MPK support at runtime (`pku` in `/proc/cpuinfo`)
- [ ] Allocate per-node pkeys via `pkey_alloc()`
- [ ] Create jemalloc arenas per node with `pkey_mprotect()`
- [ ] Set PKRU in child before `exec->spin()` to restrict memory access
- [ ] Graceful fallback when MPK not available

### Passing Criteria

- [ ] MPK detected at runtime (graceful fallback if unsupported)
- [ ] Per-node heap arenas created with separate pkeys
- [ ] Cross-domain write causes SIGSEGV (not silent corruption)
- [ ] Performance overhead < 1%

---

## Risks and Mitigations

| Risk                                       | Severity          | Mitigation                                                     |
|--------------------------------------------|-------------------|----------------------------------------------------------------|
| glibc TLS sharing (tcache double-free)     | Critical (solved) | `_dl_allocate_tls()` + `CLONE_SETTLS` + TID offset discovery   |
| Shared memory corruption after child crash | Critical          | MPK (19.7) + jemalloc arenas limit blast radius                |
| rclcpp mutex poisoning on child death      | Critical          | Children crash outside lock-held windows; watchdog as backstop |
| glibc malloc lock poisoning                | Critical          | Use jemalloc with per-node arenas                              |
| DDS thread ownership confusion             | Medium            | Nodes constructed in parent; DDS threads stay in parent        |
| Leaked shared_ptr refcounts                | Low               | Parent holds authoritative refs; bounded leak per restart      |

## References

- `docs/container-isolation-design.md` — background, rationale, Linux isolation analysis
- `docs/clone-vm-container-design.md` — detailed implementation design
- `external/rclcpp/rclcpp_components/include/rclcpp_components/component_manager_isolated.hpp` — upstream subclass pattern
- [clone(2)](https://man7.org/linux/man-pages/man2/clone.2.html)
- [pidfd_open(2)](https://man7.org/linux/man-pages/man2/pidfd_open.2.html)
- [Robust futexes — kernel docs](https://docs.kernel.org/locking/robust-futexes.html)
- [Memory Protection Keys — kernel docs](https://docs.kernel.org/core-api/protection-keys.html)
