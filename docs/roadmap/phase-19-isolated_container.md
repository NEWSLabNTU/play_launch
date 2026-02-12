# Phase 19: Clone-Isolated Component Manager

**Status**: 🔧 In Progress (19.0 + 19.1 + 19.2 complete)
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
        │     └── 19.3 Integration tests  ⏳ next
        └── 19.4 play_launch integration
              ├── 19.5 cgroups            (optional)
              └── 19.6 MPK               (optional, experimental)
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

| File | Action |
|------|--------|
| `clone_isolated_component_manager.hpp` | New — class declaration + ChildInfo struct |
| `clone_isolated_component_manager.cpp` | New — clone(CLONE_VM) + TLS + TID implementation |
| `observable_component_manager.hpp` | Modified — `event_pub_` private → protected |
| `component_container.cpp` | Modified — `--isolated` flag + explicit shutdown ordering |
| `CMakeLists.txt` | Modified — added new source file |

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

| File | Action |
|------|--------|
| `play_launch_msgs/msg/ComponentEvent.msg` | Added `uint8 CRASHED=3` |
| `clone_isolated_component_manager.hpp` | Added monitor members (epoll_fd_, stop_fd_, thread, atomic) |
| `clone_isolated_component_manager.cpp` | Monitor implementation (constructor, destructor, loop, handler) |

### Passing Criteria

- [x] Monitor thread starts with container and stops on shutdown
- [x] Crash event published on `~/_container/component_events` topic
- [x] Resources cleaned up (stack freed, pidfd closed, TLS freed)
- [x] Container process itself survives child crash
- [x] Existing tests pass (326/326)

---

## Phase 19.3: Integration Tests

**Status**: Planned

Automated tests for crash isolation and normal operation.

### Work Items

- [ ] Create test workspace `tests/fixtures/crash_isolation/`
- [ ] Write `crash_node.cpp` — composable node that segfaults on timer
- [ ] Write `healthy_node.cpp` — composable node that publishes at 10 Hz
- [ ] Test 19.3.1: Normal load/unload cycle (load 3, unload 1, verify 2 remain)
- [ ] Test 19.3.2: Crash isolation (crash 1 of 3 nodes, verify others survive)
- [ ] Test 19.3.3: Intra-process communication (pub/sub between isolated nodes)
- [ ] Test 19.3.4: Per-node PID visibility (`kill(pid, 0)` checks)
- [ ] Integrate into `just test-all` via nextest

### Passing Criteria

- [ ] All 4 test cases pass
- [ ] Tests integrated into `just test-all`
- [ ] No orphan processes after test completion (ManagedProcess cleanup)
- [ ] Tests complete within 60s timeout

---

## Phase 19.4: play_launch Integration (Rust)

**Status**: Planned

Make `container_actor.rs` spawn isolated containers via config.

### Work Items

- [ ] Add `container_type` config field (`"standard"`, `"mt"`, `"isolated"`, `"isolated_mt"`)
- [ ] Update container spawning to pass `--isolated` / `--use_multi_threaded_executor` flags
- [ ] Handle `LOAD_FAILED` events from child crashes in event subscriber
- [ ] Add optional `auto_reload_on_crash` config (re-issue LoadNode on crash)
- [ ] Update config YAML schema and documentation

### Passing Criteria

- [ ] `play_launch replay` uses isolated container when configured
- [ ] Crashed composable nodes detected and reported in logs
- [ ] Optional auto-reload re-loads crashed nodes
- [ ] Default config (`"standard"`) unchanged — existing tests pass

---

## Phase 19.5: Per-Node cgroups (Optional)

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

## Phase 19.6: MPK Memory Domains (Optional, Experimental)

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

| Risk | Severity | Mitigation |
|------|----------|------------|
| glibc TLS sharing (tcache double-free) | Critical (solved) | `_dl_allocate_tls()` + `CLONE_SETTLS` + TID offset discovery |
| Shared memory corruption after child crash | Critical | MPK (19.6) + jemalloc arenas limit blast radius |
| rclcpp mutex poisoning on child death | Critical | Children crash outside lock-held windows; watchdog as backstop |
| glibc malloc lock poisoning | Critical | Use jemalloc with per-node arenas |
| DDS thread ownership confusion | Medium | Nodes constructed in parent; DDS threads stay in parent |
| Leaked shared_ptr refcounts | Low | Parent holds authoritative refs; bounded leak per restart |

## References

- `docs/container-isolation-design.md` — background, rationale, Linux isolation analysis
- `docs/clone-vm-container-design.md` — detailed implementation design
- `external/rclcpp/rclcpp_components/include/rclcpp_components/component_manager_isolated.hpp` — upstream subclass pattern
- [clone(2)](https://man7.org/linux/man-pages/man2/clone.2.html)
- [pidfd_open(2)](https://man7.org/linux/man-pages/man2/pidfd_open.2.html)
- [Robust futexes — kernel docs](https://docs.kernel.org/locking/robust-futexes.html)
- [Memory Protection Keys — kernel docs](https://docs.kernel.org/core-api/protection-keys.html)
