# Phase 19: Clone-Isolated Component Manager

**Status**: ⏳ Planned
**Priority**: High (crash isolation, per-node resource control)
**Dependencies**: Phase 18 complete. ObservableComponentManager merged.

## Overview

Replace `std::thread`-based executor isolation with `clone(CLONE_VM)`-based
process isolation. Each composable node gets its own Linux PID while sharing the
container's address space for zero-copy intra-process communication.

This gives: signal-level crash isolation, per-node PID, per-node cgroup CPU
control, `waitpid()` death detection, `pidfd` monitoring — all without
sacrificing the shared-memory IPC that containers exist for.

## Design References

- `docs/container-isolation-design.md` — background, rationale, Linux isolation analysis
- `docs/clone-vm-container-design.md` — detailed implementation design

## Phases

### Phase 19.0: Consolidate Container Executable

**Goal**: Merge the two separate main() files (`component_container.cpp` and
`component_container_mt.cpp`) into a single executable with runtime
configuration via CLI flags.

**Rationale**: Separate ST/MT binaries are not a ROS requirement — just
historical convention from `rclcpp_components`. Upstream already demonstrates
consolidation in `component_container_isolated.cpp`, which uses a single binary
with `--use_multi_threaded_executor`. Consolidating now simplifies Phase 19.1
(one binary to extend, not three) and Phase 19.4 (one executable to spawn).

#### Current State

```
src/play_launch_container/
├── include/.../observable_component_manager.hpp   # 1 class
├── src/observable_component_manager.cpp           # Event publishing logic
├── src/component_container.cpp                    # main() — SingleThreadedExecutor
└── src/component_container_mt.cpp                 # main() — MultiThreadedExecutor
```

The two main() files are nearly identical (6 vs 10 lines of logic). The only
difference is executor type and `thread_num` parameter handling.

#### Target State

```
src/play_launch_container/
├── include/.../observable_component_manager.hpp   # Unchanged
├── src/observable_component_manager.cpp           # Unchanged
└── src/component_container.cpp                    # Single main() with CLI flags
```

#### Implementation

Single `component_container.cpp` with CLI flag parsing (follows upstream
`component_container_isolated.cpp` pattern):

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Parse CLI args (same pattern as upstream component_container_isolated.cpp)
  bool use_multi_threaded = false;
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  for (auto & arg : args) {
    if (arg == "--use_multi_threaded_executor") {
      use_multi_threaded = true;
    }
  }

  // Create executor
  std::shared_ptr<rclcpp::Executor> exec;
  if (use_multi_threaded) {
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  } else {
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  // Create manager, handle thread_num parameter for MT mode
  auto node = std::make_shared<play_launch_container::ObservableComponentManager>(exec);
  if (use_multi_threaded && node->has_parameter("thread_num")) {
    auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
  }

  exec->add_node(node);
  exec->spin();
}
```

#### CMakeLists.txt Changes

```cmake
# Before: two executables
add_executable(component_container src/component_container.cpp)
add_executable(component_container_mt src/component_container_mt.cpp)

# After: one executable
add_executable(component_container src/component_container.cpp)
```

#### Files Changed

| File | Action |
|------|--------|
| `src/play_launch_container/src/component_container.cpp` | Rewrite: single main() with CLI flags |
| `src/play_launch_container/src/component_container_mt.cpp` | **Delete** |
| `src/play_launch_container/CMakeLists.txt` | Remove `component_container_mt` target |

#### play_launch Rust Side

Update `container_actor.rs` to pass `--use_multi_threaded_executor` when
spawning MT containers instead of using a different executable name.

#### Backward Compatibility

The `component_container` binary name is unchanged. Launch files that reference
`component_container` continue to work. Launch files that reference
`component_container_mt` need to switch to
`component_container --use_multi_threaded_executor`.

Since play_launch controls container spawning (not user launch files), this is
an internal change with no user-facing impact.

#### Success Criteria

- [ ] Single `component_container` executable builds
- [ ] Default mode: single-threaded executor (same as before)
- [ ] `--use_multi_threaded_executor` flag selects MT executor
- [ ] `thread_num` parameter works in MT mode
- [ ] Existing tests pass without changes
- [ ] `component_container_mt.cpp` deleted

---

### Phase 19.1: CloneIsolatedComponentManager (C++)

**Goal**: Subclass `ObservableComponentManager` that spawns each node's executor
loop in a `clone(CLONE_VM)` child process. Inherits event publishing from
`ObservableComponentManager`.

#### Files

| File | Description |
|------|-------------|
| `src/play_launch_container/include/play_launch_container/clone_isolated_component_manager.hpp` | Header |
| `src/play_launch_container/src/clone_isolated_component_manager.cpp` | Implementation |

No new executable — the consolidated `component_container` gains an `--isolated`
CLI flag:

```cpp
// In component_container.cpp (from Phase 19.0), add:
bool use_isolated = false;
for (auto & arg : args) {
  if (arg == "--use_multi_threaded_executor") use_multi_threaded = true;
  if (arg == "--isolated") use_isolated = true;
}

// Create appropriate manager
rclcpp::Node::SharedPtr node;
if (use_isolated) {
  node = std::make_shared<play_launch_container::CloneIsolatedComponentManager>(exec);
} else {
  node = std::make_shared<play_launch_container::ObservableComponentManager>(exec);
}
```

#### Class Hierarchy

```
rclcpp_components::ComponentManager         (upstream: load/unload services)
  └── ObservableComponentManager            (ours: event publishing)
        └── CloneIsolatedComponentManager   (ours: clone(CLONE_VM) isolation)
```

`CloneIsolatedComponentManager` inherits event publishing from
`ObservableComponentManager` automatically. No composition needed — the death
monitor (Phase 19.2) calls the inherited `event_pub_` publisher directly.

#### Implementation

Override two virtual methods (same pattern as upstream
`rclcpp_components::ComponentManagerIsolated`):

**`add_node_to_executor(uint64_t node_id)`**:
1. Create a dedicated `SingleThreadedExecutor`
2. Add the node to it (`exec->add_node(wrapper.get_node_base_interface())`)
3. `mmap(MAP_ANONYMOUS | MAP_STACK)` a child stack (default 8 MB)
4. `clone3()` with flags:
   - `CLONE_VM` — share address space
   - `CLONE_FILES` — share file descriptors
   - `CLONE_FS` — share cwd/root/umask
   - `CLONE_PIDFD` — get pidfd for monitoring
   - `CLONE_CLEAR_SIGHAND` — child gets default signal handlers
   - exit_signal = `SIGCHLD`
5. Child function: `exec->spin(); _exit(0);`
6. Store `{child_pid, pidfd, executor, stack_ptr, stack_size}` in `children_` map

**`remove_node_from_executor(uint64_t node_id)`**:
1. Look up child in `children_` map
2. `exec->cancel()` — child's `spin()` returns
3. `waitpid(child_pid, &status, WNOHANG)` — check if already exited
4. If still running: `kill(child_pid, SIGTERM)`, then `waitpid(child_pid, &status, 0)`
5. `munmap(stack_ptr, stack_size)` — free child stack
6. `close(pidfd)`
7. Erase from `children_` map

**Constructor**: Start a monitor thread (see 19.2).

#### Key Data Structures

```cpp
struct ChildInfo {
    pid_t pid;
    int pidfd;
    std::shared_ptr<rclcpp::Executor> executor;
    void* stack_ptr;
    size_t stack_size;
    uint64_t node_id;
    std::string node_name;
};

std::mutex children_mutex_;  // Robust mutex (pthread_mutex_t ROBUST)
std::map<uint64_t, ChildInfo> children_;  // node_id -> child info
```

#### clone3 Wrapper

Write a thin C wrapper since `clone3()` has no glibc wrapper:

```cpp
static pid_t clone3_vm(
    int (*fn)(void*), void* arg,
    void* stack, size_t stack_size,
    int* pidfd_out);
```

#### Success Criteria

- [ ] `component_container --isolated` starts successfully
- [ ] LoadNode service creates clone'd child
- [ ] Child spins executor and processes callbacks
- [ ] UnloadNode kills child cleanly
- [ ] Composable node pub/sub works (shared address space IPC)

---

### Phase 19.2: Child Death Monitor

**Goal**: Detect child crashes via `epoll` on pidfds, clean up resources, publish
events.

#### Implementation

A dedicated monitor thread (spawned in constructor):

```
monitor_thread:
    epoll_fd = epoll_create1(0)

    loop:
        // New children registered via a pipe/eventfd from add_node_to_executor
        register_new_pidfds_with_epoll()

        epoll_wait(epoll_fd, events, max_events, timeout_ms)

        for each ready pidfd:
            siginfo_t info;
            waitid(P_PIDFD, pidfd, &info, WEXITED | WNOHANG)

            if child exited:
                log: "Node {name} (PID {pid}) died: signal {sig}"

                // Cleanup in shared address space:
                exec->cancel()          // may be no-op
                exec->remove_node(...)  // remove from executor
                munmap(stack)
                close(pidfd)

                // Publish event via inherited event_pub_:
                publish_event(LOAD_FAILED, node_id, node_name, error_msg)

                // Erase from children_ map
                erase_child(node_id)

                // Leave node_wrappers_ entry for on_unload_node to clean up
                // (or auto-unload if configured)
```

#### Event Publishing

`CloneIsolatedComponentManager` inherits `event_pub_` from
`ObservableComponentManager`. The death monitor publishes `LOAD_FAILED` events
directly — no bridging or composition needed.

#### Success Criteria

- [ ] Monitor thread starts with container
- [ ] Child SIGSEGV detected within 100ms
- [ ] Crash event published on `~/_container/component_events` topic
- [ ] Other children unaffected (continue processing callbacks)
- [ ] Resources cleaned up (stack freed, pidfd closed)

---

### Phase 19.3: Integration Tests

**Goal**: Verify crash isolation and normal operation.

#### Test Cases

**19.3.1: Normal load/unload cycle**
- Load 3 composable nodes into isolated container
- Verify all 3 are running (ListNodes)
- Unload one, verify other 2 still running
- Unload remaining

**19.3.2: Crash isolation**
- Load 3 composable nodes
- Cause SIGSEGV in one node (test fixture node that dereferences null on timer)
- Verify: crashed node dies, other 2 continue processing
- Verify: LOAD_FAILED event published for crashed node
- Verify: container process itself survives

**19.3.3: Intra-process communication**
- Load a publisher and subscriber composable node with `use_intra_process_comms: true`
- Verify messages flow between them (zero-copy path via IntraProcessManager)
- Kill publisher node, verify subscriber detects no more messages

**19.3.4: Per-node PID visibility**
- Load nodes, verify each has its own PID in `/proc`
- Verify `kill(child_pid, 0)` returns 0 (child exists)
- After unload, verify `kill(child_pid, 0)` returns ESRCH (child gone)

#### Test Fixtures

New test workspace: `tests/fixtures/crash_isolation/`
- `crash_node.cpp` — composable node that segfaults on a timer callback
- `healthy_node.cpp` — composable node that publishes at 10 Hz
- `launch file` — loads crash_node + 2 healthy_nodes into isolated container

All tests use `component_container --isolated` (single binary).

#### Success Criteria

- [ ] All 4 test cases pass
- [ ] Tests integrated into `just test-all`
- [ ] No orphan processes after test completion (ManagedProcess cleanup)

---

### Phase 19.4: play_launch Integration (Rust)

**Goal**: Make `container_actor.rs` work with all container modes via the
consolidated single binary.

#### Changes

**Container spawning** (`container_actor.rs`):
- Always spawn `component_container` (single binary from our package)
- Pass CLI flags based on config:
  - `"standard"` → no extra flags (default: single-threaded)
  - `"mt"` → `--use_multi_threaded_executor`
  - `"isolated"` → `--isolated`
  - `"isolated_mt"` → `--isolated --use_multi_threaded_executor`

```rust
// Build command for container
let mut cmd = Command::new("component_container");
match config.container_type.as_str() {
    "mt" => { cmd.arg("--use_multi_threaded_executor"); }
    "isolated" => { cmd.arg("--isolated"); }
    "isolated_mt" => {
        cmd.arg("--isolated");
        cmd.arg("--use_multi_threaded_executor");
    }
    _ => {} // "standard" — no extra flags
}
```

**Event handling**:
- `LOAD_FAILED` events from child crashes already fit the existing event
  subscription in `container_actor.rs` — no change needed for detection
- Add optional auto-reload behavior: when a `LOAD_FAILED` event arrives due to
  child crash (not a load error), automatically re-issue the LoadNode request

**Config YAML**:

```yaml
composable_node_loading:
  container_type: isolated    # "standard" (default), "mt", "isolated", "isolated_mt"
  auto_reload_on_crash: true  # Re-load nodes that crash (isolated only)
```

#### Success Criteria

- [ ] `play_launch replay` can use isolated container via config
- [ ] Crashed composable nodes are detected and reported
- [ ] Optional auto-reload works
- [ ] Existing tests still pass with default container type

---

### Phase 19.5: Per-Node cgroups (Optional)

**Goal**: Assign CPU cgroup limits to each composable node at spawn time via
`CLONE_INTO_CGROUP`.

#### Design

Pre-create cgroup hierarchy:

```
/sys/fs/cgroup/play_launch/
├── <container_name>/
│   ├── <node_name_1>/   (cpu.max, cpuset.cpus)
│   ├── <node_name_2>/
│   └── ...
```

In `add_node_to_executor()`:
1. `mkdir` the node's cgroup dir
2. Write cpu.max, cpuset.cpus from config
3. `open()` the cgroup dir → fd
4. Pass fd as `clone_args.cgroup` with `CLONE_INTO_CGROUP`

#### Config YAML

```yaml
composable_node_loading:
  container_type: isolated
  cgroups:
    enabled: true
    base_path: /sys/fs/cgroup/play_launch
    defaults:
      cpu_max: "max 100000"     # No limit by default
      cpuset_cpus: "0-7"
    per_node:
      "pointcloud_filter":
        cpu_max: "50000 100000"  # 50% of one core
        cpuset_cpus: "0-3"
      "tensorrt_detector":
        cpu_max: "200000 100000" # 200% (2 cores)
        cpuset_cpus: "4-7"
```

#### Success Criteria

- [ ] Cgroup dirs created on container start
- [ ] Children born into their cgroup (no race window)
- [ ] CPU limits enforced (verify with `stress` in test node)
- [ ] Cgroup dirs cleaned up on container shutdown

---

### Phase 19.6: MPK Memory Domains (Optional, Experimental)

**Goal**: Use Intel Memory Protection Keys to isolate each node's heap, catching
cross-node memory corruption before it happens.

#### Design

1. At `dlopen()` time, allocate a `pkey` for the new plugin
2. Create a jemalloc arena for the plugin, backed by `mmap`'d memory
3. `pkey_mprotect()` the arena's pages with the plugin's key
4. Before `exec->spin()`, set PKRU to allow only that plugin's key + key 0
5. Child runs with restricted PKRU — stray pointers to other plugins' heaps
   trigger SIGSEGV before corruption occurs

#### Prerequisites

- Intel Skylake+ or AMD Zen 3+ (check `pku` in `/proc/cpuinfo`)
- jemalloc (replace glibc malloc via `LD_PRELOAD`)
- Up to 14 composable nodes per container (hardware limit)

#### Implementation

New files:
- `src/play_launch_container/include/play_launch_container/mpk_allocator.hpp`
- `src/play_launch_container/src/mpk_allocator.cpp`

Interface:
```cpp
class MpkAllocator {
public:
    static bool is_supported();  // checks /proc/cpuinfo for pku
    int create_domain();         // pkey_alloc()
    void* allocate(int domain, size_t size);  // from domain's arena
    void activate_domain(int domain);  // WRPKRU
    void deactivate_all();       // WRPKRU = deny all non-zero keys
};
```

#### Success Criteria

- [ ] MPK support detected at runtime (graceful fallback if unsupported)
- [ ] Per-node heap arenas created with separate pkeys
- [ ] Cross-domain write causes SIGSEGV (not silent corruption)
- [ ] Performance overhead < 1% (WRPKRU at callback boundaries only)

---

## Risks and Mitigations

| Risk | Severity | Mitigation |
|------|----------|------------|
| Shared memory corruption after child crash | Critical | MPK (19.6) + jemalloc arenas limit blast radius |
| rclcpp mutex poisoning on child death | Critical | Children crash outside lock-held windows in most cases; watchdog timer as backstop |
| glibc malloc lock poisoning | Critical | Use jemalloc with per-node arenas |
| DDS thread ownership confusion | Medium | Nodes constructed in parent; DDS threads stay in parent |
| Leaked shared_ptr refcounts | Low | Parent holds authoritative refs; bounded leak per restart |
| TLS confusion | Low | CLONE_CLEAR_SIGHAND + child uses only executor spin |

## Implementation Order

```
19.0 (Consolidate executable)          ← prerequisite: single binary
  │
  └── 19.1 (CloneIsolatedComponentManager)  ← core functionality
        │
        ├── 19.2 (Death monitor)            ← crash detection
        │     │
        │     └── 19.3 (Integration tests)  ← verify isolation works
        │
        └── 19.4 (play_launch integration)  ← user-facing
              │
              ├── 19.5 (cgroups)            ← optional resource control
              │
              └── 19.6 (MPK)               ← optional memory protection
```

19.0 is a quick prerequisite. 19.1 → 19.2 → 19.3 is the critical path.
19.4 makes it usable. 19.5 and 19.6 are independent enhancements.

## References

- `docs/container-isolation-design.md` — background, rationale, Linux isolation analysis
- `docs/clone-vm-container-design.md` — detailed implementation design
- `external/rclcpp/rclcpp_components/src/component_container_isolated.cpp` — upstream consolidation pattern
- `external/rclcpp/rclcpp_components/include/rclcpp_components/component_manager_isolated.hpp` — subclass pattern
- [clone3(2)](https://man.archlinux.org/man/clone3.2.en)
- [Robust futexes — kernel docs](https://docs.kernel.org/locking/robust-futexes.html)
- [Memory Protection Keys — kernel docs](https://docs.kernel.org/core-api/protection-keys.html)
