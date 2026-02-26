> **Archived**: This describes the clone(CLONE_VM) approach, replaced by fork()+exec()
> in Phase 19. Kept for historical reference.

# Design: clone(CLONE_VM) Container with Process-Level Node Isolation

Custom container that spawns each composable node as a separate Linux process
(own PID, own signal disposition) while sharing the address space for zero-copy
intra-process communication.

## The Idea

Instead of using `std::thread` for per-node executor isolation (what
`ComponentManagerIsolated` does), use `clone(CLONE_VM)` without `CLONE_THREAD`.
Each composable node gets its own PID but shares the container's virtual memory.

```
Container process (PID 1000)
├── manages: LoadNode service, ListNodes service, node_wrappers_ map
├── shares address space with all children
│
├── Child PID 1001: node A executor spin loop
│   ├── own PID → own /proc entry, killable, cgroup-assignable
│   ├── own signal disposition → SIGSEGV kills only this child
│   └── shared memory → zero-copy pub/sub via IntraProcessManager
│
├── Child PID 1002: node B executor spin loop
│   └── (same properties)
│
└── Child PID 1003: node C executor spin loop
    └── (same properties)
```

## What This Gives Us

| Property | std::thread (current) | clone(CLONE_VM) |
|---|---|---|
| Zero-copy intra-process | Yes | **Yes** (shared address space) |
| SIGSEGV kills only one node | No (kills process) | **Yes** (separate thread group) |
| SIGABRT kills only one node | No | **Yes** |
| Separate PID per node | No | **Yes** |
| `/proc/<pid>/` per node | No | **Yes** |
| `waitpid()` for death detection | No | **Yes** |
| cgroups CPU per node | Needs TID mapping | **Yes** (via PID) |
| `kill(pid, SIGTERM)` per node | No | **Yes** |
| `CLONE_INTO_CGROUP` at spawn | No | **Yes** (clone3) |
| `CLONE_PIDFD` for monitoring | No | **Yes** (clone3) |
| Memory limits per node | No | No (shared address space) |
| OOM per node | No | No |

## Architecture

### Class Hierarchy

```
rclcpp_components::ComponentManager         (upstream: load/unload services)
  └── ObservableComponentManager            (ours: event publishing)
        └── CloneIsolatedComponentManager   (ours: clone(CLONE_VM) isolation)
```

`CloneIsolatedComponentManager` subclasses `ObservableComponentManager` to
inherit event publishing. It overrides `add_node_to_executor()` and
`remove_node_from_executor()` (same pattern as upstream
`ComponentManagerIsolated`):

```
ObservableComponentManager (base)
├── on_load_node()          — publishes LOADED/LOAD_FAILED events
├── on_unload_node()        — publishes UNLOADED events
├── event_pub_              — inherited by CloneIsolatedComponentManager

CloneIsolatedComponentManager (new)
├── add_node_to_executor()  — creates dedicated executor + clone(CLONE_VM)
├── remove_node_from_executor() — kill(child_pid) + waitpid()
├── monitor_children()      — epoll on pidfds for death detection
└── (death events published via inherited event_pub_)
```

### Consolidated Binary

All container modes run from a single `component_container` executable with
CLI flags:

```
component_container                              # default: ST executor
component_container --use_multi_threaded_executor # MT executor
component_container --isolated                   # clone(CLONE_VM) per-node
component_container --isolated --use_multi_threaded_executor
```

### Node Loading Flow

```
LoadNode request arrives
│
├─1. Parent (on_load_node, inherited from ObservableComponentManager):
│   ├── dlopen() shared library (into shared address space)
│   ├── class_loader::createInstance<NodeFactory>()
│   ├── factory->create_node_instance(options)  ← node is constructed HERE
│   ├── node_wrappers_[id] = wrapper
│   ├── add_node_to_executor(id)
│   └── publish LOADED or LOAD_FAILED event
│
├─2. add_node_to_executor (our override):
│   ├── exec = make_shared<SingleThreadedExecutor>()
│   ├── exec->add_node(wrapper.get_node_base_interface())
│   ├── allocate child stack via mmap(MAP_ANONYMOUS | MAP_STACK)
│   ├── clone3(CLONE_VM | CLONE_PIDFD | CLONE_INTO_CGROUP)
│   │   └── child: exec->spin(); _exit(0)
│   └── store {child_pid, pidfd, executor, stack_ptr} in child_map_
│
└─3. Response: success, full_node_name, unique_id
```

### Node Unloading Flow

```
UnloadNode request (unique_id)
│
├─1. remove_node_from_executor (our override):
│   ├── Look up child in child_map_
│   ├── exec->cancel()          ← sets spinning=false, child's spin() returns
│   ├── kill(child_pid, SIGTERM) ← if cancel wasn't enough
│   ├── waitpid(child_pid)      ← reap child
│   ├── munmap(stack)           ← free child's stack
│   └── erase from child_map_
│
├─2. Parent (on_unload_node, inherited from ObservableComponentManager):
│   ├── exec->remove_node(...)
│   ├── node_wrappers_.erase(id)
│   └── publish UNLOADED event
```

### Child Death Detection

Use `clone3()` with `CLONE_PIDFD` for race-free monitoring:

```
Parent spawns monitor thread:
│
├── epoll_create1()
├── For each child:
│   └── epoll_ctl(ADD, child.pidfd, EPOLLIN)
│
└── Loop:
    ├── epoll_wait()  ← blocks until a child exits
    ├── siginfo_t info;
    │   waitid(P_PIDFD, pidfd, &info, WEXITED)
    ├── Determine which node died
    ├── Log crash info (info.si_status, info.si_code)
    ├── Clean up:
    │   ├── exec->cancel() (may be no-op if already stopped)
    │   ├── exec->remove_node(...)
    │   ├── munmap(child_stack)
    │   ├── close(pidfd)
    │   └── Publish LOAD_FAILED event via inherited event_pub_
    └── Optionally: reload the node (call on_load_node again)
```

## clone3() Flags

```c
struct clone_args args = {
    .flags = CLONE_VM           // Share address space (zero-copy)
           | CLONE_FS           // Share cwd, root, umask
           | CLONE_FILES        // Share file descriptor table
           | CLONE_PIDFD        // Get pidfd for race-free monitoring
           | CLONE_INTO_CGROUP  // Place child in specific cgroup at birth
           ,
    .pidfd = (uint64_t)&pidfd,
    .exit_signal = SIGCHLD,     // Notify parent on exit
    .stack = (uint64_t)stack_base,
    .stack_size = STACK_SIZE,
    .cgroup = cgroup_fd,        // Target cgroup for CPU isolation
};
```

**Flags NOT used:**
- `CLONE_THREAD` — we want separate thread groups (separate PID, own signals)
- `CLONE_SIGHAND` — we want separate signal handlers (child installs its own)
- `CLONE_SETTLS` — *(originally planned to omit; now used — see TLS section)*

**Adding `CLONE_CLEAR_SIGHAND`** (Linux 5.5+): Resets all child signal handlers
to SIG_DFL. SIGSEGV in the child causes immediate death (default behavior)
rather than running the parent's handler in the child's context.

## Known Risks and Mitigations

### Risk 1: Shared Memory Corruption (CRITICAL)

**Problem**: When a child crashes, the shared address space may be corrupted. The
wild pointer that caused the segfault may have already written garbage to heap
metadata, other nodes' data, or the executor's internal structures.

**Mitigation — Intel MPK (best effort)**:

Assign each node's heap allocations to a separate Memory Protection Key. Before
entering a node's callback, set PKRU to allow only that node's key. A stray
pointer from node A that lands in node B's heap triggers SIGSEGV *before*
corruption occurs.

```
Child 1 (node A): PKRU allows key 0 (shared) + key 1 (node A heap)
Child 2 (node B): PKRU allows key 0 (shared) + key 2 (node B heap)

Node A buffer overflow → hits key 2 page → SIGSEGV → child 1 dies
Node B's heap is untouched.
```

Limitations: 14 usable keys, shared memory (key 0) remains unprotected,
requires Intel Skylake+ or AMD Zen 3+, requires per-node jemalloc arenas.

**Mitigation — Heap allocator isolation (complementary)**:

Use jemalloc with per-node arenas. Even without MPK, a use-after-free in node
A's arena cannot return memory belonging to node B.

### Risk 2: Mutex Poisoning (CRITICAL)

**Problem**: If a child dies while holding a `std::mutex` (executor's `mutex_`,
IntraProcessManager's lock, glibc's malloc arena lock), that mutex is
permanently locked. Other threads/children deadlock.

**Mitigation — Robust mutexes for our code**:

For mutexes we control, use `pthread_mutex_t` with `PTHREAD_MUTEX_ROBUST`:

```c
pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
// After child death, next locker gets EOWNERDEAD:
int ret = pthread_mutex_lock(&mtx);
if (ret == EOWNERDEAD) {
    repair_shared_state();
    pthread_mutex_consistent(&mtx);
}
```

**rclcpp mutexes are NOT robust** — `std::mutex` in Executor,
IntraProcessManager, Context, and global logging are all non-robust. We cannot
change them without modifying rclcpp.

**Mitigation options**:
1. **Crash outside rclcpp code**: The child only runs `exec->spin()`. Crashes in
   user callback code (node logic) happen outside the executor lock. The
   dangerous window (`get_next_executable()`) is small.
2. **Watchdog timer**: If a child dies AND another child deadlocks (no progress
   for N seconds), restart the entire container.
3. **Accept the risk**: For non-safety-critical use, mutex poisoning is an
   acceptable degradation.

### Risk 3: glibc malloc Lock Poisoning (CRITICAL)

**Problem**: glibc's `malloc`/`free` use internal per-arena mutexes that are NOT
robust. A child dying inside `malloc()` poisons the arena lock.

**Mitigation**: Replace glibc's allocator with jemalloc (via `LD_PRELOAD`).
jemalloc uses per-thread caches and per-arena locks. Combined with per-node
arenas, each child's allocations go through its own arena. A poisoned arena
affects only the dead child.

### Risk 4: Leaked shared_ptr Refcounts (MEDIUM)

**Problem**: When a child dies, C++ destructors don't run. `shared_ptr` on the
child's stack has its refcount permanently elevated.

**Mitigation**: The parent holds the authoritative `shared_ptr` in
`node_wrappers_`. When it erases the entry, the refcount drops. Leaked refcounts
on the dead child's stack are in `munmap`'d memory. This is a bounded memory
leak, cleaned up when the container exits.

### Risk 5: DDS Thread Ownership (MEDIUM)

**Problem**: DDS threads spawned during node construction belong to the parent's
thread group, not the child's.

**Mitigation**: This is actually fine — DDS threads survive the child's death.
When the parent cleans up the node (erases from `node_wrappers_`), the
destructor properly shuts down DDS entities.

### Risk 6: Child Exit Path (LOW)

**Problem**: Clone children must exit cleanly without running `atexit` handlers
or flushing `stdio` buffers in the shared address space.

**Mitigation**: The child installs a graceful SIGTERM handler that calls
`executor->cancel()`, causing `spin()` to return. The clone function then
returns, which the kernel handles as `_exit(return_value)` — no atexit
handlers, no stdio flush. See [Child Signal Handling](#child-signal-handling)
for the full design.

### Risk 7: TLS and struct pthread Initialization (SOLVED)

**Problem**: `clone(CLONE_VM)` without `CLONE_SETTLS` shares the parent's TLS
pointer → glibc tcache double-free. Even with `CLONE_SETTLS`,
`_dl_allocate_tls(nullptr)` allocates TLS data (`.tdata`/`.tbss` sections) but
does NOT initialize the `struct pthread` header fields that `pthread_create` →
`start_thread` would normally set up. This causes SIGSEGV crashes in the clone
child when glibc functions access these uninitialized fields:

| Missing Field              | Crash Symptom                                                       |
|----------------------------|---------------------------------------------------------------------|
| `locale` pointer (NULL)    | `__printf_fp_l` SIGSEGV on `%f` float formatting                    |
| ctype tables (NULL)        | `isalpha()`/`rcl_validate_topic_name` SIGSEGV via `__ctype_b_loc()` |
| tcbhead_t.self/tcb (NULL)  | `THREAD_SELF` returns NULL → random SIGSEGV in any glibc call       |
| `multiple_threads` (0)     | Potential futex operation races                                     |
| Many struct pthread fields | `pthread_create` SIGSEGV (robust_list, stack info, etc.)            |

**Solution**: Five-part initialization in `child_executor_fn`, before any glibc
calls:

1. **TID**: Set manually at offset 720 (glibc 2.35 x86_64), discovered by
   scanning the parent's TLS at startup.
2. **tcbhead_t.self/tcb**: Set to FS register value (offsets 0x00, 0x10).
3. **multiple_threads**: Set to 1 (offset 0x18).
4. **Locale**: `uselocale(LC_GLOBAL_LOCALE)` → glibc falls back to global locale.
5. **Ctype tables**: `__ctype_init()` via `dlsym` → populates per-thread ctype
   pointers from the current locale (same as glibc's `start_thread()` does).

**Executor constraint**: Clone children always use `SingleThreadedExecutor`,
regardless of the container's `use_multi_threaded_` setting. `pthread_create()`
accesses too many struct pthread fields to safely initialize manually. Each node
already runs in its own isolated process, so multi-threading within each child
process adds complexity for no benefit.

See Phase 19.1 (initial TLS) and Phase 19.9 (struct pthread init) in the
roadmap for details.

### Risk 8: FastDDS SHM Segment Leakage (LOW)

**Problem**: FastRTPS (the DDS implementation in ROS2 Humble) creates shared
memory segments in `/dev/shm/fastrtps_*` for zero-copy transport. When a
process is killed with SIGKILL, `shm_unlink()` never runs, leaving orphaned
segments that accumulate and can eventually exhaust SHM ports or cause
DDS startup failures in subsequent processes.

**Mitigation**: The graceful SIGTERM handler (Risk 6) ensures children exit
through the normal return path, allowing FastRTPS cleanup to run. For
SIGKILL'd processes (crash tests, OOM kills), two additional mitigations exist:

1. **`FASTRTPS_DEFAULT_PROFILES_FILE`**: Point to a FastDDS XML profile that
   disables SHM transport (UDP-only). Used in integration tests.
2. **Manual cleanup**: `rm -f /dev/shm/fastrtps_*` between test runs.

See [FastDDS SHM Transport](#fastdds-shm-transport) for the full analysis.

## Child Signal Handling

Clone children need graceful signal handling for two reasons:

1. **DDS/SHM cleanup**: FastRTPS creates `/dev/shm/fastrtps_*` segments that
   must be unlinked by destructors. `SIG_DFL` for SIGTERM terminates the
   process immediately without running any C++ destructors.
2. **Parent cleanup flow**: `cleanup_child()` sends SIGTERM first, waits
   500ms, then SIGKILL. With graceful handling, children exit within
   milliseconds of receiving SIGTERM.

### Implementation

Each child stores its executor pointer in a `thread_local` variable (each
child has its own TLS via `CLONE_SETTLS`). The SIGTERM handler calls
`executor->cancel()`, which writes to the executor's interrupt guard condition
(a pipe write — async-signal-safe). This wakes up `spin()`, which returns.
The clone function then returns, and the kernel calls `_exit()`.

```cpp
// Thread-local: per-child despite shared address space (CLONE_SETTLS)
static thread_local rclcpp::Executor * g_child_executor = nullptr;

static void child_sigterm_handler(int sig)
{
  (void)sig;
  if (g_child_executor) {
    g_child_executor->cancel();  // pipe write, async-signal-safe
  }
}

static int child_executor_fn(void * arg)
{
  auto * boot = static_cast<ChildBootstrap *>(arg);

  // ... TID setup, signal handler reset ...

  // Install graceful SIGTERM handler
  g_child_executor = boot->exec;
  struct sigaction sa{};
  sa.sa_handler = child_sigterm_handler;
  sigaction(SIGTERM, &sa, nullptr);

  boot->exec->spin();   // blocks until cancel() or shutdown
  return 0;             // kernel does _exit(0) — no atexit, no stdio flush
}
```

### Why not spin_some() / spin_once() polling?

Early iterations used a poll loop (`while (!flag) { spin_some(); usleep(1ms); }`)
or `spin_once(100ms)`. Both broke service callback latency:

- `spin_some()` processes available callbacks then returns immediately, requiring
  a sleep to avoid busy-wait. The sleep adds latency to incoming service calls.
- `spin_once(timeout)` processes only ONE callback per call. With multiple
  queued service requests, throughput drops dramatically.

`spin()` blocks efficiently inside `rcl_wait` (epoll) and processes all ready
callbacks. `cancel()` wakes it via the interrupt guard condition — the exact
mechanism ROS2's own signal handler uses.

### Shutdown sequence

When the container receives SIGTERM:

```
1. ROS2 signal handler fires → rclcpp::ok() = false
2. Main executor loop exits → rclcpp::shutdown()
3. ~CloneIsolatedComponentManager() runs:
   a. Stop worker threads (join)
   b. Stop monitor thread (eventfd wakeup + join)
   c. For each child:
      - executor->cancel()     ← redundant with SIGTERM handler, harmless
      - kill(child_pid, SIGTERM) ← handler calls cancel(), spin() returns
      - waitpid (up to 500ms)  ← child exits within milliseconds
      - SIGKILL if still alive ← fallback for stuck children
      - munmap(stack), close(pidfd), free(TLS)
```

The 500ms wait in `cleanup_child()` is generous — with the cancel-based
handler, children typically exit in <1ms after SIGTERM.

## FastDDS SHM Transport

FastRTPS (ROS2 Humble's DDS) uses shared memory segments in `/dev/shm/` for
high-throughput, zero-copy inter-process communication. Each DDS participant
creates multiple segments (`fastrtps_<hash>` and `fastrtps_<hash>_el`).

### The Problem

When a process exits **gracefully**, FastRTPS destructors call `shm_unlink()`
to remove these segments. When a process is killed with **SIGKILL**:

- Destructors never run
- `shm_unlink()` is never called
- Segments persist in `/dev/shm/` as orphans
- The kernel keeps the segment data alive as long as any fd references it,
  but the **name** persists indefinitely since no process calls `shm_unlink()`

Over time, orphaned segments accumulate. FastRTPS has a finite number of SHM
"ports" (2^15 per domain). Exhaustion causes:

- DDS participant creation failures (new containers can't start)
- Lost DDS messages (ComponentEvent LOADED never arrives)
- Service call hangs (LoadNode/UnloadNode never get responses)

### Sources of SIGKILL'd Processes

| Source | Description |
|--------|-------------|
| `test_crash_detection` | Explicitly SIGKILLs a clone child to test crash monitoring |
| `ManagedProcess::drop` | Sends SIGTERM, waits 2s, then SIGKILL to the process group |
| `PR_SET_PDEATHSIG` | Kernel sends SIGKILL to child if parent dies unexpectedly |
| OOM killer | Kernel sends SIGKILL to processes exceeding memory limits |

### Mitigations

**1. Graceful SIGTERM handler (primary)**

The clone child's SIGTERM handler (see above) ensures `spin()` returns and
the clone function exits normally. This allows the parent's destructor to
clean up DDS entities via `node_wrappers_.erase()`, which triggers FastRTPS
destructors and `shm_unlink()`.

**Effectiveness**: Eliminates SHM leakage for all normal shutdown paths.
Does not help SIGKILL'd processes (by definition).

**2. FastDDS XML profile — disable SHM transport (tests)**

For integration tests, set `FASTRTPS_DEFAULT_PROFILES_FILE` to a profile
that disables SHM transport entirely:

```xml
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>udp_transport</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

This profile is at `tests/fixtures/fastdds_no_shm.xml` and is set by the
test harness in `fixtures::play_launch_cmd()`.

**Trade-off**: UDP transport has higher latency than SHM for large messages,
but is functionally equivalent for testing purposes.

**3. Manual cleanup**

```bash
rm -f /dev/shm/fastrtps_*
```

### Loading Timeout Fallback

Even with SHM disabled, DDS can occasionally lose messages under load. The
Rust `container_actor` implements a fallback timer: if a composable node has
been in `Loading` state for >10 seconds with a known `unique_id` (LoadNode
service succeeded), it is promoted to `Loaded` and a warning is logged.

This prevents nodes from being stuck in `Loading` state permanently when
`ComponentEvent LOADED` is lost.

## Per-Node cgroups via CLONE_INTO_CGROUP

```bash
# Pre-create cgroup hierarchy
mkdir /sys/fs/cgroup/ros_container
echo "+cpu +cpuset" > /sys/fs/cgroup/ros_container/cgroup.subtree_control

# For each node:
mkdir /sys/fs/cgroup/ros_container/node_perception
echo "50000 100000" > /sys/fs/cgroup/ros_container/node_perception/cpu.max
echo "0-3" > /sys/fs/cgroup/ros_container/node_perception/cpuset.cpus
```

Then in `clone3()`:

```c
int cgroup_fd = open("/sys/fs/cgroup/ros_container/node_perception", O_RDONLY);
args.flags |= CLONE_INTO_CGROUP;
args.cgroup = cgroup_fd;
```

The child is born directly in its cgroup — no race window.

## Limitations to Accept

1. **Memory corruption is contained, not prevented.** MPK protects heap domains
   but shared state (rclcpp, DDS, libc) remains vulnerable.

2. **Mutex poisoning is partially mitigated.** Our mutexes can be robust. rclcpp
   mutexes cannot. A child dying inside rclcpp code (rare) causes unrecoverable
   deadlock.

3. **This is experimental.** `clone(CLONE_VM)` without `CLONE_THREAD` is valid
   but unusual. C++ runtimes and DDS libraries were not designed with this model
   in mind. Edge cases are likely.

4. **Not a substitute for process isolation.** For safety-critical deployments,
   separate processes remain the only reliable isolation.

## References

- [clone(2)](https://man7.org/linux/man-pages/man2/clone.2.html)
- [clone3(2)](https://man.archlinux.org/man/clone3.2.en)
- [Robust futexes — kernel docs](https://docs.kernel.org/locking/robust-futexes.html)
- [pthread_mutexattr_setrobust(3)](https://man7.org/linux/man-pages/man3/pthread_mutexattr_setrobust.3.html)
- [CLONE_INTO_CGROUP — LWN](https://lwn.net/Articles/807882/)
- [glibc posix_spawn uses CLONE_VM — LWN](https://lwn.net/Articles/908915/)
- [Memory Protection Keys — kernel docs](https://docs.kernel.org/core-api/protection-keys.html)
- `external/rclcpp/rclcpp_components/include/rclcpp_components/component_manager_isolated.hpp` — subclass pattern
- `external/rclcpp/rclcpp/src/rclcpp/executor.cpp` — mutex and add_node thread safety
