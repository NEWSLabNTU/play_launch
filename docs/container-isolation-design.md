# Container Isolation Design

Why ROS 2 component containers exist, their problems, the consequences of
splitting composable nodes into separate processes, what Linux can and cannot do
for isolation within a single process, and why we chose `clone(CLONE_VM)`.

## Why Containers Exist

A ROS 2 component container (`component_container`) is a process that
dynamically loads multiple node plugins into a single address space. The primary
motivation is **intra-process communication (IPC)**: bypassing DDS serialization
to share data between nodes via `shared_ptr` in the same process.

### Intra-Process Communication

When nodes share a process, the `IntraProcessManager` (a singleton per
`rclcpp::Context`) routes messages directly through memory:

```
Publisher::publish(unique_ptr<Msg>)
  → IntraProcessManager::do_intra_process_publish()
    → If 1 subscriber: promote unique_ptr → shared_ptr (0 copies)
    → If N subscribers: promote once, share N times (0 copies)
  → Subscription callback fires
```

This completely bypasses DDS — no serialization, no network stack. For a 2MB
point cloud, intra-process takes ~100-200us vs several milliseconds through DDS.

**The mechanism requires same-process:** `shared_ptr` is a process-local
reference count and cannot cross process boundaries. Nodes must share the same
`rclcpp::Context` (and therefore the same process).

### Secondary Benefits

| Benefit | Mechanism |
|---|---|
| Reduced DDS discovery | Fewer DDS participants (1 per container vs 1 per node) |
| Lower memory footprint | Shared libraries loaded once, single DDS participant |
| Thread efficiency | Shared executor pool vs per-process executor overhead |
| Faster startup | `dlopen()` into running container vs `fork()/exec()` + DDS init |

Benchmarks (arXiv:2305.09933): ~28% CPU reduction, ~33% memory reduction, ~50%
latency reduction for composable nodes vs separate processes.

## Problems with Containers

### 1. No Fault Isolation

All nodes share one process. A segfault, memory corruption, or unhandled
exception in any node kills the entire container and every node in it.

In Autoware with 15 containers and 54 composable nodes, a single node crash can
take down 3-10 nodes, cascading through the system.

`ComponentManagerIsolated` provides per-component executor threads but does
**not** help — all threads share the same address space.

### 2. Sequential Loading Blocks the Container

`on_load_node()` is a synchronous service callback. In single-threaded executor
mode, loading is strictly sequential. A slow constructor blocks everything:

- **TensorRT compilation**: 1-5 minutes on first run
- **Large parameter sets**: behavior_path_planner loads 800+ parameters
- **File I/O**: Large model files, YAML configs

While one node loads, no subsequent `LoadNode` requests, callbacks for
already-loaded nodes, or `ListNodes` queries can be processed.

### 3. No Per-Node Resource Limits

Linux cgroups operate at process granularity. You cannot set CPU, memory, or I/O
limits per composable node — only per container. A memory leak in one node
consumes the container's entire allocation.

### 4. Debugging Difficulty

- Core dumps contain all nodes' stacks interleaved
- Memory profilers report all nodes together
- Cannot attribute CPU/memory usage to individual nodes
- Logs are mixed from all nodes in the process

### 5. No Hot Reload

A misbehaving node cannot be restarted independently — unloading and reloading
it happens through the same executor that may already be stuck.

---

## Consequences of Process Decomposition

What happens when composable nodes that shared a container are split into
separate processes, across different RMW implementations.

### `use_intra_process_comms: true` Is Harmless

**It does not break.** The rclcpp publisher always creates a DDS publisher
alongside the intra-process path. When a subscriber is in another process,
`get_intra_process_subscription_count()` returns 0, so the publisher sends via
DDS. The intra-process path finds no local subscribers and returns immediately.

There is no error, no warning, no silent failure. The flag can be left in place.

### FastDDS (Default in Humble)

**Default same-host transport: Shared Memory (SHM)**

FastDDS enables SHM transport by default since v2.3.4. Communication
automatically uses `/dev/shm` segments instead of UDP.

**Data path for PointCloud2 (~1.9 MB):**

3 copies total (serialize → write to SHM → deserialize). Eliminates kernel
network stack but keeps CDR serialization.

| Message | Intra-process | FastDDS SHM | Slowdown |
|---------|--------------|-------------|----------|
| Small (<1KB) | <10 us | ~50 us | ~5x (negligible absolute) |
| 2 MB payload | <100 us | ~2.8 ms | ~28x |

**Critical caveat — SHM segment size**: Default is 512 KB. Messages exceeding
this fall back to **UDP loopback**. A 1.9 MB PointCloud2 hits this cliff. Fix:

```xml
<transport_descriptors>
    <transport_descriptor>
        <transport_id>shm_transport</transport_id>
        <type>SHM</type>
        <segment_size>4194304</segment_size>  <!-- 4MB -->
    </transport_descriptor>
</transport_descriptors>
```

**FastDDS Data Sharing (true zero-copy)**: Requires **bounded/POD types only**.
PointCloud2, Image, and virtually all Autoware sensor messages contain `vector<>`
or `string` fields, so Data Sharing **cannot be used**.

**Discovery overhead**: Simple Discovery Protocol scales as `n*(n-1)`. Going from
15 to 54 participants = 13.6x more discovery traffic. Mitigation: FastDDS
Discovery Server (`ROS_DISCOVERY_SERVER=localhost:11811`).

### CycloneDDS

**Default same-host transport: UDP loopback**

CycloneDDS has **no built-in shared memory**. Same-host communication goes
through the full kernel network stack via loopback. ~5 copies total including
kernel socket buffer copies and UDP fragmentation.

**CycloneDDS + iceoryx (PSMX plugin):** Adds shared memory but requires RouDi
daemon, XML configuration, and iceoryx libraries. For non-fixed-size types like
PointCloud2, still serializes to shared memory (2 copies).

### Zenoh (Tier 1 since Kilted)

**Router-based architecture, no DDS.** Discovery overhead drops by **97-99.97%**
compared to DDS. Going from 15 to 54 processes has minimal impact. Data goes
through TCP loopback. CDR serialization still required.

### Resource Overhead: 15 Containers → 54 Processes

| Configuration | Estimated baseline overhead |
|---|---|
| 15 containers | ~300 MB (15 × ~20 MB each) |
| 54 processes | ~756 MB (54 × ~14 MB each) |
| Delta | **+~450 MB** |

Thread count increases from ~75-165 to ~270-594 (5-11 DDS threads per
participant). Full graph discovery convergence can take **several seconds**.

### Impact by Message Type

| Message type | Size | Intra-process | Inter-process (SHM) | Impact |
|---|---|---|---|---|
| `geometry_msgs/Twist` | 48 bytes | <1 us | ~10-50 us | Negligible |
| `tf2_msgs/TFMessage` | ~200 bytes | <1 us | ~10-50 us | Negligible |
| `nav_msgs/Path` | ~1-10 KB | <10 us | ~50-200 us | Minor |
| `sensor_msgs/Image` | ~1-2 MB | <100 us | ~1-3 ms | Significant |
| `sensor_msgs/PointCloud2` | ~1-4 MB | <100 us | ~2-5 ms | Significant |

**Bottom line**: Splitting is safe — `use_intra_process_comms` gracefully falls
back to DDS. The cost is CDR serialization on large messages and increased
memory/discovery overhead. For Autoware's LiDAR/camera pipelines, this adds
~1-5 ms per pipeline stage. For everything else, the impact is negligible.

---

## Linux Isolation Within a Process

Can we keep nodes in containers while using Linux kernel features to add crash
isolation, resource limits, and memory protection?

### Crash Isolation: Not Possible

| Approach | Why it fails |
|----------|-------------|
| SIGSEGV handler + `pthread_exit()` | Address space already corrupted; lock poisoning; C++ destructors skipped |
| `siglongjmp()` from handler | Skips all RAII (lock_guard, shared_ptr); heap metadata may be corrupted |
| seccomp-bpf | Operates on syscalls, not hardware faults |
| `clone()` without `CLONE_VM` | Separate address space = process isolation by another name; loses shared_ptr IPC |

**Every production system that successfully isolates native code crashes uses
processes**: Chrome (process per site), Erlang (language-level processes), Android
(process per app), Nginx (worker processes), PostgreSQL (process per connection).

### CPU Isolation: Yes (cgroups v2 Threaded Mode)

Individual threads can be placed in different cgroups for CPU control:

| Controller | Threaded? | What it controls |
|---|---|---|
| `cpu` | Yes | Bandwidth (quota/period), weight |
| `cpuset` | Yes | CPU/NUMA pinning |
| `pids` | Yes | Thread count limits |
| `memory` | **No** | Process-level only |
| `io` | **No** | Process-level only |

Requires a **fixed mapping from node to thread** — the standard ROS 2 executors
don't provide this (callback dispatch is non-deterministic). A custom executor
that dedicates a specific thread to each node's callbacks is needed.

### Memory Protection: Partial (Intel MPK)

Memory Protection Keys (Intel Skylake+, AMD Zen 3+) assign one of 16 keys to
memory pages. A per-thread register (PKRU) controls read/write access per key.
Switching costs ~34 cycles (~7 ns).

```
Container process
├── Shared memory (key 0): rclcpp, libc, DDS, stacks, globals
├── Plugin A heap (key 1): jemalloc arena → pkey_mprotect(key 1)
├── Plugin B heap (key 2): jemalloc arena → pkey_mprotect(key 2)

Thread A: PKRU = allow(key 0, key 1), deny(key 2)
Thread B: PKRU = allow(key 0, key 2), deny(key 1)
```

**What it protects**: Heap memory in per-plugin arenas (accidental buffer
overflows, use-after-free, wild pointers). Up to 14 plugins per container.

**What it does NOT protect**: Stack, global/static variables, code pages, shared
libraries (all key 0). Any code can execute `WRPKRU` to bypass.

### What Cannot Be Isolated Within a Process

| Resource | Why not |
|----------|---------
| Memory limits | `memory` cgroup is process-only |
| OOM handling | `oom_score_adj` is process-only |
| I/O bandwidth | `io` cgroup is process-only |
| Crash fate | Shared address space = corrupted state |
| Lock state | Mutex held by crashed thread poisons all waiters |

### Summary

| | Container (current) | Container + Linux | Separate processes |
|---|---|---|---|
| Zero-copy IPC | Yes | Yes | No (CDR serialize) |
| Crash isolation | No | **No** | Yes |
| CPU per node | No | **Yes** (cgroups v2) | Yes |
| Memory per node | No | **No** | Yes |
| Memory corruption | No protection | **Partial** (MPK) | Full |
| Restart individual node | No | **No** | Yes |

---

## Our Approach: clone(CLONE_VM)

Given the constraints above, we chose `clone(CLONE_VM)` without `CLONE_THREAD`:
each composable node gets its own Linux PID while sharing the container's virtual
memory. This is a middle ground between threads and full processes.

See [clone-vm-container-design.md](clone-vm-container-design.md) for the
detailed implementation design.

See [roadmap/phase-19-isolated_container.md](roadmap/phase-19-isolated_container.md)
for the implementation roadmap.

### What clone(CLONE_VM) Gives Us

| Property | std::thread | clone(CLONE_VM) | Separate process |
|---|---|---|---|
| Zero-copy IPC | Yes | **Yes** | No |
| Signal isolation (SIGSEGV) | No | **Yes** | Yes |
| Separate PID per node | No | **Yes** | Yes |
| `waitpid()` death detection | No | **Yes** | Yes |
| `pidfd` monitoring | No | **Yes** | Yes |
| cgroups CPU per node | TID only | **PID** | Yes |
| `kill(pid)` per node | No | **Yes** | Yes |
| Memory limits per node | No | No | Yes |
| Mutex poisoning risk | Shared | **Shared** | Isolated |
| DDS overhead | 1 participant | 1 participant | N participants |

### Why Not Full Process Separation

Full process separation (one node per process) eliminates all isolation problems
but sacrifices intra-process zero-copy IPC. For Autoware's LiDAR/camera
pipelines, this adds ~1-5 ms per pipeline stage — significant for real-time
perception. It also increases memory by ~450 MB and DDS discovery overhead by
~14x.

`clone(CLONE_VM)` preserves zero-copy IPC (shared address space) while gaining
per-node PID, signal isolation, and cgroup control. The trade-off is complexity
and residual shared-memory risks (mutex poisoning, memory corruption).

### Why Not Agnocast

TIER IV's Agnocast achieves true zero-copy IPC between separate processes for
all message types via a Linux kernel module. However, it is **not transparent**
— nodes must use `agnocast::Publisher`/`agnocast::Subscription` instead of
standard rclcpp types. Since we do not control node source code, Agnocast is not
viable for our use case.

---

## References

- [ROS 2 Intra-Process Communication Design](https://design.ros2.org/articles/intraprocess_communications.html)
- [ROS 2 Zero Copy via Loaned Messages](https://design.ros2.org/articles/zero_copy.html)
- [Impact of ROS 2 Node Composition (arXiv:2305.09933)](https://arxiv.org/abs/2305.09933)
- [FastDDS SHM Transport](https://fast-dds.docs.eprosima.com/en/v2.6.10/fastdds/transport/shared_memory/shared_memory.html)
- [CycloneDDS Shared Memory Support](https://github.com/ros2/rmw_cyclonedds/blob/rolling/shared_memory_support.md)
- [Agnocast: Zero-Copy IPC (arXiv:2506.16882)](https://arxiv.org/abs/2506.16882)
- [Control Group v2 — Kernel Docs](https://docs.kernel.org/admin-guide/cgroup-v2.html)
- [Memory Protection Keys — Kernel Docs](https://docs.kernel.org/core-api/protection-keys.html)
- [Chrome Site Isolation](https://www.chromium.org/Home/chromium-security/site-isolation/)
- [Zenoh Discovery](https://zenoh.io/blog/2021-03-23-discovery/)
- [launch_ros #171: LoadComposableNodes hang](https://github.com/ros2/launch_ros/issues/171)
