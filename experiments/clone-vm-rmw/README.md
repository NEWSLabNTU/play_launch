# clone(CLONE_VM) containers: does the RMW survive it?

A minimal reproducer, and the measurements it produced on an AGX Orin
(aarch64, JetPack 6.2 / R36.4.4, Ubuntu 22.04, glibc 2.35, ROS 2 Humble).

## Why this exists

`play_launch`'s `--container-mode isolated` fork+execs a `component_node` per
composable. That buys a real segfault boundary and costs a full DDS participant
per node — phase 65 measured what that costs on a 93-composable stack, and the
golf-cart capture put a number on the per-process floor: **~4.9% of a core for a
process that does nothing at all**.

`clone(CLONE_VM)` without `CLONE_THREAD` is the shape that would give both: each
node gets its own PID and its own signal disposition (so a SIGSEGV kills one
node, not the container) while sharing the address space (so intra-process
zero-copy still works and there is one DDS participant, not 93).

That was designed once and abandoned —
[`docs/archive/clone-vm-container-design.md`](../../docs/archive/clone-vm-container-design.md)
— with the surviving note that fork+exec "avoids all TLS/glibc issues that
plague clone(CLONE_VM)". This directory reopens the question with measurements
instead of memory.

## What was found

### 1. The abandoned code could never have run on this machine

The historical child bootstrap (`7e6122a^:src/play_launch_container/src/clone_isolated_component_manager.cpp`)
reads the thread pointer with

```c
syscall(SYS_arch_prctl, 0x1003 /* ARCH_GET_FS */, &fs_val);
```

`SYS_arch_prctl` is **not defined on aarch64** — that is a compile error, not a
runtime bug. Every offset around it is x86_64-specific too, and x86_64 and
aarch64 do not merely differ in the numbers, they use **different TLS variants**:

| | x86_64 (variant II) | aarch64 (variant I) |
|---|---|---|
| `struct pthread` vs thread pointer | **at** the TP | **1984 bytes below** the TP |
| `tid` field | TP + 720 | TP − 1776 (`pthread_self() + 208`) |
| `tcbhead_t.self`, `multiple_threads` | TP + 0x10, TP + 0x18 | not there at all |

Both numbers above were measured on this machine, not read off a header. So the
old code's `tls_base + 720` would, on aarch64, write a TID **past** the thread
pointer and into the actual thread-local variable block — silently corrupting
whichever `thread_local` happens to live there. That is the kind of damage that
produces a hang rather than a crash, which may be where the memory of "it gets
stuck in the RMW backend" comes from.

**The probe discovers both offsets at runtime** (compare `pthread_self()` to the
thread pointer; scan for our own TID) rather than hardcoding either, so it is
correct on both variants and survives a glibc upgrade.

### 2. The foundation is sound on aarch64

`clone(CLONE_VM | CLONE_SETTLS)` with a fresh `_dl_allocate_tls()` block, plus
`uselocale(LC_GLOBAL_LOCALE)` and `__ctype_init()`, gets a child through every
glibc facility the x86_64 port struggled with:

```
tid set → ctype/locale → isalpha → malloc/free → printf("%f") → shared mutex → pthread_create
```

All seven passed. Note the last one: the archived design concluded that
"`pthread_create()` accesses too many struct pthread fields to safely initialize
manually" and restricted clone children to a `SingleThreadedExecutor` forever.
On aarch64/glibc 2.35 `pthread_create()` **returns 0 in the clone child**. That
constraint deserves re-testing rather than inheriting.

### 3. The verdict is per-backend, and it is not subtle

```
BACKEND                    VERDICT    CHILD REACHED          DETAIL
──────────────────────────────────────────────────────────────────────────
rmw_cyclonedds_cpp         EXIT 134   —                      double free or corruption (out)
rmw_fastrtps_cpp           PASS       entered spin()         clean through shutdown
rmw_zenoh_cpp              PASS       entered spin()         clean through shutdown
```

(`run_matrix.sh` output, verbatim. Cyclone reports no stage because the child
dies before the parent's first report; the gdb invocation below names the frame.)

**Two of the three work.** `rmw_zenoh_cpp` 0.1.9 behaves exactly as
`rmw_fastrtps_cpp` does, on every axis measured — both children deliver, both
cancel, shutdown returns, and a SIGSEGV kills only the child that raised it.
It needs its router (`rmw_zenohd`) running, like any Zenoh deployment; that is
unrelated to CLONE_VM, and a run without one fails in the ordinary way with
`Unable to connect to a Zenoh router`.

**FastRTPS works, completely.** The node is built in the parent (exactly as a
container does), the child spins it, the parent publishes, the child's callback
fires, `cancel()` brings the child home, `rclcpp::shutdown()` returns. Exit 0.

**CycloneDDS dies**, and the stack names the reason precisely:

```
#0  _dl_tlsdesc_dynamic ()              at ../sysdeps/aarch64/dl-tlsdesc.S:164
#1  libddsc.so.0
#2  dds_take ()
#3  librmw_cyclonedds_cpp.so
#4  rcl_take ()
#5  rclcpp::SubscriptionBase::take_type_erased(...)
...
#9  rclcpp::executors::SingleThreadedExecutor::spin()
#10 child_fn ()
#11 thread_start ()                     at clone.S:79
```

`_dl_tlsdesc_dynamic` is the **dynamic TLS descriptor resolver**. Cyclone reads a
thread-local from a dlopen'd module on its take path, which goes through a TLS
descriptor rather than a fixed offset; resolving it walks the DTV of the calling
thread. The DTV in a `_dl_allocate_tls()` block is not in a state that resolver
accepts, and it faults on first touch.

Note the verdict column above says `EXIT 134` — SIGABRT, "double free or
corruption" — where an earlier run of the same binary reported a SIGSEGV in the
frame below. **That the symptom moves between runs is itself the signature.** A
TLS block whose DTV the resolver will not accept does not fail the same way
twice: sometimes the resolver faults, sometimes it returns a wrong pointer and
the corruption surfaces later in the allocator. Do not chase the specific
symptom; the cause is the same either way.

This is the real answer to "it gets stuck in the RMW backend library": it is not
RMW as a layer, it is **one backend's use of dynamic TLS**. Anything that reads a
`thread_local` from a shared object the loader resolved lazily is exposed;
anything using initial-exec TLS, or none, is not — and two of the three shipped
backends are in the second group.

### 4. Teardown must be `cancel()`, never a signal

The first version of this probe stopped the child with `SIGKILL`. FastRTPS then
"hung" — and it was not FastRTPS. Every thread of the parent sat in
`futex_wait_queue_me`, because the child died holding a non-robust mutex
somewhere in rclcpp or the DDS stack, in an address space the parent shares.

That is **Risk 2 of the archived design, reproduced on demand**. Replacing the
kill with `executor->cancel()` — which writes the interrupt guard condition, so
`spin()` returns and the clone function exits through the kernel's `_exit` —
turned the same run into a clean exit 0.

Consequence for any future container mode: a clone child may only ever be
stopped cooperatively. `SIGKILL` on a clone child is not a fallback, it is a
guaranteed deadlock of the whole container. The archived design already said
this; it is now measured.

### 5. A crashing node kills the container unless the dump is suppressed

This is the finding that decides whether the mode is worth having at all, and
the archived design does not mention it. Its very first table asserts
"SIGSEGV kills only this child: **Yes**". That is false as written.

SIGSEGV, SIGABRT and SIGBUS — the signals a crashing node actually raises — are
*core-dumping* signals. To write a dump the kernel needs the address space to
itself, so `do_coredump()` calls `zap_threads()` on **every other task sharing
that mm**. In a CLONE_VM container that is the manager and every sibling node.
The crash the boundary exists to contain takes down exactly what it is meant to
protect.

Measured both ways, with no play_launch involved:

```
                                  parent   child0   child1
SIGSEGV to child0, as designed     dead     dead     dead      exit 139
+ PR_SET_DUMPABLE(0), RLIMIT_CORE 0 alive   Z        S         ISOLATION HOLDS
```

and again end to end through `play_launch --container-mode clone-vm`, on
FastRTPS and on Zenoh alike:

```
before   container S   child1 S   child2 S
after    container S   child1 Z   child2 S
```

The coredump zap is a property of the kernel, not of any backend: it reproduces
identically under both working RMWs, and suppressing the dump fixes it under
both.

Two calls in the child, before it ever spins, are the whole fix:

```c
prctl(PR_SET_DUMPABLE, 0, 0, 0, 0);
struct rlimit no_core{0, 0};
setrlimit(RLIMIT_CORE, &no_core);
```

An undumpable task makes `do_coredump()` bail before it reaches the zap. The
cost is that clone children produce no core files — a core per node, or a
container that survives one.

## The mode this produced

`--container-mode clone-vm`, hidden from `--help` (`#[value(hide = true)]`) and
implemented by `CloneVmComponentManager`. It refuses to start under
`rmw_cyclonedds_cpp` with the reason above rather than dying on the first
message, treats `rmw_fastrtps_cpp` and `rmw_zenoh_cpp` as measured-good, warns
on anything else rather than blocking the evaluation it exists for, and stops
children only with
`executor->cancel()` — never a signal. A child that does not exit within the
grace period has its stack and TLS **deliberately leaked**, because unmapping a
stack a running child is executing on is worse than the leak.

## Running it

```bash
# --base-paths experiments on purpose: every colcon recipe in the justfile uses
# --base-paths src, so this package is never part of the normal build.
colcon build --base-paths experiments --packages-select clone_vm_rmw_probe
./experiments/clone-vm-rmw/run_matrix.sh
```

Two switches on the probe itself, both off by default, because each one changes
what the run is measuring:

| env | what it adds |
|---|---|
| `PROBE_SEGV_CHILD0=1` | SIGSEGV child 0 and report whether the parent and child 1 survive |
| `PROBE_NO_COREDUMP=1` | make the children undumpable first — the difference between the two rows in finding 5 |

`run_matrix.sh` discovers the installed backends from the ament index rather
than hardcoding them, so an added `rmw_zenoh_cpp` is picked up with no edit.

`rmw_zenoh_cpp` needs its router up before any of this means anything:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
```

Without it the probe fails at `rclcpp::init` with `Unable to connect to a Zenoh
router`, which is an ordinary Zenoh misconfiguration and says nothing about
CLONE_VM.

For a SEGV, the frame is named by running under gdb:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp gdb -q -batch \
  -ex 'set follow-fork-mode child' -ex run -ex 'bt 25' --args <probe>
```

## What this does and does not establish

It establishes that the address-space-sharing model is viable on aarch64 for
`rmw_fastrtps_cpp`, end to end: two nodes, the second loaded while the first is
spinning, both delivering, both cancelled, clean shutdown — and that a crashing
node takes only itself down.

It does **not** say anything about the failure modes that only appear under
load: mutex poisoning from a *real* crash rather than a signal we chose to send,
heap corruption across nodes, or `shared_ptr` refcounts leaked on a dead child's
stack. The archived design enumerates these; none of them is measured here.

Nor about memory. A clone child cannot have its own memory limit or its own OOM
scope — that is the one thing fork+exec buys which this can never match.
