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

### 3. All three shipped RMWs work — and the one that did not was our bug

```
BACKEND                    VERDICT    CHILD REACHED          DETAIL
──────────────────────────────────────────────────────────────────────────
rmw_cyclonedds_cpp         PASS       entered spin()         clean through shutdown
rmw_fastrtps_cpp           PASS       entered spin()         clean through shutdown
rmw_zenoh_cpp              PASS       entered spin()         clean through shutdown
```

Three repetitions each, all PASS.

This section previously said Cyclone segfaulted inside `dds_take` and concluded
that one backend's use of dynamic TLS was incompatible with `CLONE_VM`. **That
was wrong.** The crash was real and reproducible, and the cause was a thread
pointer this code computed incorrectly. Cyclone was simply the only backend that
touched the broken part often enough to notice. Keeping the wrong version here
would be worse than useless, so what follows is the actual diagnosis.

### 4. The bug: `_dl_allocate_tls` returns the thread pointer, not a `struct pthread *`

Measured, not read out of a header:

```
live thread: TP=0xffffb2c967e0  pthread_self=0xffffb2c96020  gap=-1984
             TP[0] = 0xffffb2c96f20          <- the DTV pointer

_dl_allocate_tls() -> 0xaaaaf0751a80
    blk[0]         = 0xaaaaf07521d0          <- a DTV: blk IS the TCB, i.e. the TP
    (blk-gap)[0]   = (nil)                   <- what the child was given as its TP
```

The code passed `blk - gap` to `CLONE_SETTLS`, treating the return as a
`struct pthread *` that needed adjusting across the variant gap. It does not:
on aarch64 `_dl_allocate_tls` hands back the **TCB address, which is exactly the
value the thread pointer should take**.

**Why this hid for so long.** On aarch64 the DTV pointer lives at `TP[0]`.
Static and initial-exec TLS are addressed as `TP + offset` and never read it, so
with a thread pointer 1984 bytes off the child was reading a shifted, zeroed
window of its own TLS block — and a zeroed tcache is a *valid empty* tcache, a
zeroed `errno` is a fine `errno`. `malloc`, `printf("%f")`, `isalpha`, mutexes
and `pthread_create` all pass. Only **dynamic** TLS reads `TP[0]`, and only a
`thread_local` in a `dlopen`'d module is dynamic.

The fault, from `ld.so` itself:

```asm
_dl_tlsdesc_dynamic:
    mrs  x4, tpidr_el0    ; thread pointer
    ldr  x0, [x4]         ; x0 = TP[0] = DTV pointer
    ldr  x2, [x0]         ; dtv[0].counter          <-- SIGSEGV, x0 = 0
```

which is precisely where gdb stopped, with `x0 = 0x0`.

**Why Cyclone and nothing else.** `libddsc.so.0.10.5` carries exactly three
`R_AARCH64_TLSDESC` relocations, and one of them is named:

```
R_AARCH64_TLSDESC  tsd_thread_state + 0
```

That is `q_thread.c`'s exported per-thread state pointer, read by

```c
DDS_INLINE_EXPORT inline struct thread_state *lookup_thread_state (void) {
  struct thread_state *thrst = tsd_thread_state;   // dynamic TLS read
  ...
}
```

which is the **first statement of `dds_read_impl`**, the body of `dds_take`. So
every single take does a dynamic-TLS read.

The other two backends are not virtuous, merely lucky:
`librmw_zenoh_cpp.so` has **no** TLSDESC relocations at all, and
`libfastrtps.so.2.6` has two — `std::__once_call` and `std::__once_callable`,
libstdc++'s `std::call_once` machinery, touched during initialisation in the
parent and never on the take path in the child.

The fix is one line: pass `_dl_allocate_tls`'s return value to `CLONE_SETTLS`
unchanged. `PROBE_TLS_SHIFT=1` restores the old, broken behaviour if you want to
watch it fail.

**This very likely explains the archived design's TLS saga too.** That document
enumerates a list of `struct pthread` fields it had to hand-initialise on x86_64
— a null locale pointer crashing `__printf_fp_l`, null ctype tables crashing
`isalpha`, a null `tcbhead_t.self`. Every one of those is a symptom of reading
`struct pthread` at the wrong address, which is the same mistake in the same
place. The fixups are still applied here (`_dl_allocate_tls` genuinely does not
set the tid, the locale or the ctype tables — `start_thread` does), but they
should be re-derived from a correct thread pointer rather than inherited.

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
implemented by `CloneVmComponentManager`. It refuses no backend —
all three shipped RMWs are measured working — but still reports an untested one
through the same hook rather than silently accepting it, and stops children only
with
`executor->cancel()` — never a signal. A child that does not exit within the
grace period has its stack and TLS **deliberately leaked**, because unmapping a
stack a running child is executing on is worse than the leak.

## The real stack: what broke, and why it now works

Against the golf cart's Autoware stack — 142 nodes, 16 containers, 82
composables, perception off, the two modes differing in nothing but
`--container-mode` — clone-vm first failed hard: 14 of 16 containers died and
only 28 of 82 composables loaded. Two defects, and the second was created by
fixing the first.

### 1. `_dl_allocate_tls` leaves `struct pthread` uninitialised

It allocates through `__libc_memalign` and clears only the TCB — 16 bytes at the
thread pointer, holding the DTV. The `struct pthread` **below** the thread
pointer keeps whatever the allocator last had there. `pthread_create` never sees
that: it takes its stack from a fresh mmap, so a real thread starts zeroed.

Loading one real Autoware composable by hand showed it immediately:

```
node 1: spinning in clone-vm child pid 845369
malloc(): unsorted double linked list corrupted
```

Zeroing the region below the thread pointer fixes that load — and took the stack
from 18 to 28 of 82. It also made things worse, which is the interesting part.

### 2. A zeroed `struct pthread` tells glibc the child is single-threaded

`header.multiple_threads` is at **offset 0**. glibc reads it to decide whether
the process is single-threaded and takes an unlocked fast path when it is:
`__libc_malloc` calls `_int_malloc` **without the arena lock** under
`SINGLE_THREAD_P`. So a child with a freshly zeroed `struct pthread` concludes it
is alone and allocates unlocked while every thread in the parent allocates too.

Measured on this glibc, which is where the offset comes from:

```
before any thread     *(int *) pthread_self() = 0   __libc_single_threaded = 1
after pthread_create  *(int *) pthread_self() = 1   __libc_single_threaded = 0
```

Before the zeroing, that field held garbage that was usually non-zero, so the
child usually looked multi-threaded by accident. Zeroing made it reliably 0 —
which is why container failures went from 11 of 16 to 14 of 16. **The two fixes
belong together.**

The race shows up as whatever it happens to break. All three of these came out
of the same build:

```
malloc.c:4302: _int_malloc: Assertion `(size) >= (nb)' failed
malloc(): unsorted double linked list corrupted
pthread_mutex_lock.c:94: Assertion `mutex->__data.__owner == 0' failed
```

which is why chasing the symptom went nowhere. It is one race with three faces,
and the mutex assertion that started the hunt was the least informative of them.
The commonality analysis is worth keeping for the same reason: the casualties
shared no executor type, no composable count, and not even whether they had ever
cloned a child — because the corruption was in the shared heap, not in any
container's own structure.

### With both fixes: identical behaviour, measurably cheaper

Same stack, same run pair, 62 of 82 composables loaded in **both** modes, zero
assertions and zero heap corruption in both:

| | observable | clone-vm | delta |
|---|---|---|---|
| host CPU, steady state | 85.6% | **72.8%** | **−12.8 points** |
| host CPU, whole run | 84.4% | 74.9% | −9.5 |
| host CPU, peak | 99.9% | 100.0% | — |
| threads | 768 | **673** | −95 |
| host memory, peak | 18.7 GB | 18.8 GB | +0.1 |

On a 12-core Orin, 12.8 points is about **1.5 cores**, for the same work, while
keeping the per-node SIGSEGV boundary. Memory is unchanged, which is expected:
the address space is shared either way, and the saving is threads and DDS
participants rather than pages.

**One run per mode for the CPU figure.** Treat the size as indicative, not
settled: `ublox` respawned 39 and 37 times respectively across the two runs (the
receiver is not attached), which is noise present on both sides but not
identical.

The *correctness* claim has more behind it, because a heap race that appears once
in one 200 s run proves very little. Four clone-vm runs of the full stack, the
original plus three repeats:

```
rep1: composables=62/82 asserting=0 malloc_corrupt=0
rep2: composables=62/82 asserting=0 malloc_corrupt=0
rep3: composables=62/82 asserting=0 malloc_corrupt=0
```

identical every time. Before the fix the failure was immediate and in every run.

### What is still not initialised

The child sets four things: `multiple_threads`, its tid, the locale, and the
ctype tables. `start_thread` sets considerably more. That is mostly fine, and for
a reason worth stating: **zero is the correct initial value for nearly every
field** — `gscope_flag`, the TSD first block, `cancelhandling` all start zeroed
in a real thread, which is why zeroing `struct pthread` was the right move.
`multiple_threads` is the one field whose correct value is not zero, which is
exactly why zeroing alone made things worse before this was found.

The known gap is `robust_head`, which `start_thread` makes self-referential
before calling `set_robust_list`. A clone child here has it NULL and never makes
that syscall, so a robust mutex taken in a child that then dies would not be
recovered. Latent rather than live: neither `libfastrtps` nor `libddsc` refers to
the robust-mutex API at all on this install. It becomes real the moment something
in the stack does — a shared-memory transport is the likely candidate.

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
| `PROBE_TLS_SHIFT=1` | restore the wrong thread pointer of finding 4, to watch Cyclone fail |
| `PROBE_NO_FIXUPS=1` | skip the tid / locale / ctype initialisation in the child |

`run_matrix.sh` discovers the installed backends from the ament index rather
than hardcoding them, so an added `rmw_zenoh_cpp` is picked up with no edit.

`rmw_zenoh_cpp` needs its router up before any of this means anything:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
```

Without it the probe fails at `rclcpp::init` with `Unable to connect to a Zenoh
router`, which is an ordinary Zenoh misconfiguration and says nothing about
CLONE_VM.

**Leaked `/dev/shm/fastrtps_*` segments will make this flaky, and the flakiness
looks like a backend problem.** Repeated runs produced intermittent 60 s
timeouts on FastRTPS and Zenoh that vanished once `/dev/shm` was cleared and
`FASTRTPS_DEFAULT_PROFILES_FILE` was pointed at `tests/fixtures/fastdds_no_shm.xml`.
A SIGKILLed process never runs `shm_unlink`, and this probe kills children on
purpose. Check `ls /dev/shm | grep -c fastrtps` before believing any
intermittent result here.

For a SEGV, the frame is named by running under gdb:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp gdb -q -batch \
  -ex 'set follow-fork-mode child' -ex run -ex 'bt 25' --args <probe>
```

## What this does and does not establish

It establishes that the address-space-sharing model is viable on aarch64 for all
three shipped RMWs, end to end: two nodes, the second loaded while the first is
spinning, both delivering, both cancelled, clean shutdown — and that a crashing
node takes only itself down.

It does **not** say anything about the failure modes that only appear under
load: mutex poisoning from a *real* crash rather than a signal we chose to send,
heap corruption across nodes, or `shared_ptr` refcounts leaked on a dead child's
stack. The archived design enumerates these; none of them is measured here.

Nor about memory. A clone child cannot have its own memory limit or its own OOM
scope — that is the one thing fork+exec buys which this can never match.
