# Efficient monitoring: what /proc costs, and what htop does about it

**Status: research, with measurements. No code changed yet.**

**§6 supersedes the extrapolation in §2.3.** The bench model predicted the
monitor at 0.47% of a core; measured against the real golf-cart stack it is
3.8%, and it is 26% of what play_launch spends rather than most of it. Read §6
before acting on §5.

Motivated by high CPU on a golf-cart Autoware stack on an Orin. Every number
below was measured on a bench Tegra (`5.15.148-tegra`, 12 cores, 64 GiB) — the
same SoC class as the vehicle, so the ARM-versus-x86 differences that turn out
to matter are represented rather than assumed. Probes are in `tmp/` and are
reproduced in full at the end.

**The headline is a negative result.** The resource monitor is not the problem.
At its 2 s default, supervising 144 processes, the whole per-tick walk costs
**0.47% of one core**. It can be made 6.2× cheaper and that is worth doing, but
it will not be felt. The cost that scales with the *system* rather than with the
monitor is per-message interception, and there the overhead figure recorded in
`CLAUDE.md` **understates this hardware by 4.6×**.

---

## 1. What a /proc read costs

Open, read to EOF, close — what a monitor actually does, since most /proc
contents are generated at `open()` and cannot be cheaply re-read.

### Per-process (multiplies by process count)

| file | p50 | note |
|---|---|---|
| `/proc/<pid>/stat` | **10.9 µs** | one line, formatted from `task_struct` |
| `/proc/<pid>/statm` | 7.1 µs | seven numbers |
| `/proc/<pid>/task` (readdir) | 7.3 µs | thread count |
| `/proc/<pid>/status` | 13.7 µs | ~50 formatted lines |
| `/proc/<pid>/smaps_rollup` | 21.3 µs | walks page tables |
| `/proc/<pid>/smaps` | 49.4 µs | walks and formats every VMA |

### Namespace-wide (identical for every process in the namespace)

| file | p50 | entries |
|---|---|---|
| `/proc/net/tcp` | **2611.8 µs** | 51 |
| `/proc/net/tcp6` | **2517.7 µs** | 3 |
| `/proc/net/udp` | 167.8 µs | 6 |
| `/proc/net/udp6` | 167.2 µs | 5 |
| `/proc/net/sockstat` | **32.7 µs** | — |
| `/proc/net/sockstat6` | **29.0 µs** | — |

### System-wide (once per tick)

| file | p50 |
|---|---|
| `/proc/stat` | 27.8 µs |
| `/proc/meminfo` | 7.1 µs |
| `/proc/diskstats` | 101.9 µs |
| `/proc` (readdir, 871 entries) | 406.6 µs |

---

## 2. Three findings from that table

### 2.1 `/proc/net/tcp` costs the same whether it has 51 sockets or 3

`tcp6` holds **three** entries and costs 2517 µs — within 4% of `tcp` with 51.
The cost is not the sockets; it is the **hash table**. `/proc/net/tcp`'s
seq_file iterates every bucket of the TCP hash table, which the kernel sizes
from physical memory. On a 64 GiB board that is hundreds of thousands of mostly
empty buckets walked to print a few dozen lines.

Consequences worth stating plainly:

- **It does not get cheaper on an idle system.** The floor is the table size.
- **It gets worse on bigger machines**, which is the opposite of the intuition
  that a beefier board absorbs monitoring overhead.
- One read costs as much as **240 `/proc/<pid>/stat` reads**.

`/proc/net/sockstat` reports what we actually want — counts — from per-protocol
counters, with no table walk:

```
sockets: used 1253
TCP: inuse 48 orphan 0 tw 0 alloc 50 mem 4
UDP: inuse 5 mem 4
```

**32.7 µs against 2611.8 µs — 80×**, for the same question.

One honest caveat: `inuse` is not exactly the line count of `/proc/net/tcp`
(48 versus 50 here). The file lists sockets in the hash table including
`TIME_WAIT`; `sockstat` reports the protocol's in-use counter and breaks out
`tw`/`orphan`/`alloc` separately. For a monitoring gauge that is not a loss —
arguably it is more informative — but it is a different number, not a cheaper
route to the same one.

### 2.2 One `stat` read already contains what we make three reads for

`/proc/<pid>/stat` is currently read for `utime`/`stime` only. It also carries,
in the same line:

```
state       R
utime       3
stime       0
num_threads 1        <- no readdir of task/ needed
vsize       19456000 bytes
rss         2069 pages = 8474624 bytes
```

Fields 18, 21 and 22 (1-indexed) are `num_threads`, `vsize` and `rss`. So the
current triple — `stat` for CPU, `statm` (via sysinfo) for memory, `readdir
task/` for the thread count — is **25.3 µs of work for a 10.9 µs read**.

### 2.3 The measured A/B at golf-cart scale

144 real forked processes, both arms producing the same per-process numbers plus
namespace socket counts:

```
  arm A (today)        9.43 ms/tick   436 opens/tick
  arm B (proposed)     1.51 ms/tick   146 opens/tick
  reduction             6.2x          66.5% fewer opens

    at  500 ms interval:  A  1.89%   B  0.30%
    at 1000 ms interval:  A  0.94%   B  0.15%
    at 2000 ms interval:  A  0.47%   B  0.08%
```

**6.2×, and it does not matter.** At the 2 s default the monitor is already
under half a percent of one core. This is worth doing for the 500 ms case and
for honesty about where cycles go, not because it will change what the vehicle
feels.

---

## 3. What htop does

Read from source (`htop-dev/htop`, `linux/LinuxProcessTable.c`), not from
reputation — one technique that reputation recommends turns out to be worthless
here.

### 3.1 Reads are gated on which columns are visible — the central idea

```c
ss->flags |= Process_fields[id].flags;      // Settings.c:283
```

Each displayable field declares which /proc reads it needs. htop takes the union
over *visible* columns and reads only those files:

```c
if (ss->flags & PROCESS_FLAG_LINUX_CGROUP)   ... readCGroupFile(...)
if (ss->flags & PROCESS_FLAG_LINUX_SMAPS)    ... readSmapsFile(...)
if (ss->flags & PROCESS_FLAG_IO)             ... readIoFile(...)
if (ss->flags & PROCESS_FLAG_LINUX_SECATTR)  ... readSecattrData(...)
```

Turn off the IO columns and htop never opens `/proc/<pid>/io`. This is the
technique with the largest structural payoff and the one we have none of: our
monitor reads a fixed set every tick regardless of whether anything consumes it.

### 3.2 Expensive reads are sharded across ticks, deterministically by PID

```c
// Read smaps file of each process only every second pass to improve performance
static int smaps_flag = 0;
if ((pid & 1) == smaps_flag) {
   LinuxProcessTable_readSmapsFile(lp, procFd, this->haveSmapsRollup);
}
```

Half the processes per pass, alternating. Cost halves; staleness is bounded at
exactly two ticks; no scheduling state is kept, because the shard is a property
of the PID. Better than a TTL cache for a fixed population — it spreads the work
instead of synchronising it into one expensive tick.

### 3.3 Capability probed once, not per read

```c
this->haveSmapsRollup = (access(PROCDIR "/self/smaps_rollup", R_OK) == 0);
```

Then `smaps_rollup` (21.3 µs) is preferred over `smaps` (49.4 µs) forever after.

### 3.4 Process-shared data is not re-read per thread

```c
process->super.m_virt     = mainTask->super.m_virt;
process->super.m_resident = mainTask->super.m_resident;
```

Threads inherit the main task's memory figures rather than re-reading `statm`
for each. Relevant to us: a ROS node has ~11 threads, so any per-thread read is
an 11× multiplier.

### 3.5 Default interval: 1.5 s

`#define DEFAULT_DELAY 15` (deciseconds). Our 2 s default is already more
conservative than an interactive tool's.

### 3.6 The technique that does not pay — `openat` with a cached dirfd

htop holds a dirfd for `/proc/<pid>` and opens `stat`, `statm`, `io` relative to
it, paying path resolution once instead of three times. Measured here:

```
  full path, 3 files     20.6 us/process
  dirfd + openat x3      20.6 us/process
  saving                  0.3%
```

**0.3%.** Path resolution is not where the time goes; generating the file
contents is. Recorded because it is exactly the kind of optimisation that gets
adopted on plausibility.

---

## 4. Where the cycles actually are

The monitor is 0.47% of a core. So the CPU the vehicle feels is elsewhere, and
two candidates are already documented.

### 4.1 The process count itself — already measured, phase 61

144 processes under `--container-mode isolated` held **10.2 of 12 cores** during
startup against 3.9 under `observable`, at peak load1 190 versus 45. That is the
system, not the supervision of it, and it is the documented price of fault
isolation.

### 4.2 Per-message interception — and the recorded overhead is wrong for ARM

`CLAUDE.md` records, from phase 58 W2:

> `CLOCK_THREAD_CPUTIME_ID` is not in the vDSO, but costs **85 ns/call vs
> 17 ns** for `CLOCK_MONOTONIC` — ~150 ns per message, 0.15% of a core at
> 10k msg/s.

Measured on this Tegra, pinned, three runs, `schedutil` at 1.5 GHz:

```
  CLOCK_MONOTONIC            39.3 ns/call  (vDSO)
  CLOCK_THREAD_CPUTIME_ID   387.2 ns/call  (syscall)      391.2 / 392.2 / 399.5
  CLOCK_REALTIME             35.6 ns/call  (vDSO)
```

**387 ns, not 85.** A 4.6× understatement, and the ratio between the two clocks
is ~10× rather than ~5×. The recorded figures were evidently taken on an x86
development machine; the deployment target is ARM, where the syscall entry the
non-vDSO clock requires is proportionally far more expensive.

Per hook invocation the interceptor pays one `CLOCK_THREAD_CPUTIME_ID` plus one
`CLOCK_MONOTONIC` — **~426 ns on this board against the ~150 ns recorded** — and
it pays it inside **every intercepted process**, so it multiplies by 144 in a
way the monitor never does.

This matters beyond arithmetic, because that 150 ns figure is the justification
for `interception.events` defaulting **on**.

### 4.3 The event log's own cost

`EventLog::write_record` allocates twice per message — `node.to_string()` and
`serde_json::to_string` — then writes the line. Measured at 500k records:

```
  to_string + String alloc :     335 ns/msg
  to_writer, borrowed name :     209 ns/msg      (-37.5%)

  at    5000 msg/s -> today  0.17% of a core, borrowed  0.10%
  at   20000 msg/s -> today  0.67% of a core, borrowed  0.42%
  at   50000 msg/s -> today  1.67% of a core, borrowed  1.05%
  at  200000 msg/s -> today  6.69% of a core, borrowed  4.18%
```

Serialising into the `BufWriter` with a borrowed node name removes both
allocations for a 37.5% cut. Note the other axis: at 50k msg/s this is also
**5.5 MB/s written continuously**, which on eMMC is likely to be felt as kernel
writeback long before the CPU figure is.

---

## 5. What to do, ranked by measured effect

**Not yet decided — this document is the evidence, not the plan.**

1. **Re-measure interception overhead on ARM and revisit the `events` default**
   (§4.2). The largest real cost, the one that multiplies by process count, and
   the one whose published figure is wrong for the hardware we ship to. Nothing
   else on this list changes what the vehicle feels as much as this might.
2. **`/proc/net/sockstat` instead of the four hash-walking files** (§2.1). 80×,
   two lines of parsing, one behaviour change to document (`inuse` versus line
   count).
3. **Take `num_threads`, `vsize` and `rss` from the `stat` line already read**
   (§2.2). Removes `statm` and the `task/` readdir outright.
4. **Serialize events with a borrowed name into the writer** (§4.3). 37.5% of a
   cost that scales with traffic.
5. **Gate per-process reads on what is consumed**, htop-style (§3.1). The
   structural fix; larger than the others and only worth it once there is more
   than one optional read to gate.
6. **Shard expensive reads by PID** if any per-process read ever becomes
   expensive enough to need it (§3.2). Nothing we read today qualifies.

Explicitly **not** recommended: `openat` with a cached dirfd (§3.6), measured at
0.3%.

### An option not taken, and why it is not obvious

Phase 66 gives every node and container a cgroup, and `cpu.stat` reports a whole
group's CPU in one read. Measured: `cpu.stat` 25.0 µs, `memory.current` 21.9 µs,
`pids.current` 20.7 µs — each *more* than a single `/proc/<pid>/stat` at
10.9 µs. So a cgroup read wins only when it replaces three or more per-process
reads, which for a six-composable container it does and for a single-node group
it does not.

Note the asymmetry with memory, where the cgroup read is a **correctness** fix
rather than a performance one: summing children's RSS counts shared pages once
per process and over-reported by 2.4×, which `memory.current` fixed. CPU time is
additive, so no such argument applies — for CPU the cgroup route is an
optimisation to justify per group size, not a defect to repair.

---

## Reproducing

`tmp/` is gitignored; these are small enough to paste back.

| probe | what |
|---|---|
| `tmp/procbench.c` | per-read costs, all three tables in §1 |
| `tmp/tickbench.c` | the 144-process A/B in §2.3 |
| `tmp/clkbench.c` | the clock costs in §4.2 |
| `tmp/openatbench.c` | the dirfd negative result in §3.6 |
| `tmp/serdebench/` | the event-log costs in §4.3 |

htop source: `external/htop` (gitignored; `git clone --depth 1
https://github.com/htop-dev/htop`).


---

## 6. Measured on the real stack

Everything above is bench primitives. This section is the golf-cart launch
itself (`~/repos/2026-golf-cart`, `host:=master launch_perception:=true`),
**159 nodes — 51 pure, 16 containers, 92 composables** — jailed in
`ROS_DOMAIN_ID=137` with CAN TX forced off. Same board.

Sampled `utime+stime` of every `play_launch` process in the launch's session,
in steady state (once the process count settles above 50).

| arm | play_launch, % of one core | min | max | threads | procs |
|---|---|---|---|---|---|
| **0.5.1** (what the vehicle runs) | **47.5** | 15 | 79 | 43 | 70¹ |
| **0.9.0** default | **14.6** | 8 | 32 | 23 | 160 |
| 0.9.0 `--disable-monitoring` | 10.8 | 7 | 26 | 20 | 159 |
| 0.9.0 `--disable-all` | 5.9 | 2 | 44 | 20 | 157 |

¹ session-visible count only — 0.5.1's children `setsid` out of the session, so
this undercounts. Both arms spawned the identical 159 nodes.

### 6.1 The biggest lever is a version upgrade

**0.5.1 costs 3.3× what 0.9.0 costs** on the same workload — 47.5% against
14.6% of a core. No code change required to get it.

Two structural differences behind that. 0.5.1's `launch` is a two-step that
re-invokes itself, so the supervisor is *two* processes with **43 threads**;
0.9.0's is in-memory, one process, **23 threads**. And the parse step:

| | parse time |
|---|---|
| 0.5.1 (Python `dump_launch`) | **71 s** |
| 0.9.0 (Rust parser) | **0.77 s** |

**92×.** That is not steady-state CPU, but for 71 s one core is pinned at
~112% before a single node starts — and it is why the first measurement attempts
here saw nothing: the sampling window expired inside the parse.

### 6.2 Where 0.9.0's 14.6% goes

| component | cost | share |
|---|---|---|
| supervision floor (`--disable-all`) | 5.9% | **40%** |
| web UI + diagnostics + interception | 4.9% | 34% |
| resource monitoring | 3.8% | **26%** |

**Monitoring is a quarter of it.** The largest single term is the irreducible
cost of spawning and supervising 159 processes with `--disable-all` — every
feature off.

### 6.3 The bench model was wrong by 8×, and in an instructive way

§2.3 predicted the per-tick `/proc` walk at 0.47% of a core for 144 processes at
the 2 s default. The measured monitoring delta is **3.8%**. So the file reads
this document spent most of its length on are roughly **an eighth** of what
monitoring costs — the rest is CSV writing, `sysinfo` bookkeeping, per-node
actor state and channel traffic, none of which a `/proc` microbenchmark sees.

The 80× `sockstat` win and the single-`stat`-read consolidation are still real
and still worth taking. They are just worth **~3% of 26% of 14.6%**, not the
headline. A microbenchmark measures what you point it at, and pointing it at the
syscalls is what makes everything else invisible.

### 6.4 What this run does not measure

**No sensors are attached to this bench.** Message rates are therefore near
zero, so §4.2's per-message interception cost — the term that scales with
traffic and is paid inside all 159 processes rather than in the supervisor — is
**absent from every number above**. On a vehicle with lidar and cameras
publishing, it is additive to this, and at 387 ns per `CLOCK_THREAD_CPUTIME_ID`
on ARM it is the term most likely to grow.

That also means these figures are a **floor** for the vehicle, not an estimate
of it.

### 6.5 Revised recommendation

1. **Upgrade the vehicle to 0.9.0.** 3.3× on the supervisor, 92× on parse,
   measured, no code change. Everything else on this list is smaller.
2. Re-measure interception on ARM and revisit the `events` default (§4.2) — the
   one term this run could not see.
3. The `/proc` work (§5 items 2–3) — real, cheap, and now correctly sized at a
   few percent of a quarter of the total.

### 6.6 Two measurement errors worth recording

Both produced confident wrong numbers before being caught.

**A sampling loop that assumes its own interval.** The first version slept 1 s
and divided by 1 s, while its own `pgrep`/`awk` over 160 processes made the real
interval **4980 ms**. Every CPU figure was inflated ~3–5× — 0.9.0 read as 60.3%
rather than 14.6%. It was caught only because system CPU printed **446% of 12
cores**, which is impossible. Sample loops must divide by *measured* elapsed
time; a harness heavy enough to measure is heavy enough to distort.

**`|| echo 0` after `pgrep -c`, for the fourth time in this repository.**
`pgrep -c` prints `0` *and* exits non-zero, so the fallback appends a second
zero and the comparison fails with `[: 0\n0: integer expression expected`.
Already recorded three times in `CLAUDE.md`; recorded again here because
re-committing it cost another run.
