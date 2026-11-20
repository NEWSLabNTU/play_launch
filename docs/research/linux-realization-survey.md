# Realizing a Contract on Linux: Mechanisms, Guarantees and Gaps

**Status**: Research
**Date**: 2026-11-20
**Prompted by**: how far a contract's timing and safety requirements can be
mapped onto modern Linux (PREEMPT_RT, the scheduler classes, cgroup v2,
CPU isolation, interference control) with a stated theoretical guarantee
for each step, rather than as best-effort configuration.
**Feeds**: the design work that follows this survey (section 9).
**Related**: [scheduling-derivation-prior-art.md](scheduling-derivation-prior-art.md)
(priority assignment, deadline decomposition, reservations, LET, synthesis;
not repeated here),
[../reports/rt-mixed-criticality/reservations-result.md](../reports/rt-mixed-criticality/reservations-result.md),
[../design/cgroup-per-container.md](../design/cgroup-per-container.md),
[../guide/rt-scheduling.md](../guide/rt-scheduling.md),
rlm `docs/contract-theory.md` and `docs/scheduling.md`.

## Summary

A contract is a pair (A, G): assumptions on inputs, guarantees on outputs.
Today play_launch *realizes* a contract by deriving a scheduling plan and
applying it, but it never discharges the guarantee. A node's `max_latency`
is a promise that nothing shows the Linux configuration keeps, and nothing
records which properties of the host that promise silently relies on.

Three findings frame the rest of this survey.

1. **On Linux, a hard guarantee is a theorem with measured premises.** Every
   result that can be proved (fixed-priority response-time analysis, CBS
   isolation, EDF admission, compositional supply bounds) needs at least one
   input no tool can prove on Linux. These inputs are the cost of a callback,
   the kernel's non-preemptible blocking, and the interference through
   caches and memory. The honest form of a guarantee is "proved, given these
   declared facts, these configured facts read back from the kernel, and
   these measured bounds with their conditions". This extends the
   bound/supervised/unsupervised levels phase 93 already gives a budget.

2. **The mechanism set changed under us.** Several properties the current
   realizer and guide assume have moved:
   - PREEMPT_RT is mainline since 6.12.
   - Since 6.12 the fair server (a per-CPU deadline server) replaces root
     RT throttling. It preempts every `SCHED_FIFO` thread for about 5% of
     each CPU, and pre-6.12 analyses omit that term.
   - EEVDF replaced CFS, and `sched_runtime` on a fair task is now a slice
     request.
   - `PREEMPT_LAZY` arrived in 6.13, and 7.0 drops `PREEMPT_NONE` and
     `PREEMPT_VOLUNTARY` on x86 and arm64.

   The two hosts we test on are a 6.8 desktop and a 5.15-tegra Orin, both
   before every one of these changes. A realizer that does not check the
   kernel version and its features cannot state which of the analyses
   applies.

3. **The right abstraction for a ROS node is not upstream.** A node is about
   11 threads. CBS reserves per thread, and the measured W8 result showed
   that applying it from outside the process loses to `SCHED_FIFO`.
   Hierarchical CBS (HCBS), which gives a cgroup one deadline server with
   FIFO threads inside, is exactly the shape a node needs and has a
   compositional theory (the periodic resource model). It is still an
   unmerged RFC (v6, 2026-06).

The practical consequence: the strongest sound mapping available today is
**fixed-priority response-time analysis on an isolated cpuset partition**,
with cost and kernel blocking as measured inputs. Deadline reservations are
an **overrun-isolation tool**, not a latency tool. Cache, memory bandwidth,
I/O and network are **bounded by measurement or best effort**. A full proof
chain for an ASIL-D path exists only off Linux, on a partitioning hypervisor
with an RTOS cell, which is nano-ros's track.

## Method and labels

Kernel status was checked against kernel.org documentation, LWN and LKML
as of 2026-11. Each claim below is labelled:

- **THM**: a theorem, proven under stated assumptions.
- **IMPL**: what the kernel implements, which may diverge from the theory.
- **EMP**: empirical, known only by measurement.

Items marked *unverified* were not confirmed against a primary source and
must be checked before a design depends on them. The classic-paper DOIs
were given from memory and are marked the same way.

## 1. What the realizer does today

Established by reading play_launch and rlm at v0.3.29, not from the
literature.

| Stage | Where | What it does |
|---|---|---|
| Facts | rlm `derive` | Per node: fastest timer rate, min deadline over paths and services, hazard-derived criticality, `claims_concurrency`, cost from the platform file's `budget` (absent stays absent) |
| Plan | rlm `sched` mappers | `rate_monotonic`, `deadline_monotonic`, `chain_aware`, `manual`; overrides win; `SCHED_DEADLINE` derived only under `reservations: required` |
| Validation | rlm `validate`, play_launch `sched.rs` | Band violations, rate/deadline vs priority contradictions, chain sampling-cost feasibility, DL admission `Σ runtime/period < M · rt_runtime/rt_period` |
| Apply | `play_launch_rt_helper` | `sched_setattr` on every TID; affinity; uclamp; nice; reset-on-fork only for DL |
| Isolation | `cpuset.rs`, `cgroup.rs` | Detects a cpuset partition by readback, never creates one; per-node and per-container cgroups with memory, pids, `cpu.weight`, `cpu.max` |
| Run time | runtime enforcement, interception | Latency, rate, age, FTTI detection and reaction observed; phase 93 classifies each budget as bound, supervised or unsupervised |

Gaps in that pipeline, before any Linux question:

- **No response-time analysis.** The plan ranks priorities, but no step
  computes a worst-case response time R and compares it with the deadline D.
  The chain feasibility check sums sampling periods, and treats a missing
  cost as zero (`ChainFeasibleWithoutWcet` makes that visible). Nothing
  shows R ≤ `max_latency` for any node.
- **cgroup limits are not derived from the contract.** CPU and memory limits
  come from a hand-written `cgroups.limits` config, matched by glob. A
  hazard-derived integrity level does not reach `memory.min`, `cpu.weight`
  or a partition.
- **No host model.** Nothing records or checks the facts every analysis
  assumes:
  - the kernel version and preemption model;
  - whether the fair server is present, and its parameters;
  - whether `sched_rt_runtime_us` is still a throttle;
  - `nohz_full`, `rcu_nocbs`, IRQ and workqueue placement;
  - SMT, frequency governor, THP, `mlockall`.

  The RT guide still describes the 95% RT throttle, which on a 6.12+
  kernel without `RT_GROUP_SCHED` is the fair server instead (to verify on
  a 6.12+ host before the guide is changed).
- **Independence stops at the scheduler.** `not_sharing: [process,
  container, cpus]` is checked against the model and the platform file.
  Caches, memory bandwidth, the DDS shared-memory transport and IPIs are
  not in the vocabulary at all.
- **Threads are not modelled.** A node's DDS receive and event threads
  inherit its policy (by design: the per-TID sweep). No analysis accounts
  for them, and no contract can name them.

## 2. PREEMPT_RT

**Status.**
- Merged for 6.12 on x86, arm64 and riscv, behind `CONFIG_EXPERT`
  ([LWN](https://lwn.net/Articles/992185/)). The last blocker was nbcon
  printk.
- Upstream documentation:
  [core-api/real-time](https://docs.kernel.org/core-api/real-time/index.html).

**What it bounds (IMPL, EMP).**
- `spinlock_t` and `rwlock_t` become rt_mutex sleeping locks with priority
  inheritance.
- IRQ handlers are forced into threads (`SCHED_FIFO` 50 by default, settable
  per `irq/N-*` thread).
- Softirqs run in task context.

Most of the kernel becomes preemptible. What remains non-preemptible is
`raw_spinlock` regions, hard-IRQ entry, non-threaded IRQs (per-CPU timer,
IPIs, `IRQF_NO_THREAD`), NMIs and SMIs.

**What it does not give.** There is no proof for the kernel as a whole.
- The closest formal artefact is de Oliveira's automata model of
  PREEMPT_RT scheduling latency
  ([ECRTS 2020](https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.ECRTS.2020.9)).
  It decomposes latency into IRQ, NMI, preempt-off and IRQ-off windows, and
  needs measured window bounds.
- `rtla timerlat` implements that decomposition at run time
  ([rtla](https://docs.kernel.org/tools/rtla/index.html)).
- `hwlat_detector` finds SMI and firmware gaps the OS cannot see.

The kernel blocking term is therefore always a measured premise.

**Points that matter for an analysis.**
- **PI is transitive** (`FUTEX_LOCK_PI`, rt_mutex), so blocking is bounded
  by the priority inheritance protocol (THM, Sha, Rajkumar and Lehoczky
  1990). Chained blocking is allowed, so the blocking term is a sum over
  resources. There is no kernel priority ceiling.
- **Middleware locks bypass PI.** rclcpp, DDS and allocator locks are
  mostly plain mutexes (*unverified per RMW*), so PI does not reach them.
- **`migrate_disable()`** pins a task without disabling preemption. This
  makes the system non-work-conserving, so global-scheduling theory holds
  only approximately on RT.
- **Proxy execution** would give mutexes bandwidth inheritance, including
  across `SCHED_DEADLINE`. It is partly upstream
  ([LKML 2025-07](https://lkml.indiana.edu/hypermail/linux/kernel/2507.2/00500.html))
  and excludes `sched_ext` at build time. Do not rely on it.
- **Serial consoles** remain a latency source until their driver is nbcon.
  The 8250 conversion was at v7 in 2026-07 (*merge unverified*).
- **Preemption models.** `PREEMPT_LAZY` (6.13) changes only fair tasks.
  From 7.0, x86 and arm64 reject `PREEMPT_NONE` and `PREEMPT_VOLUNTARY`, so
  RT-class threads are always fully preemptible. A non-RT kernel still has
  long non-preemptible sections, so its worst case cannot be bounded.

## 3. The scheduler classes

Order of precedence: stop, deadline (and the deadline servers), RT
(FIFO/RR), fair (EEVDF), `sched_ext`, idle.

### SCHED_DEADLINE (CBS over EDF)

[Documentation](https://docs.kernel.org/scheduler/sched-deadline.html).

**Theorems.**
- **Uniprocessor or partitioned (THM).**
  - EDF meets every deadline when utilisation is ≤ 1 (D = P).
  - CBS isolation (Abeni and Buttazzo, RTSS 1998; *DOI unverified*): a
    reservation (Q, P) never takes more than Q per P, whatever its client
    does.
  - Its supply bound function makes the client analysable alone.
- **Global EDF (THM).** Utilisation below M guarantees only **bounded
  tardiness** (Devi and Anderson), not deadlines. The kernel's admission
  test, `Σu ≤ M · rt_runtime/rt_period` per root domain, is necessary, not
  sufficient, for hard global EDF.

**Implementation (IMPL).**
- A deadline thread's affinity may not be narrower than its root domain,
  so hard per-CPU reservations need an exclusive cpuset partition.
- `SCHED_FLAG_RECLAIM` (GRUB) reclaims idle bandwidth, within admitted
  bandwidth (THM for the algorithm).
- `SCHED_FLAG_DL_OVERRUN` sends SIGXCPU on overrun. This is a run-time
  evidence channel the realizer does not yet read.

**Why W8 lost.** CBS assumes sporadic jobs, and an `rclcpp` spin loop is
one job that never ends: the reservation's deadline is postponed and the
task degrades towards background service. The isolation theorem still
holds, so *other* nodes were protected from this node's overrun. The
node's own latency was not.

**Consequence.** Use a reservation for what its theorem gives (bounding a
node's effect on others) and not as a latency mechanism, until either:
- an executor releases callbacks as sporadic jobs; or
- HCBS lands.

### The fair server (6.12)

- Root-level RT throttling was replaced by a per-CPU deadline server that
  runs `SCHED_NORMAL` tasks when they are starved, about 50 ms per 1 s per
  CPU by default ([LWN](https://lwn.net/Articles/943242/)).
- It is a deadline entity, so it sits **above every FIFO priority**.
- For fixed-priority RTA it is a new highest-priority interferer (C = 50 ms,
  T = 1 s per CPU, deferred, active only while fair tasks are starved).
- `sched_rt_runtime_us = -1` disables it, and so re-admits RT starvation of
  everything else.
- It has had real defects: a big.LITTLE starvation bug fixed in 2025-07,
  and a reported 2026 latency regression under I/O load (*status
  unverified*).
- Tuning goes through debugfs `sched/fair_server/cpuN/` (*path
  unverified*).

### HCBS (hierarchical CBS for cgroups)

- The Andriaccio and Abeni RFC is at v6 (2026-06), **not merged**
  ([patchew](https://patchew.org/linux/20260608121546.69910-1-yurand2000@gmail.com/),
  [LWN](https://lwn.net/Articles/1024757)).
- Each cgroup gets one deadline server, and the FIFO/RR threads inside it
  are scheduled by priority.
- The theory is the periodic resource model and compositional analysis
  (Shin and Lee, RTSS 2003): a group's supply bound function lets each
  group be analysed alone (THM).
- This is the shape that removes F2: a node's reservation covers all of its
  threads, and the node's own priorities order them inside it. It is the
  most important upstream item to watch.

### EEVDF, sched_ext, uclamp, core scheduling

- **EEVDF** ([doc](https://docs.kernel.org/scheduler/sched-eevdf.html))
  - Bounded lag relative to other fair tasks (THM for the idealised
    algorithm). No absolute bound, because every RT and DL entity preempts
    it.
  - Since 6.12, `sched_runtime` on a fair task requests a slice of
    0.1 to 100 ms. Latency-nice was not merged.
  - Right for QM (best-effort) nodes: a short slice gives a lower expected
    wakeup latency, not a bound.
- **sched_ext** (6.12): a BPF class *below* RT and DL, with a watchdog that
  falls back to fair. It cannot give an RT guarantee; it is for best-effort
  policy experiments.
- **uclamp**: a hint to frequency selection and energy-aware scheduling.
  No timing guarantee. Its indirect value is that it keeps a frequency
  drop from inflating a measured cost.
- **Core scheduling**: a security feature. It adds interference rather than
  removing it. For timing, take the SMT sibling offline instead.
- **`RT_GROUP_SCHED`**: legacy throttling with weak semantics, which HCBS
  is meant to replace. Avoid it.

## 4. CPU isolation

| Mechanism | Removes | Settable at run time |
|---|---|---|
| cpuset v2 partition `root` | load balancing across the boundary, foreign tasks; own DL root domain | yes (root writes `cpuset.cpus.partition`) |
| partition `isolated` | load balancing inside too; unbound workqueues | yes |
| `nohz_full=` | the periodic tick while one task runs (a residual 1 Hz tick remains) | boot only |
| `rcu_nocbs=` | RCU callback execution | boot only |
| `isolcpus=managed_irq` | managed (multi-queue) IRQs, which ignore user affinity | boot only |
| `/proc/irq/*/smp_affinity`, `irqaffinity=` | other device IRQs | yes |
| workqueue `cpumask` | unbound kworkers | yes |
| `timer_migration` | unpinned timers on idle or isolated CPUs | yes |

Notes on the table:
- Moving `nohz_full` and managed IRQs to run time through isolated
  partitions is open on LKML (Waiman Long, 2026-04; DHM v3, 2026-06). Until
  it merges, a launch-time tool can *detect* full isolation but cannot
  *establish* it.
- **None of these removes**:
  - IPIs, including TLB shootdowns caused by threads of the same process on
    other CPUs, which a node spread across CPUs causes itself;
  - per-CPU `vmstat` and LRU drain work;
  - SMIs and NMIs;
  - cache and memory contention.

**Evidence.**
- `rtla osnoise` attributes per-CPU noise to IRQ, softirq, thread, NMI and
  hardware.
- `rtla timerlat` measures wakeup latency.
- `/proc/interrupts` deltas show what reached an isolated CPU.
- The partition readback (`root invalid`) shows whether the partition is
  real, and play_launch already reads it.

## 5. Interference beyond the CPU

**Cache and memory bandwidth.**
- `resctrl` ([doc](https://docs.kernel.org/filesystems/resctrl.html))
  provides:
  - Intel CAT (LLC way partitioning; capacity isolation, timing effect EMP);
  - MBA (approximate: delay values are not bandwidth, as its own
    documentation says);
  - CMT and MBM monitoring.
- Arm MPAM's base driver merged in 6.19. Its resctrl glue was still in
  review in 2026, and Orin support is unknown (*unverified*).
- MemGuard (per-core bandwidth regulation, Yun et al. RTAS 2013) gives a
  bound under stated assumptions, but is out of tree. So is PALLOC (DRAM
  bank partitioning).
- Mainline has no cache colouring; Xen 4.20, Bao and Jailhouse do.
- On mainline Linux, memory-interference bounds come only from
  measurement.

**Memory.**
- `mlockall(MCL_CURRENT | MCL_FUTURE)` plus prefaulting removes page faults
  after startup.
- cgroup v2:
  - `memory.min` is hard protection from reclaim;
  - `memory.low` is best-effort protection;
  - `memory.high` throttles by reclaim, which **injects latency into the
    node itself** and so must never be set on a real-time node;
  - `memory.max` ends in OOM.
- Transparent huge pages (THP) cause compaction stalls of milliseconds
  (EMP); use `madvise` with `defrag=never` or turn them off.
- PSI is evidence of stalls where it exists. It does not exist on
  5.15-tegra.

**I/O.** `io.max` (hard throttle), `io.latency` (target with peer
throttling) and `io.cost` (proportional). None is a theorem. Real-time
nodes should not do synchronous I/O; logging and recording belong on
housekeeping CPUs.

**Network.**
- `SO_PRIORITY` maps traffic through `mqprio` to hardware queues.
- `taprio` (802.1Qbv) gives time-aware shaping, which is a theorem at the
  network level given gPTP and a full TSN domain.
- Threaded NAPI turns receive processing into a thread that can be given a
  priority.
- RPS, XPS and RFS move packet processing off isolated CPUs.
- The host software path is measured only.

**Power and SMT.**
- PM QoS (`/dev/cpu_dma_latency`, per-CPU `pm_qos_resume_latency_us`) caps
  C-state exit latency. The `performance` governor or `scaling_min_freq`
  fixes frequency.
- On Jetson, `nvpmodel` and `jetson_clocks` own frequency, the memory
  controller's included.
- **A measured cost is valid only at the frequency, power mode and thermal
  state it was measured in.** Nothing in the kernel signals that these
  changed except trace events.
- An SMT sibling slows its partner by 10 to 50% (EMP). For hard timing,
  take it offline.

## 6. Theory that applies, and what each result needs

| Result | Gives | Needs | Linux realization |
|---|---|---|---|
| FP response-time analysis with blocking (Joseph and Pandya; Audsley; Sha et al.) | R_i ≤ D_i per thread (THM) | cost C_i, arrival bound T_i, blocking B_i, partitioned | `SCHED_FIFO` on a cpuset partition; add the IRQ-thread, fair-server and kernel-blocking terms |
| Chain composition (series sum, fork-join max, sampling at timer boundaries; Davare et al., Becker et al.) | end-to-end latency and data age (THM given per-hop R) | per-hop R, periods | already rlm's composition rules, fed from RTA instead of declared `max_latency` |
| CBS isolation | an overrun stays inside its reservation (THM) | runtime ≥ C for the node's own deadline | `SCHED_DEADLINE` per thread; HCBS per group later |
| Periodic resource model (Shin and Lee) | a group analysed alone, from its supply bound (THM) | group (Q, P), per-thread C inside | HCBS (unmerged); Xen RTDS |
| Bounded tardiness under global EDF (Devi and Anderson) | lateness bound, not deadline (THM) | Σu ≤ M | DL on a multi-CPU root domain; fits a soft `max_age` with slack |
| Mixed criticality (Vestal 2007; Burns and Davis survey) | HI deadlines under LO overrun | a mode change on overrun | no bounded mode change on Linux; CBS isolation protects HI without one, which is the main argument for reservations despite W8 |
| ROS 2 executor analyses (Casini 2019, Blaß 2021, PiCAS 2021, Teper 2024) | chain bounds for executor semantics (THM) | per-callback C, executor semantics per rclcpp version, a DDS thread model (usually omitted, which is unsound) | callback-per-thread designs reduce to FP-RTA per thread, the cleanest mapping |
| Measurement-based probabilistic WCET (Cucu-Grosjean 2012) | exceedance probability | i.i.d. execution times | violated on commodity hardware under Linux; a measured cost is a high-water mark plus margin, which is what `measure` produces |

**What a tool can prove, given its inputs:**
- schedulability of a declared task set under FP, EDF or CBS on a
  partition;
- end-to-end chain bounds;
- admission;
- independence by configuration (no shared process, cgroup, CPU or CAT
  ways);
- QoS compatibility.

**What it can only measure:**
- cost;
- the kernel blocking term;
- SMIs;
- cache, bandwidth and DDS interference;
- the absence of page faults;
- frequency effects;
- whether the kernel's EDF and CBS conform to their theory.

Kernel runtime verification
([RV](https://docs.kernel.org/trace/rv/index.html)) runs automata monitors
over scheduler events, a path to checking the last item while the system
runs.

## 7. Related systems

- **AUTOSAR Adaptive Execution Management**
  - Execution and machine manifests declare processes, function-group
    states, resource groups (cgroup-like), scheduling policy, priority and
    affinity. Platform Health Management supervises alive, deadline and
    logical properties.
  - Configuration only; analysis is external (TIMEX and tools).
  - The closest analogue to play_launch, and it does not attempt the proof
    either.
- **AADL with Ocarina**: generates POSIX threads from thread properties.
  The proof lives in Cheddar or MAST, not the OS.
- **ROS 2 Real-Time Working Group** ([ros-realtime](https://ros-realtime.github.io/)):
  - Guidance: `mlockall`, static allocation, callback-group threads with
    FIFO priorities, PREEMPT_RT.
  - The reference system is benchmark-based and offers no guarantee.
- **Apex.OS**: a certified ROS 2 derivative with a deterministic executor
  and no allocation after init. Details are proprietary.
- **Kubernetes CPU manager (static)**: exclusive cores by cpuset for
  integer requests. Isolation by exclusivity, no RT policy.
- **systemd resource control**
  - Exposes `CPUSchedulingPolicy`, `CPUAffinity`, `AllowedCPUs`,
    `MemoryMin` to `MemoryMax`, `IOWeight` and `Delegate=`.
  - It is the natural root-side provisioning path for what play_launch
    deliberately refuses to create: partitions and controller delegation.
  - Whether it writes `cpuset.cpus.partition` is *unverified*.
- **Partitioning hypervisors** (Jailhouse, Bao, Xen with RTDS and LLC
  colouring) put an RTOS cell beside Linux with spatial and temporal
  isolation. On commodity multicore this is **the only route today to a
  full proof chain alongside Linux**, and memory bandwidth still needs
  MemGuard-style regulation.
  - nano-ros on Zephyr is that cell: static priorities, no paging, much
    smaller unknown terms.
- **ELISA** (safety-critical Linux) argues in the same assumption and
  evidence style this survey recommends
  ([2026 update](https://directory.elisa.tech/annual-update/2026-02/Linux_Features_WG_Updates_2026.pdf)).

## 8. Synthesis

### 8a. Guarantee classes

Five classes, ordered from strongest. The first three already have
counterparts in phase 93's budget levels (bound, supervised, unsupervised).

| Class | Meaning |
|---|---|
| **proved** | A theorem whose premises are all declared or configured-and-read-back |
| **proved, measured premises** | A theorem with at least one measured premise (cost, kernel blocking), each carrying its artefact and the conditions it was measured under |
| **supervised** | Not proved; violation is detected at run time and a reaction fits the FTTI |
| **measured** | Observed over a run; no bound beyond the run |
| **best effort** | Configured, no claim |

A premise is one of:
- **declared**: the author states it;
- **configured**: the realizer set it and read it back from the kernel;
- **measured**: with an artefact, a run length and its conditions
  (frequency, power mode, co-runners).

### 8b. Contract quantity to mechanism

| Contract quantity | Mechanism(s) | Best class reachable | Premises | Run-time evidence |
|---|---|---|---|---|
| path `max_latency`, event chain budget (hard) | FIFO priorities from RTA, on an isolated partition; IRQ threads placed; `mlockall`; fixed frequency | proved, measured premises | PREEMPT_RT; cost; kernel blocking (`timerlat`, `hwlat`); every thread known, DDS included; PI on shared locks; fair-server term; SMT sibling off | interception take-to-publish max against the bound; `timerlat` on those CPUs; no unexpected preemptor in `sched_switch` |
| path `max_latency` (soft) | FIFO, no partition | measured | as above, without isolation | percentiles and max |
| `budget` (overrun isolation) | `SCHED_DEADLINE` per thread; HCBS per node later | proved (others are protected); the node's own deadline only if runtime ≥ cost | exclusive root domain; sporadic job structure, or HCBS | DL_OVERRUN count; throttle statistics |
| `min_rate`, timer rate | timer thread under FIFO or DL | proved if the producer's R ≤ its period | as for latency | interception rate windows; QoS deadline-missed events |
| `max_jitter` | partition, `nohz_full`, `rcu_nocbs`, IRQ affinity, PM QoS, no THP | proved, measured premises (jitter ≤ WCRT − BCRT) | both bounds | `osnoise`; interception spread |
| `max_age` | chain RTA plus sampling periods; or LET | proved given per-hop bounds | stamp propagation; PTP across hosts | interception data age (phase 88 C3) |
| FTTI (FDTI + FRTI) | DDS deadline and liveliness; detector and reaction at top priority on reserved CPUs | proved arithmetic; detection delivery measured | detector not co-located with the fault; RMW event-thread priority | `hazard-detected`, `hazard-reaction` timings |
| integrity level (ASIL) | criticality-monotonic split: HI in the RT band or DL, QM in EEVDF with slice and nice | proved for CPU demand; not for memory, cache, I/O or kernel state | QM holds no lock HI needs; fair-server share accounted | per-class CPU; `cpu.pressure` on the HI cgroup |
| `independence: process` | separate processes (`isolated`) | proved by configuration, for crashes | no shared memory segments (the DDS SHM transport shares them) | pid map; `/proc/*/maps` scan |
| `independence: container`, memory | cgroup per group; `memory.min` for HI; `memory.oom.group`; `oom_score_adj` | configured (reclaim never takes HI below `memory.min`) | delegation; no `memory.high` on HI | `memory.events`; PSI where present |
| `independence: cpus` | cpuset partitions; IRQ and workqueue masks | proved by configuration for the scheduler only | partition readback valid; managed IRQs at boot | partition readback; `/proc/interrupts` deltas; `osnoise` |
| cache, memory bandwidth (not in the vocabulary) | CAT, MBA, MPAM; MemGuard | configured capacity (CAT); measured or best effort otherwise | hardware support; one resctrl group per partition | CMT and MBM counters |
| I/O | `io.max`, `io.latency`, `io.cost` | best effort | | `io.stat`; PSI io |
| network (cross-host `max_transport`) | `SO_PRIORITY`, `mqprio`, `taprio`; threaded NAPI with a priority | proved at the network level with TSN; host stack measured | TSN NIC; DDS sets priority | `tc -s`; NIC timestamps |
| determinism premises | `mlockall`, prefault, THP off, no swap; fixed frequency | configured | | `VmLck`; zero `minflt`/`majflt` deltas after startup; cpufreq and thermal events |

### 8c. Gaps with no sound Linux mapping today

1. **No reservation for a multi-threaded node.** CBS is per thread and HCBS
   is unmerged.
2. **The kernel blocking term is unprovable.** Every proved latency carries
   a measured `timerlat`/`hwlat` maximum, valid only for the workload and
   run length observed.
3. **Memory and cache interference.** Only CAT capacity and approximate
   MBA/MPAM exist upstream. Without bandwidth or DRAM-bank isolation,
   `not_sharing` cannot make a complete independence claim for HI and QM
   on one host.
4. **No bandwidth inheritance.** PI across `SCHED_DEADLINE` carries no
   bandwidth accounting, proxy execution is partial, and middleware locks
   are not PI. The blocking term for locks inside DDS is unknown.
5. **Executor and middleware threads are not modelled**, in the contract or
   in most published analyses.
6. **No bounded mode change for mixed criticality.** DL_OVERRUN plus a
   userspace demotion has unbounded latency.
7. **The fair server intrudes on FIFO.** It must enter the analysis.
   Disabling it brings back RT starvation of everything else.
8. **Isolation cannot be established at launch.** `nohz_full`, `rcu_nocbs`
   and managed IRQs are boot-time only.
9. **Global EDF gives tardiness, not deadlines.** Partitioned DL needs
   root-owned exclusive partitions, and `migrate_disable` weakens global
   assumptions further on RT.
10. **A measured cost has validity conditions nobody checks.** Frequency,
    thermal state, co-runners and input data all change it, and the kernel
    signals none of it.

## 9. Proposed design work

In dependency order. Each item states its acceptance in terms of a
guarantee class, not a feature.

- **D1. A host model.** Read the kernel version, preemption model, fair
  server, `sched_rt_runtime_us`, boot isolation parameters, partition state,
  IRQ placement, SMT, frequency governor, THP and lock state into one
  record. Carry it in the run bundle, and in the model when resolved for a
  target.
  - Every later item states its premises against this record, so a
    guarantee can say which kernel it holds on.
  - First consequence: correct the RT guide's 95% throttle text for 6.12+.
- **D2. Response-time analysis as a check.** Fixed-priority RTA per thread
  class on each partition, from the plan's priorities and the platform
  file's costs. It includes blocking, the fair-server term and a declared
  or measured kernel blocking bound. Feed the per-hop R into rlm's existing
  chain composition in place of the declared `max_latency`.
  - The output is the guarantee class of each `max_latency` and each event
    chain, with its premises listed.
  - This is the core of "solid theoretical guarantees", and it needs no
    kernel feature that is not already mainline.
- **D3. Premise evidence.** Turn `measure`, `rtla timerlat`/`osnoise`,
  `hwlat`, fault counters and DL_OVERRUN into recorded premises with their
  conditions. `check --against <run>` would then report a premise as
  violated when a run exceeds it, or as stale when the host model differs
  (another frequency governor, another kernel).
- **D4. Integrity-driven isolation.** Derive the partition plan,
  `memory.min`, the HI/QM class split, IRQ placement and the absence of
  `memory.high` from hazard-derived integrity instead of a hand-written
  config.
  - Provisioning stays root-side: a generated systemd drop-in or a script
    extending `provision_rt_cpuset.sh`. play_launch still creates nothing
    privileged.
- **D5. Vocabulary for what the contract cannot yet say**: threads (a
  node's executor and middleware threads); shared memory as a sharing
  dimension; cache and bandwidth as `not_sharing` dimensions. This belongs
  to rlm and nano-ros together, under the one-meaning-in-rlm rule.
- **D6. Watch items, not work.** Re-evaluate when each moves:
  - HCBS (the per-node reservation; would replace F2 and reopen W8);
  - dynamic isolation through isolated partitions;
  - proxy execution;
  - MPAM resctrl.
- **D7. The off-Linux route.** For a requirement only "proved" can satisfy
  (ASIL-D), state that Linux reaches "proved, measured premises" at best,
  and point at a hypervisor partition with nano-ros. This is a statement in
  the check output, not a feature.

## References

Primary documentation:
- [docs.kernel.org: real-time](https://docs.kernel.org/core-api/real-time/index.html),
  [sched-deadline](https://docs.kernel.org/scheduler/sched-deadline.html),
  [sched-eevdf](https://docs.kernel.org/scheduler/sched-eevdf.html),
  [sched-ext](https://docs.kernel.org/scheduler/sched-ext.html),
  [sched-util-clamp](https://docs.kernel.org/scheduler/sched-util-clamp.html),
  [cgroup-v2](https://docs.kernel.org/admin-guide/cgroup-v2.html),
  [no_hz](https://docs.kernel.org/timers/no_hz.html),
  [resctrl](https://docs.kernel.org/filesystems/resctrl.html),
  [psi](https://docs.kernel.org/accounting/psi.html),
  [rtla](https://docs.kernel.org/tools/rtla/index.html),
  [osnoise](https://docs.kernel.org/trace/osnoise-tracer.html),
  [timerlat](https://docs.kernel.org/trace/timerlat-tracer.html),
  [hwlat](https://docs.kernel.org/trace/hwlat_detector.html),
  [RV](https://docs.kernel.org/trace/rv/index.html),
  [pi-futex](https://docs.kernel.org/locking/pi-futex.html),
  [PM QoS](https://docs.kernel.org/power/pm_qos_interface.html),
  [NAPI](https://docs.kernel.org/networking/napi.html),
  [network scaling](https://docs.kernel.org/networking/scaling.html)

LWN and LKML:
- [PREEMPT_RT merge](https://lwn.net/Articles/992185/)
- [deadline servers](https://lwn.net/Articles/943242/)
- [HCBS](https://lwn.net/Articles/1024757) and
  [RFC v6](https://patchew.org/linux/20260608121546.69910-1-yurand2000@gmail.com/)
- [PREEMPT_LAZY](https://lkml.iu.edu/2411.2/01726.html)
- [proxy execution](https://lkml.indiana.edu/hypermail/linux/kernel/2507.2/00500.html)
- dynamic isolation: [Long 2026-04](https://lkml.iu.edu/2604.2/06386.html),
  [DHM v3](https://lkml.iu.edu/2606.2/04056.html)
- [MPAM driver](https://lkml.iu.edu/hypermail/linux/kernel/2512.0/03301.html)
- [fair server and `sched_rt_runtime_us=-1`](https://lkml.iu.edu/hypermail/linux/kernel/2503.0/08332.html)

Papers (DOIs from memory, to verify):
- de Oliveira et al., automata model of PREEMPT_RT latency, ECRTS 2020
  ([link](https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.ECRTS.2020.9))
- Abeni and Buttazzo, CBS, RTSS 1998
- Sha, Rajkumar and Lehoczky, priority inheritance protocols, IEEE TC 1990
- Shin and Lee, periodic resource model, RTSS 2003
- Devi and Anderson, tardiness bounds under global EDF, RTS 2008
- Vestal, mixed criticality, RTSS 2007; Burns and Davis,
  [MC review](https://www-users.york.ac.uk/~ab38/review.pdf)
- Yun et al., MemGuard, RTAS 2013; PALLOC, RTAS 2014
- Cucu-Grosjean et al., MBPTA, ECRTS 2012
- Blaß et al., [ROS 2 executor analysis, RTSS 2021](https://people.mpi-sws.org/~tblass/papers/rtss21-ros.pdf)
- Choi et al., [PiCAS, RTAS 2021](https://par.nsf.gov/biblio/10276465)
- Casini et al. and Teper et al.: see scheduling-derivation-prior-art.md

Systems:
- [ros-realtime](https://ros-realtime.github.io/)
- [Kubernetes CPU management](https://kubernetes.io/docs/tasks/administer-cluster/cpu-management-policies/)
- [systemd resource control](https://www.freedesktop.org/software/systemd/man/latest/systemd.resource-control.html)
- [Jailhouse](https://github.com/siemens/jailhouse),
  [Bao](https://github.com/bao-project/bao-hypervisor)
- [ELISA Linux features 2026](https://directory.elisa.tech/annual-update/2026-02/Linux_Features_WG_Updates_2026.pdf)

Unverified, to check before a design depends on them:
- the fair-server debugfs path;
- the venues of Tang et al., Sobhani et al. and Arafat et al.;
- the callback-per-thread attribution to Wilson et al.;
- the HCBS 2019 paper URL;
- the AUTOSAR EM specification URL;
- the MPAM resctrl merge release;
- the 8250 nbcon merge;
- the `sched_ext` deadline-server release;
- RV scheduler monitor versions;
- PI coverage of each RMW's internal locks.
