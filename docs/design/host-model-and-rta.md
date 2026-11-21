# Host model and response-time analysis (D1, D2)

**Status:** proposed, 2026-11-21. Design only; nothing here is implemented.
**Follows:** `docs/research/linux-realization-survey.md` §9, items D1 and D2.
**Pins read:** play_launch `main` at b575d951, rlm v0.3.29 (9e3f2c4).

## 1. Problem

A contract's path `max_latency` and an event chain's budget are promises.
Today play_launch derives priorities from them and applies those priorities,
but no step shows that the priorities keep the promise:

- `chain-latency` (`manifest_loader.rs::check_scope_path_critical_path`)
  sums a per-hop cost along the critical path. That cost is
  `GlobalNode::traversal_latency_ms` (`manifest_graph.rs:109`): the
  sampling period plus the path's **declared** `max_latency`. A promise is
  checked against the sum of other promises.
- The plan (`rate_monotonic`, `deadline_monotonic`, `chain_aware`) orders
  priorities. No step computes a worst-case response time R and compares it
  with the deadline.
- Nothing records the host the plan ran on. A 6.12 kernel adds a deadline
  server above every `SCHED_FIFO` priority; a 6.8 kernel throttles the RT
  class instead. A non-RT kernel has unbounded non-preemptible sections. The
  same plan has different worst cases on each, and the model cannot tell
  them apart.

rlm already has the vocabulary for the answer. `SourceKind::Analysed`
means "an upper bound from an analysis (WCET, response time, link
schedule)", and `BudgetLevel::Bound` is what it earns
(`types/src/types.rs:359`). Nothing produces an analysed latency.

D1 records the host. D2 produces the analysis. D2 depends on D1, because
every Linux-specific term in the analysis is a fact about the host.

## 2. Goals and non-goals

Goals:

- Every path `max_latency` and every event chain budget gets a **guarantee
  class** (survey §8a) and a list of the **premises** it rests on.
- A premise names its kind (declared, configured and read back, or
  measured) and where it came from.
- The analysis is the textbook one, applied soundly. Where the system
  falls outside its assumptions, the result is a lower class with the
  reason, never a number that looks proved.
- The arithmetic is runtime-agnostic and lives in rlm. The Linux terms
  live in play_launch. nano-ros can supply Zephyr terms to the same
  arithmetic.

Non-goals for this design:

- Global (multi-CPU) fixed-priority analysis. v1 analyses entities pinned
  to one CPU (§5.3).
- Reservations. `SCHED_DEADLINE` stays an overrun-isolation tool (survey
  §3); D2 does not analyse a deadline-scheduled node's own latency.
- Measuring premises. D3 turns `measure`, `rtla` and fault counters into
  recorded premises. D2 consumes a premise however it was produced and
  says which are missing.
- Provisioning. play_launch still creates no partition and changes no
  sysctl.

## 3. D1: the host model

### 3.1 What is recorded

One record per host, read once at the start of a run and by
`play_launch verify`. Each field carries its value **and** where it was
read, or why it could not be read. Absent is never zero.

| Field | Read from | Why the analysis needs it |
|---|---|---|
| kernel release | `uname(2)` | which terms apply (6.12 fair server, 6.13 lazy preemption) |
| preemption model | `/sys/kernel/realtime` (RT); `uname -v` (`PREEMPT_RT`, `PREEMPT_DYNAMIC`); `/sys/kernel/debug/sched/preempt` when readable | whether kernel blocking is boundable at all |
| RT throttle | `/proc/sys/kernel/sched_rt_runtime_us`, `sched_rt_period_us` | pre-6.12: the RT class stalls for `period - runtime` per period |
| fair server | release ≥ 6.12; debugfs `sched/fair_server/cpuN/{runtime,period}` when readable (*path unverified*) | 6.12+: a deadline entity above every FIFO priority |
| RR slice | `/proc/sys/kernel/sched_rr_timeslice_ms` | already a platform fact (`resources.rr_timeslice`); probing it removes the hand-written copy |
| CPUs and SMT | `/sys/devices/system/cpu/{online,smt/active}`, per-CPU `topology/thread_siblings_list` | an SMT sibling is an interferer no scheduler term covers |
| boot isolation | `/proc/cmdline`; `/sys/devices/system/cpu/{isolated,nohz_full}` | whether the tick, RCU callbacks and managed IRQs can reach a CPU |
| cpuset partition | `execution/cpuset.rs` (exists: `partition_state`, `effective_cpus`) | the CPU set an entity is analysed on |
| IRQ placement | `/proc/irq/*/smp_affinity_list`, `/proc/interrupts` | which IRQs (or IRQ threads) can preempt on a CPU |
| frequency | `cpufreq/scaling_governor`, `scaling_{min,max}_freq` per CPU | a measured cost is valid only at the frequency it was measured at |
| memory | THP mode, swap present, `RLIMIT_MEMLOCK` | page-fault premises |
| cgroup v2 | mounted controllers; PSI present | evidence channels, and what the cgroup layer can enforce |

Reading a debugfs file needs root on most hosts. An unreadable field is
recorded as `unreadable: <errno>`, which is a different fact from "the
kernel has no such feature".

### 3.2 Two copies, on purpose

The host model exists twice, and the difference between them is the
point.

- **Assumed:** what the analysis assumed. It is written in the platform
  file, beside the facts that already live there (`resources:` is
  documented as "platform facts"). A user writes it by hand or captures it
  from a host: `play_launch verify --host-model > host.yaml`.
- **Observed:** what the probe read at the start of a run. It is written
  to `play_log/<ts>/host_model.json`.

At run start the two are compared field by field. A field that differs
makes every guarantee resting on it **stale**, and the run says so once,
naming the field and both values. D3's `check --against <run>` reads the
same comparison from the bundle.

Without an assumed copy, `check` analyses against the probed host when it
runs on the target, and otherwise reports the Linux terms as missing
premises. It never assumes a default kernel.

### 3.3 Where the types live

- The record type goes in rlm's `sched` crate as `PosixHost`, a field of
  `PosixResources`. It is a platform fact for `target: posix`, and nano-ros
  never reads it.
- The probe goes in play_launch (`execution/host_model.rs`). It reads
  `/proc` and `/sys` only: no ROS, no privileges, no writes.
- The model carries the assumed copy when it is resolved with a platform
  file, so `up` (which reads the model and never the platform file) can
  compare too.

### 3.4 First consequence

`docs/guide/rt-scheduling.md` describes the 95% RT throttle. On a 6.12+
kernel without `RT_GROUP_SCHED` the mechanism is the fair server. The
guide is corrected once the probe has been run on a 6.12+ host, and it
names the kernel each paragraph applies to.

## 4. Guarantee classes and premises

From the survey (§8a), ordered from strongest:

| Class | Meaning |
|---|---|
| proved | every premise is declared or configured-and-read-back |
| proved, measured premises | at least one premise is measured, with its artefact and conditions |
| supervised | not proved; a run-time monitor detects violation and a reaction fits the FTTI |
| measured | observed over a run, no bound beyond it |
| best effort | configured, no claim |

The first, third and fourth map onto phase 93's `BudgetLevel` (`Bound`,
`Supervised`, `Unsupervised`). The second is new and is the class almost
every Linux result lands in, since cost and kernel blocking cannot be
proved there.

A premise:

```yaml
- name: cost(/perception/detector, on_scan)
  kind: measured            # declared | configured | measured
  value: 8.08ms
  source: play_log/2026-11-20-.../interception/events.jsonl
  conditions: { runs: 1039, governor: performance, kernel: 6.8.0-138 }
```

A result's class is the weakest class among the premises it used. A
missing premise does not lower the class; it stops the analysis for that
entity, and the result is `best effort` with `missing: [<premise>]`.

## 5. D2: response-time analysis

### 5.1 The schedulable entity

Linux schedules threads. The contract describes paths (callbacks). The
analysis needs to know which callbacks share a thread, and it needs every
thread on the CPU, including the middleware's.

One **executor entity** per thread that runs callbacks:

- a node under a single-threaded executor: one entity holding every path
  of the node (rclcpp's default callback group is mutually exclusive, and
  `GlobalNode` already derives groups from `concurrency:`);
- composables in one container under `observable` or `stock`: one entity
  per container thread, since composables share the container's executor
  (`ContractSources::process_per_composable` tells the two apart);
- a composable under `isolated`: its own process, one entity.

Inside an entity, callbacks run to completion in the order the executor
picks. To each other they are **non-preemptive at equal priority**.

The node's **other threads** (DDS receive and event threads, logging) get
the node's policy from the per-TID sweep, by design (Phase 38). They are
on the same CPU at the same priority and the contract cannot name them.
v1 models them as one interferer per process, with cost and period as a
premise. Without that premise the analysis stops for every entity on the
CPU, because omitting it is exactly the unsound step most published ROS 2
analyses take (survey §6).

### 5.2 Job streams

Each path of an entity is a job stream with:

- **C**: cost, from the platform file's `budget` (`DeriveFacts::exec_ms_for`).
  A per-node budget is attributed only to a single-path node, as today.
  The premise kind follows the budget's `source:` (`measured` for a
  `measure` fragment, `analysed` for a WCET tool, `declared` otherwise).
- **T**: minimum inter-arrival time. A timer path has its period. An
  input-triggered path has one stream per input topic, since a callback
  fires once per message on each topic (the fan-in rule of phase 68). The
  stream's T is the subscriber's `arrival_curve` when declared, else the
  upstream publisher's period **minus its release jitter**. A derived
  `rate_hz` is a mean and is not used as T.
- **J**: release jitter. A timer's is a premise (the timer's release
  lateness, measurable by `timerlat`). An input's is the upstream path's
  response-time spread, R − R_best, taken from the analysis itself.
- **D**: the path's `max_latency` if declared, else T.

### 5.3 The equation

For job stream i of an entity at priority p, pinned to CPU k, the worst
response R_i is the least fixed point of

```
w = C_i + B_i
    + Σ_{j ∈ hp(i)} ⌈(w + J_j) / T_j⌉ · C_j        higher or equal priority on CPU k
    + I_platform(w)                                 the platform's terms (§5.4)
R_i = w + J_i
```

computed by the usual iteration from w = C_i, stopping with "infeasible"
as soon as w + J_i > D_i (Joseph and Pandya; Audsley et al.; Tindell and
Clark for jitter).

- `hp(i)` holds every other stream at a higher priority on CPU k, and every
  stream of **another** entity at the **same** priority. `SCHED_FIFO` does
  not preempt between equals, but it orders them by arrival, so an equal
  that arrived first runs first. Counting them as interferers is the safe
  reading.
- `B_i`, blocking, is the largest of:
  - the longest other callback of the same entity (it may have started an
    instant before i was released, and runs to completion);
  - the longest lower-priority non-preemptive section on CPU k (the
    platform's kernel term, §5.4);
  - a lock held by a lower-priority thread. rclcpp and DDS locks are not
    priority-inheriting, so this term is a premise (declared or measured);
    without it the analysis stops.

**Single CPU only in v1.** The equation is a uniprocessor result. An
entity is analysed only when its threads are confined to one CPU: a
`cpus:` override with one CPU, or a one-CPU partition. Anything else is
reported as `best effort, reason: not pinned to one CPU`, never analysed
as if it were. Global fixed-priority bounds (Bertogna and Cirinei) are a
later item.

**Holistic iteration.** A chain's downstream J depends on the upstream R,
and an upstream entity on the same CPU depends on the downstream one.
Jitters start at their premise values and the whole system is iterated
until no R changes (Tindell and Clark's holistic analysis). It is
monotone, so it either converges or crosses a deadline.

### 5.4 Platform terms

`I_platform` and the kernel blocking term are what make the result
Linux's, and each comes from D1's host model.

| Term | Kernel | Value | Premise kind |
|---|---|---|---|
| fair server | ≥ 6.12 | an interferer above every FIFO priority, C = runtime, T = period on CPU k (default about 50 ms per 1 s) | configured (read back) |
| RT throttle | < 6.12 | the RT class stalls `period − runtime` per period (default 50 ms per 1 s); same arithmetic as above | configured |
| neither | `sched_rt_runtime_us = -1` | no term; the run warns that RT can starve every fair task | configured |
| kernel blocking | PREEMPT_RT | a bound on non-preemptible kernel sections | measured (`rtla timerlat` max, with run length and load) |
| kernel blocking | not RT | no sound bound | analysis stops: `best effort, reason: kernel not PREEMPT_RT` |
| IRQ threads | PREEMPT_RT | IRQ threads above p whose affinity includes CPU k; C and T per IRQ | measured (`osnoise`) |
| SMT sibling | SMT on | not a scheduler term; a premise that the sibling is offline or idle | configured or declared |

**The fair-server term decides most results.** 50 ms per second on CPU k
means any response window that crosses a server activation grows by up to
50 ms. A path with a 10 ms deadline cannot be proved against it. Two ways
remain, and the analysis says which one each result used:

- tune the server down on the partition's CPUs (root, debugfs), which the
  host model then reads back as a configured premise; or
- declare that no fair-class task runs on CPU k. The server is active only
  while fair tasks are runnable there. The premise is measured
  (`osnoise` thread noise, or `sched_switch` on that CPU), and per-CPU
  kworkers make it hard to hold.

The pre-6.12 throttle gives the same 50 ms per second, so this is not a
new problem; it is one the current plan ignores on every kernel.

### 5.5 Chains

rlm's composition rules stay as they are: series sum, fork-join max, and
the sampling period at each timer boundary. What changes is the per-hop
number. Where an analysed R exists for the path, the critical path charges
the sampling period plus R. Elsewhere it charges the declared
`max_latency`, as today, and the chain's class falls to the weakest hop.

The seam is one function: `GlobalNode::traversal_latency_ms` takes the
analysed R when the index holds one. The reaction walk
(`walk_reaction_at`) reads the same function, so FTTI rows get the same
treatment.

### 5.6 Outputs

- Per path: R, its class, the binding term (which interferer or premise
  dominates), and the missing premises.
- Diagnostics:
  - `rta-infeasible` (error): R > D, naming the terms of R;
  - `rta-incomplete` (info): the analysis stopped, naming the missing
    premise or the unmet assumption;
  - `rta-stale` (warning): the assumed and observed host models differ in
    a field this result used.
- `check --sched --explain` gains an R column beside the priority, and
  `chain-latency` names each hop's class.
- The model carries R per path as a derived bound with a `Derivation`
  record (rlm phase 91 E1), so `measure` and the runtime observer can
  compare a run with the analysed bound, and nano-ros can read it.

### 5.7 Where the code lives

- rlm, new module `sched::rta`: job streams, interferer list, blocking
  terms, the fixed point and the holistic loop. Pure arithmetic over a
  task set; no Linux in it. A Zephyr task set uses the same code with its
  own platform terms.
- play_launch, `ros_launch_resolve::ros::sched_rta`: builds the task set
  from the model, the plan and the platform file, adds the Linux terms from
  the host model, and writes the results into the index.

## 6. Evaluation

The analysis is useful only if it is sound (R is never exceeded) and the
class is honest. Both are tested against a running system.

1. **Unit:** textbook task sets with published response times, plus one
   case per term (blocking, jitter, fair server, equal priorities).
2. **Soundness on `rt_av_demo`:** its nodes busy-burn a fixed `burn_ms`,
   so cost is known by construction (phase 58 W2). Pin the chain to one
   CPU, take the costs from `measure`, compute R, run under load, and
   compare with the observed response maximum (`measure` reports it). The
   predicted R must bound every observation in every run.
3. **Negative control:** remove one interferer from the task set (the
   load generator, or the DDS threads' term). The prediction must then
   fall below an observation. If it does not, the run does not test the
   term, and the result is reported as unexercised rather than as passed.
4. **Tightness:** report R against the observed maximum per path. A large
   ratio is not a failure, but it tells the user what the safe reading
   costs.
5. **Hosts:** the 6.8 desktop exercises the RT-throttle path and the
   non-RT stop. The fair-server path needs a 6.12+ kernel, which neither
   test host has. Until one is available, that term is tested only by unit
   test, and the guide says so.

## 7. Open questions

1. **Placement of `PosixHost`.** rlm's `sched` crate (proposed) or a
   play_launch-only type. The first is needed for the model to carry it.
2. **Equal priorities.** The safe reading counts equals as interferers.
   With `rate_monotonic` ties are common, so results will be pessimistic
   until the mapper breaks ties. Should the mapper break them?
3. **The DDS-thread term without `measure`.** v1 stops. A declared default
   per RMW would let a user get a result sooner, at the price of a number
   nobody checked.
4. **Multi-CPU partitions.** Global FP bounds are pessimistic but sound.
   Worth a second wave, or leave multi-CPU entities at best effort until
   HCBS?
5. **Tie to phase 93.** "proved, measured premises" has no `BudgetLevel`.
   Add one to rlm, or carry the premises beside `Bound`?

## 8. Order of work

1. D1 probe and record; `verify --host-model`; host model in the run
   bundle.
2. Assumed copy in the platform file; the comparison at run start.
3. rlm `sched::rta` with unit tests.
4. play_launch task-set builder and Linux terms; diagnostics.
5. Chain seam (`traversal_latency_ms`), `--explain` columns, model
   carriage.
6. Evaluation on `rt_av_demo` (soundness, negative control, tightness).
7. RT guide correction, once a 6.12+ host is available.

## References

See the survey's reference list. The analysis results used here:
Joseph and Pandya (1986), Audsley et al. (1993), Tindell and Clark
(holistic analysis, 1994), Sha, Rajkumar and Lehoczky (priority
inheritance, 1990), Bertogna and Cirinei (global FP, 2007). Bibliographic
details are as given in the survey, with the same *unverified* marks.
