# Measured cost — turning a run into declared budgets

Status: Accepted (2026-08-13). Design of record for
[phase-58](../../roadmap/phase-58-scheduling-derivation.md) **W2**.
Depends on: phase-58 W1 (`budget_us` authorable), delivered in
[phase-60](../../roadmap/phase-60-linux-sched-surface.md).

## Problem

W1 made execution cost *authorable*. It did not say where the number comes
from, and asking an integrator to invent one is exactly how the
deadline-as-cost conflation arose in the first place: a plausible number was
already in scope, so it got used.

The data already exists. The interception layer timestamps every `rcl_publish`
and `rcl_take` with `CLOCK_MONOTONIC` and correlates them by header stamp;
phase 57's exporter already builds those flows for `chrome://tracing`. Nothing
turns them into a cost.

## The quantity — and why one number will not do

`budget_us` becomes a `SCHED_DEADLINE` **runtime**, which is a CPU-time
reservation. The obvious measurement — elapsed time from a path's input take to
its output publish — is **response time**: it includes preemption, blocking on
other callbacks, and DDS wakeup latency. Feeding it into a reservation
over-declares, wasting admission bandwidth that is shared with every
`SCHED_FIFO` thread in the root domain.

The roadmap half-acknowledged this ("over-states cost unless measured on an
idle system"), but a quiet machine is a mitigation, not a fix: even idle, the
figure carries wakeup latency that is not the node's execution.

So W2 measures **both**, and keeps them apart:

$$\text{cost}_{\text{cpu}}(P,S) = \text{cpu\_ns}(\text{publish } o \in P.\text{output},\, S)
  - \text{cpu\_ns}(\text{take } i \in P.\text{input},\, S)$$

$$\text{response}(P,S) = \text{mono\_ns}(\text{publish}) - \text{mono\_ns}(\text{take})$$

`cost_cpu` is what `budget_us` takes. `response` is reported beside it, because
it is what a *deadline* is about and because the gap between the two is the
preemption the reader should see.

Attribution comes from the contract: `PathDecl.input` / `.output` name the
endpoints, so a (node, take-topic, publish-topic) triple lands on a named path.
The header stamp is the only identity a publish and its take share.

## Mechanism

Read `CLOCK_THREAD_CPUTIME_ID` inside the two existing hooks and carry it in
the event.

```
struct InterceptionEvent {          // 40 -> 48 bytes
    kind, _pad, topic_hash,
    stamp_sec, stamp_nanosec,
    handle,
    monotonic_ns,                   // wall  -> response
    cpu_ns,                         // NEW: thread CPU -> budget
}
```

A private ABI between our `.so` and our consumer, shipped together, so growing
it costs nothing external.

**Rejected: sampling `/proc/<pid>/stat`.** It yields total CPU over a run, so
dividing by invocations gives a *mean* and never a distribution — and it is
per-process, so a node with two declared paths cannot be split. Both defeat the
purpose.

**Rejected: `getrusage(RUSAGE_THREAD)`.** Separates user from kernel time,
which would be informative, but its resolution is traditionally jiffy-granular
(~1–4 ms) — too blunt for burns measured in single-digit milliseconds.

## The statistic

`budget_us` takes the observed **maximum**. p50 and p99 are reported alongside,
as comments, for judgement.

This is the safety-relevant choice. Under CBS an invocation that exceeds its
runtime is throttled until the next replenishment, so a p99 budget converts the
slowest 1% of invocations into a full-period stall — and on a safety chain those
are precisely the invocations that were already slow. Max is also the only one
of the three statistics that is an upper bound on what was actually observed,
which is what "used as an upper bound" has to mean.

Deliberately NOT max × a margin: inventing a number on top of a measurement is
the failure mode this phase exists to remove, and any margin would have no
basis in anything measured.

## Deliverable

```
play_log/<ts>/interception/events.jsonl     <- new artifact
system_model.yaml                            <- paths and their endpoints
        |
   play_launch measure <run-dir> --model <model.yaml>
        |
   a YAML fragment on stdout, never written back
```

A twelfth verb. `contract measure` was tempting because `contract eject`
already emits pasteable YAML, but this reads a *run*, not a contract; `plot`
already reads a run but is about pictures.

### Output

```yaml
# play_launch measure — play_log/2026-08-13-04-21-07
# host <hostname> · <n> CPUs · kernel <ver> · <dur> s · --sched-apply <mode>
#
# budget_us is the observed MAXIMUM cpu time from a path's input take to its
# output publish. p50/p99 are shown for judgement, not pasted: under CBS an
# invocation exceeding runtime is throttled to the next period, so a p99 budget
# turns the slowest 1% into a full-period stall.
#
# This is what this machine did on this run. It is NOT a WCET.

overrides:
  obstacle_detector:
    # detect:  cpu p50 6.9  p99 8.2  max 9.1 ms   n=1198
    #          response p99 11.4 ms (incl. DDS wakeup + preemption)
    budget_us: 9100

# brake_controller / brake: not measurable — 1204/1204 messages had no header.stamp
# map_loader / load:        not observed in this run
```

Units change three times across this boundary — the trace measures nanoseconds,
the contract thinks in milliseconds, the field is microseconds. That is one of
[phase-59](../../roadmap/phase-59-timing-vocabulary.md)'s motivations; here it
is handled by converting once, at render.

### Paths that cannot be measured still appear

Omitting them would read as "no cost", which is the absent-versus-zero
confusion this phase exists to kill. Three classes, each named in the output
with its reason:

| class | why |
|---|---|
| timer-triggered | no input take to anchor against |
| unstamped | stamp 0 cannot be correlated (the `std_msgs/String` case) |
| not exercised | declared, but no matching pair observed in this run |

### Multi-path nodes

`overrides` is node-keyed; costs are per-path. A node with two measurable paths
has no single correct budget — it depends on whether both fire within one
period, which the tool cannot know. It emits the **sum** of per-path maxima,
with the breakdown in comments and the assumption stated. Conservative,
consistent with choosing max, and visible.

This is phase-60's open question 1 resurfacing. A `costs:` section keyed
node → path is the eventual answer; summing is the honest interim.

## Boundaries

| unit | responsibility |
|---|---|
| `play_launch_interception` (`.so`) | read the thread CPU clock at the two existing hooks |
| `interception/mod.rs` | write `events.jsonl` for the run |
| `interception/measure.rs` *(new)* | **pure**: events + path declarations → per-path statistics. No I/O |
| `commands/measure.rs` *(new)* | CLI wiring and fragment rendering |

The pure core takes parsed events and returns statistics, so the part worth
testing has no filesystem in it.

## Testing

Unit tests cover pairing by stamp, percentile computation, multi-path
summation, and each of the three unmeasurable classes.

The end-to-end test uses ground truth. `rt_av_demo`'s nodes busy-burn for
**exactly** `burn_ms` (`rt_av_demo::burn_ms`), so the fixture is an oracle:

| node | declared burn | measured `cpu` max must fall in |
|---|---|---|
| `lidar_driver` | 2.0 ms | [1.8, 3.0] |
| `obstacle_detector` | 8.0 ms | [7.5, 10.0] |
| `brake_controller` | 3.0 ms | [2.7, 4.5] |

This validates hook → transport → correlation → statistics against numbers we
control. A test asserting only that "a number came out" would catch nothing.

## Errors

- No `events.jsonl` → error naming `interception.enabled: true` as the fix.
- Model carries no contracts → error; there are no paths to attribute to.
- Every message unstamped → still emits, every path marked not-measurable.
  An empty file would be indistinguishable from "no cost anywhere".

## Risks

- **The measurement may distort what it measures.**
  `clock_gettime(CLOCK_THREAD_CPUTIME_ID)` is a real syscall, twice per
  message, on a hot path. This has to be measured rather than assumed; if the
  overhead is material the design is self-defeating and the mechanism needs
  revisiting.
- **Measured is not WCET.** One machine, one run, one workload. The output
  header says so; nothing downstream should read it as a proof.
- **Contention inflates `response`.** `cpu` is largely immune but not perfectly
  — cache pressure is real. Measure quiet, apply busy.
- **Thread-local blind spot.** Thread CPU time misses work a node fans out to
  another thread or defers to a timer, so such a node under-reports. Stated in
  the output rather than papered over.

## Open questions

1. Should `events.jsonl` be gated behind its own config flag rather than
   written whenever interception is enabled? It is per-message data, so a long
   run produces a large file. Proposed: write it by default, revisit if size
   bites; the alternative is a flag nobody remembers to set before the run they
   wanted to measure.
2. Does `response` belong in the fragment at all, or only in a report? It is
   not pasteable — no field takes it. Kept as a comment because the gap between
   it and `cpu` is the most informative thing on the page.

## Non-goals

Static WCET analysis. Writing measurements back into a platform file
automatically. Per-callback attribution below the path level (that needs
executor cooperation — phase-58 side-track G). Changing what `budget_us` means.
