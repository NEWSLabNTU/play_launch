# Reservations vs fixed priority: the three-arm result

**Status:** measured 2026-08-11, phase-60 W8.
**Claim under test:** do `SCHED_DEADLINE` reservations hold the safety chain's
deadline *while returning* the best-effort throughput that `SCHED_FIFO` costs?
Phase 57 measured that cost at 26–37 %.

**Answer: no, not on vanilla `rclcpp`.** Reservations return most of the
throughput and give up most of the determinism. Fixed priority remains the
better trade for this workload.

## Numbers

Same workload, same single CPU, 25 s per arm, first 200 samples discarded as
warmup. All three arms ran in one privileged container, so they are comparable
to each other.

| arm | p50 | p99 | max | missed (60 ms deadline) |
|---|---|---|---|---|
| RT OFF | 43.0 ms | 155.0 ms | 213.9 ms | **217 / 1013** |
| `SCHED_FIFO` | 13.1 ms | 19.9 ms | 198.7 ms | **9 / 1030** |
| `SCHED_DEADLINE` | 31.6 ms | 148.6 ms | 211.7 ms | **42 / 1038** |

Best-effort throughput, against the RT-off baseline:

| band | `SCHED_FIFO` | `SCHED_DEADLINE` |
|---|---|---|
| telemetry | −29 % | −14 % |
| tools | −14 % | −3 % |
| **all best-effort** | **−16 %** | **−5 %** |

So reservations cost best-effort roughly a third of what fixed priority costs,
and deliver roughly a fifth of the determinism benefit.

## Why — and what it is not

The obvious explanation is that the declared budgets are too small: each node
busy-burns for exactly `burn_ms`, but it also serializes and publishes, so the
true cost exceeds the reservation and CBS throttles the task.

**Tested, and rejected.** A fourth arm with budgets raised to the largest
values the declared deadlines admit (`obstacle_detector` 8000 → 11000 µs,
`brake_controller` 3000 → 4200 µs; 86 % of one CPU) was **worse**, not better:
59 misses and a 194.2 ms p99. More runtime did not help, so runtime exhaustion
is not what is hurting the chain.

The remaining explanation is a model mismatch, and it is the one the prior art
predicts. `SCHED_DEADLINE` implements CBS over a **sporadic release model**: a
task is expected to wake, consume up to `runtime`, and be replenished each
`period`. An `rclcpp::spin()` loop is not that. It is an event loop whose
releases are message arrivals, with no phase relationship to the reservation's
replenishment — so a message arriving after the current runtime is spent waits
for the next replenishment rather than for a CPU. That converts a scheduling
delay into a *quantised* one, which is exactly the shape of the observed
latency: a p50 that improves over RT-off (31.6 vs 43.0 ms) alongside a p99 that
barely moves (148.6 vs 155.0 ms).

Wilson et al. note the same inheritance — a per-callback design "inherits the
sporadic release model assumed by `SCHED_DEADLINE`" — and ROSRT's per-callback
threads exist precisely because a spin loop is not a sporadic task. Our F2
constraint sharpens it further: a reservation is per-**thread**, so only the
thread-group leader is reserved and the rest of the node's ~11 threads run
`SCHED_FIFO` beneath it.

**This is a limit of applying reservations from outside the process, not
evidence that reservations are the wrong mechanism.** Getting the benefit needs
an executor that releases each callback as its own schedulable entity — phase
58's side-track G, and not something the apply layer can reach.

## What the result does establish

- **Admission control works and the arithmetic is right.** 13 ms of declared
  runtime per 20 ms period (65 % of one CPU) was admitted; the kernel's default
  ceiling is 95 %.
- **Reservations bound the RT class's appetite.** That is the −5 % vs −16 %
  column, and it is the property W4 predicted: fixed priority takes whatever it
  wants until the global throttle intervenes, whereas a reservation takes what
  it declared.
- **The derivation is sound end to end.** Costs declared in the platform file
  become `runtime`; the chain's 50 Hz timer becomes `period` by propagation;
  contract deadlines become `deadline`. Applied verbatim:
  `lidar 2000/2000/20000`, `detector 8000/12000/20000`,
  `brake 3000/8000/20000` µs.
- **The validator earns its place.** Raising `lidar_driver`'s budget was
  rejected before launch: its contract declares `max_latency_ms: 2` and its
  burn is 2 ms, so it has *zero* slack and `runtime <= deadline` fails by
  construction. That is a real property of the demo's contract, surfaced as a
  spec error rather than an `EINVAL` after spawn.

## Caveats

- **The confinement mechanism is not held constant.** The `off` and `fifo` arms
  use `taskset`; the `deadline` arm cannot, because a deadline thread's
  affinity may not be narrower than its root domain, so it uses an exclusive
  cpuset partition. Same CPU, same workload, different mechanism — the one
  thing this comparison does not control for.
- **Container noise inflates every arm.** The same `fifo` arm run directly on
  the host earlier the same day gave 0/1005 misses and a 13.2 ms p99, against
  9/1030 and 19.9 ms here. Absolute numbers are worse in a container;
  the ranking between arms is internally consistent because all three ran in
  the same one.
- **One host, one workload, one run per arm.** No repetitions, no confidence
  intervals. This is a directional result, not a characterisation.
- **`SCHED_FLAG_RECLAIM` (GRUB) was not enabled.** Without it a reservation
  idles unused bandwidth rather than yielding it. It is the obvious next thing
  to try for the throughput column, and phase 60 deliberately deferred it.

## Reproducing

```bash
cd examples/rt_av_demo
just build
just ab            # two arms: off, fifo — unprivileged
sudo -E just ab3   # three arms: adds deadline — needs root for the partition
```

`just ab3` refuses rather than reports when it is not root, when the helper
lacks `CAP_SYS_NICE`, or when the RT-off baseline meets its deadline (nothing
for scheduling to fix). A run that cannot fail is not evidence.
