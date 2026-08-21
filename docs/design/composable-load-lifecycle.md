# Failure detection, retry, and constructors that take minutes

Design for phase 64 W2. Status: **implemented** (2026-08-22). Two corrections
the implementation forced are marked inline; the rest shipped as designed.

Phase 64 W1 gave play_launch a lossless channel to its own container and, with
it, removed five mechanisms that existed to survive a congested rmw layer. That
removal was correct and it left a hole: **on the socket path nothing ever gives
up, and nothing can ever be retried.** This design closes the hole without
re-introducing what was removed.

The whole problem is one question asked about a composable that has not
reported: *is it slow, or is it gone?* Every wrong answer is expensive in a
different direction, and the two costs are not symmetric.

## What each wrong answer costs

**Calling a slow load lost, and reloading it.** A constructor that takes
minutes is normal on the machines this tool targets: Autoware's traffic light
classifier builds a TensorRT engine in ~33 s cold and ~45 s with the engine
cached (measured, one Orin); phase 64's own reference run had
`voxel_based_compare_map_filter` still constructing at 135 s. Re-dispatching
one of those does not cost a retry — it costs a **second process doing the same
expensive work**, at the moment the machine is least able to afford it, and if
the first one was in fact alive, the system now has two nodes with the same
name publishing to the same topics. The `.engine` is written last, so a killed
or duplicated TensorRT build also discards the work and repeats it on every
relaunch (that is issue #0019's shape).

**Calling a lost load slow, and waiting.** One node silently never exists. The
launch reports `82/84 loaded (1 pending)` forever, `Startup complete` never
fires, and a `play_launch up` in CI hangs. Bad, but **visible and bounded** —
and the operator can act, because the web UI already exposes a per-composable
load button.

Bias follows from the asymmetry: **prefer waiting and reporting over retrying,
and never retry on a clock.**

## The hazard this replaces (service path, today)

`rescue_lost_loads` fires when an entry has been `Loading` for more than
`total_budget` (600 s) with no `unique_id`, verifies with `ListNodes`, and on
`Absent` **re-dispatches the load**. But the isolated container's
`on_list_nodes` deliberately hides ids in `pending_node_ids_` — nodes that are
mid-construction. So for a composable that is merely slow, `ListNodes` answers
`Absent`, which is indistinguishable from lost, and the resend double-loads it.

Reaching that state needs a lost LoadNode response *and* a >600 s constructor.
The phase 64 report had **14 lost responses in one launch**, so the first half
is routine on an edge machine; only the 600 s made it rare. The socket removes
the first half entirely, and this design removes the ambiguity that makes the
second half dangerous.

## The principle: ask the owner, don't infer from silence

The container owns the child process. It knows what the supervisor cannot:
whether an id was ever accepted, whether its worker has picked it up, whether
the fork happened, whether the child is alive, and how much CPU it has burned.
Today it *volunteers* some of that (`constructing` every 15 s). It cannot yet
be **asked**, and asking is what makes a retry safe.

Two message pairs close it:

```
supervisor -> container   { "t": "query",  "unique_id": 42 }          # or "seq"
container  -> supervisor  { "t": "status", "unique_id": 42,
                            "phase": "queued|constructing|loaded|failed|unknown",
                            "pid": 41213, "elapsed_ms": 135000,
                            "cpu_ms": 132400, "plugin": "…" }

supervisor -> container   { "t": "cancel", "unique_id": 42 }
container  -> supervisor  { "t": "load_failed", "unique_id": 42,
                            "error": "cancelled by supervisor", "cancelled": true }
```

`unknown` is the only answer that permits a resend, and it is a **positive
statement by the process owner** ("I have no record of this id, and nothing is
running for it"), not an inference from a timeout. `cancel` is what makes a
retry possible in the cases where one is wanted: it is a two-step — kill,
confirm gone, *then* resend — so a double load is unrepresentable.

`status` also closes a silent window that exists today: a composable held in
`await_spawn_capacity` (the container's memory gate, up to 120 s) has been
`accepted` and emits nothing at all until its fork happens, because the 15 s
liveness report only starts after the child exists. `phase: "queued"` names
that state, and the container should also volunteer a heartbeat while in it.

## State machine (socket path)

| state | entered by | supervisor action | may retry? |
|---|---|---|---|
| `Requested` | `load` written | wait `T_ack` | **yes** — see below |
| `Queued` | `accepted`, phase `queued` | wait, report | no |
| `Constructing` | first `constructing` frame | wait forever, report every 30 s | no |
| `Loaded` | `loaded` | done | n/a |
| `Failed` | `load_failed` / `rejected` | report | only per `max_load_attempts` |
| `Crashed` | `crashed` (after `Loaded`) | report | yes — id confirmed gone |
| `Lost` | `status: unknown` | resend | **yes** |
| `Unknown` | no answer to `query` | report, escalate, degrade | no |

Exactly two states permit an automatic resend, and both are safe by
construction rather than by budget:

- **`Requested` with no `accepted` within `T_ack` (default 5 s), confirmed by
  `status: unknown`.** No id was assigned, so no worker was queued and no
  process was forked. There is nothing to double.
- **`Lost`/`Crashed`** — the container states that nothing is running for that
  id. For `Crashed` it has already reaped the child and erased it from
  `children_`.

Everything else waits. `Constructing` has **no deadline**, which is the same
rule the container's own ready-wait already follows: it is bounded by liveness,
not by time, because there is no number that is right on every platform.

## Timers, and what each may do

| timer | default | expiry means | action |
|---|---|---|---|
| `T_ack` | 5 s | the `load` frame or the container's dispatch is broken | `query`; `unknown` → resend (safe), else adopt the reported phase |
| `T_report` | 45 s (3× the container's 15 s cadence) | reporting stopped, in any phase | `query`; adopt the answer; no answer → `Unknown` |
| `T_probe` | 15 s | how often to re-`query` an `Unknown` | re-probe, warn on a backoff |
| `T_stall` | **0 = never** | policy-only: alive but not progressing | see below |

None of these convert elapsed time into failure while liveness holds. `T_ack`
and `T_report` only ever cause a *question*; the answer decides.

## Stalls: evidence, not a clock

A node that is wedged and a node that is slow both fail to report a result. The
only extra evidence available is CPU: `cpu_ms` deltas from
`/proc/<pid>/stat` (utime+stime), which the container can read as cheaply as it
already reads `starttime` for the PID-reuse guard.

A stall is therefore defined as **all** of:

- the child is alive, and
- `elapsed > stall_after_secs`, and
- `Δcpu_ms` over the last two reports is below `stall_cpu_threshold_pct` of one
  core.

Even then, zero CPU is **not proof of a wedge**: a constructor legitimately
blocked on a service that has not come up yet (a map loader, a parameter
server) burns nothing while behaving correctly, which is the ordinary shape of
an Autoware startup. So:

- `stall_action: report` (**default**) — say it, loudly, with pid, elapsed and
  the CPU evidence. Change nothing.
- `stall_action: fail` — `cancel` it; the container kills the child and
  confirms; the entry goes `Failed`. No automatic resend beyond
  `max_load_attempts`.
- `stall_action: restart` — `cancel`, wait for the confirmation, then resend.
  Only reachable through a confirmed teardown.

`stall_after_secs` defaults to `0` (never) for the same reason
`PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS` defaults to no deadline: a fixed
number here is a guess about hardware we do not have, and when it is wrong it
SIGKILLs a node that was working.

## Degraded modes

The design must say what happens when the channel itself is the thing that
broke, because "no answer" is a third state and must not be read as either
outcome (the same absent-versus-zero rule phase 60 applied to costs).

1. **Socket EOF / write failure.** The container is gone or unreachable. The
   actor already learns this from `child.wait()`; composables go `Blocked`, the
   container respawns, and the reload happens through the normal respawn path
   with a fresh channel. No per-composable retry logic is involved.
2. **Container alive, socket silent** (a bug in the reader thread, a full
   pipe). The channel is marked closed; *future* loads fall back to the
   LoadNode service, which is still there. For loads already in flight the
   supervisor re-enables the **`ListNodes` sweeps as a degraded evidence
   source** — with their original ambiguity, so `Absent` in that mode means
   "leave it Loading and report", never "resend". This is why the sweeps should
   be *disabled* rather than deleted.
3. **Thread-loaded composables (phase 65).** With per-composable isolation,
   an inline composable has no pid, no separate CPU accounting, and cannot be
   cancelled without taking its container's siblings down. `status` must
   therefore be able to answer `phase: constructing, pid: 0` with
   `cancellable: false`, and `stall_action` must refuse to act on those rather
   than silently doing nothing. A wedged inline constructor also blocks the
   container's executor, so the honest report is about the container, not the
   node.

## Startup completion

A launch where one composable constructs forever currently prints
`composable 82/84 loaded (1 pending)` every 10 s and never says
`Startup complete`. With liveness data available, the pending line should name
what it is waiting for and for how long, and after `startup.stage_timeout_secs`
the run should report **ready-with-exceptions** rather than staying silent
forever:

```
Startup complete with 1 exception: 83/84 composables loaded;
  '/perception/.../voxel_based_compare_map_filter' still constructing at 135s
  (pid 41213, 0.2% cpu) — not counted as failed
```

That is a reporting change, not a policy one: nothing is killed, nothing is
retried, and the operator gets a launch that says what it is doing.

## Config surface

```yaml
composable_node_loading:
  control_socket: true            # phase 64 W1
  ack_timeout_ms: 5000            # T_ack — the one safe automatic resend
  report_timeout_secs: 45         # T_report — silence triggers a query
  probe_interval_secs: 15         # T_probe
  max_load_attempts: 2            # total attempts; the 2nd needs a CONFIRMED absence
  stall_after_secs: 0             # 0 = never declare a stall
  stall_cpu_threshold_pct: 1.0    # evidence for a stall, not the trigger alone
  stall_action: report            # report | fail | restart
  composable_respawn: off         # off | on-crash  (see below)
```

**Correction 1 — `max_load_attempts` defaults to 2, not 1.** The draft said
`1 = none`, which contradicted the same document's "one safe automatic resend":
a load the container never acknowledged has forked nothing, and resending it is
free. The budget counts TOTAL attempts, so 2 means "one resend, and only after
a confirmed absence"; 1 disables resending entirely. The budget also bounds the
`stall_action: restart` cycle — without that, a node that stalls, is cancelled
and stalls again would cycle forever, each iteration individually safe and
collectively useless.

Every default is "wait and report". Nothing in this design changes behaviour
for anyone who does not opt in — which is the point, because the failure mode
of the aggressive setting is worse than the failure mode of the passive one.

## Crash after load

Separable, and worth stating because it is the one retry that is unambiguously
safe: a `crashed` frame means the container reaped the child and erased its id.
`composable_respawn: on-crash` would reload it after `respawn_delay`, bounded
by `max_respawn_attempts`, reusing the node-level respawn accounting so a
crash-looping composable cannot spin. Off by default until the crash-loop
accounting is shared, since a composable that crashes in its constructor and is
retried forever is a worse outcome than one that stays failed and says so.

**Correction 2 — a restarted load must not inherit the previous attempt's
events.** The ComponentEvent topic still delivers the CANCELLED attempt's
`LOAD_FAILED` after the replacement load has started, and the name-and-plugin
fallback matching (added in W1 for lost LoadNode responses) happily claimed the
fresh entry with it: measured, a restarted composable was marked `Failed` by
the outcome of the attempt that had just been killed. The fallback now applies
only to entries with no socket tracking — on the socket path the id is always
known, because the container states it in `accepted`, so a fallback there can
only ever match the wrong thing. Each attempt also gets its own clock; carrying
the elapsed time forward reported a fresh constructor as minutes old and, with
stall detection on, declared it stalled before it had been given any time.

## Invariants

The rules that make the above safe, stated so a future change can be checked
against them:

- **I1.** A resend requires a *confirmed absence* stated by the owner of the
  process. Never a timeout, never an inference from silence.
- **I2.** Time does not convert into failure while liveness holds. Only lost
  liveness, or an explicit operator policy, does.
- **I3.** Every wait is reported with its evidence — phase, pid, elapsed, CPU —
  so "slow versus stuck" is a human judgment with data rather than a guess by
  the tool.
- **I4.** Whoever owns the child kills the child. A retry is cancel → confirm →
  resend, so a double load is unrepresentable rather than merely unlikely.
- **I5.** "Cannot be measured" is its own state. It is never read as success
  (that is issue #0019) and never as failure (that is the double-load hazard
  above).

## Where the code is

| what | where |
|---|---|
| protocol v2: `query`/`status`/`cancel`, `phase`, `cpu_ms` | `src/play_launch/src/ipc/container_protocol.rs` |
| the policy: timers, probes, stalls, confirmed-teardown retries | `src/play_launch/src/member_actor/container_actor/load_policy.rs` |
| socket messages → transitions | `.../container_actor/control_events.rs` |
| pending-load map, `query`/`cancel` handlers, CPU sampling | `src/play_launch_container/src/clone_isolated_component_manager.cpp` |
| ordered frame backlog, fault injection | `src/play_launch_container/src/control_channel.cpp` |
| "waiting on" reporting | `src/play_launch/src/commands/signal_handler.rs` |
| tests | `tests/tests/load_policy.rs` |

The container carries a test-only fault injector,
`PLAY_LAUNCH_CONTROL_DROP=first_load,status`, because the two most
consequential branches — "the container never saw this load" and "the container
never answers" — cannot be reached by any legitimate input, and a branch that
decides whether to fork a second copy of a node is not one to leave untested.

## Tests

- A composable that never returns from its constructor: no retry, no failure,
  a liveness line every 30 s with rising elapsed, `ready-with-exceptions` at
  the stage timeout. (`SlowLoader` with a very large `load_delay_ms`.)
- A `load` frame dropped before acceptance (fault-injected): `T_ack` fires,
  `status: unknown`, exactly ONE resend, exactly one process.
- A slow constructor plus a forced `query`: the answer is `constructing`, and
  no resend happens — the regression test for the double-load hazard.
- `stall_action: restart` with a wedged node: cancel precedes the resend, and
  the second process starts only after the first is confirmed dead.
- Socket silenced with the container alive: loads in flight stay `Loading` and
  are reported; new loads go over LoadNode; nothing is resent.
