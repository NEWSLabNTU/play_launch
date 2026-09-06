# Phase 71 — fault detection and reaction

Status: **complete** (W1–W5, 2026-09-06; manifest crate `v0.1.30`).
Design of record: `docs/design/fault-reaction-primitives.md`.

The contract could say *"this data must arrive at ≥50 Hz"* and could not say
what happens when it does not, how fast that must be noticed, or how long
until the system is safe. ISO 26262's number for that is the fault-tolerant
time interval, and the finding of the design was that its three supervision
kinds — alive, deadline, logical — were already the three things the contract
declared. So the vocabulary is one requirement, one reaction edge, one fact,
and everything else is derived.

## W1 — vocabulary (manifest `v0.1.30`)

- `hazards.<h>`: `severity`, `description`, `guards` (bare topic or
  `{ all_of: [...] }`), `on` (closed: omission | late | loss | reported),
  `ftti`, `reaction` (a scope path).
- `sub.<e>.on_violation`: `on`, `reaction` (a path on this node), `within`,
  `mechanism` (closed: qos | diagnostics | application).
- `paths.<p>.safe_state`: `emits`, `settle`.

Fifteen field-table rows, each with a `kind`; four new contexts; model-side
mirrors (`HazardContract`, `OnViolationContract`, `SafeStateContract`).
Every existing contract resolved byte-identically.

## W2 — derivation and rules (resolver)

**FDTI** of a guard: the fastest detector among the guard's subscribers that
*react* — a subscriber that notices and does nothing has not detected
anything the system can use. `omission` → the liveliness lease; `late` → the
QoS deadline or `max_age`; `loss` → `drop.max_consecutive` × period;
`reported` → the detector's input period + its own path latency. An `all_of`
group is the slowest member (the fault is the last survivor noticed gone);
the hazard's budget must hold for whichever group faults.

**FRTI** of a reaction: a walk over the reaction edges, not the nominal
critical path. At the guard only an `on_violation` moves (the nominal path is
waiting for a message that will not come); from the first reaction on, the
walk follows `on_violation` where declared and otherwise ordinary
input-triggered paths, because a reaction is a real message and downstream
nodes forward it like any other. Ends at a `safe_state` on the sink, whose
`settle` is added. Fork-join takes the longest branch.

Rules: `fault-reaction-budget` (error above `ftti`, info with the slack
otherwise — and the message names every term), `hazard-unguarded`,
`reaction-unreachable` (three shapes: no such path, trigger does not include
the subscription, no reaction chain reaches the sink), `reaction-within`,
`reaction-unbudgeted` (no route or no settle: "INCOMPLETE EVIDENCE"),
`reaction-unguarded` (who watches the watcher — a warning, and the answer is
a second hazard whose guard is the first reaction's output).

**Two arithmetic defects, both mine, caught by the first fixture.** (1) A
rate floor was counted as a detector, so a 50 Hz `min_rate_hz` "detected" a
dead lidar in 20 ms while the real lease was 100 ms. `min_rate_hz` is a
requirement; nothing fires when a period passes. (2) The reaction route was
the generic critical path (42 ms): it charged the lidar's sampling period —
the lidar is dead — and the nominal callbacks rather than the reactions
(7 ms). Both fixed before the numbers matched the design's.

## W3 — verified on a running system

`measure` gained hazards: the fault is the guard's last publish, the
reaction is the first sink publish after it **that carries no upstream
provenance**. That clause was forced by the first run: the brake's nominal
response to the *last* scan arrives 11 ms after it wearing that scan's
stamp, and a naive "first publish after" reported 11 ms for a 100 ms lease.
The nominal pipeline forwards provenance; a reaction originates its own.

`rt_av_demo` became the oracle a third time: `die_after` stops the lidar's
data (the node stays alive), `watchdog_ms` is the lease applied by the
detector itself, the brake commands an emergency stop on a "lost" message.
`just fault`:

```
derived:  FDTI 100 + route 7 (+ settle 200)
observed: fault → reaction 104.17 ms   (oracle: lease 100 + route 7 + noise)
```

`measure` consumes `hazards`, `on_violation.within_ms`, `mechanism` and
`safe_state.settle_ms` from the model; the census stays at four.

Not done in W3: wiring `rclcpp`'s QoS event callbacks into
`runtime_enforcement` as a live FDTI observer. The `mechanism` field is where
that plugs in; `measure` reads the `/diagnostics` stream for
`mechanism: diagnostics` today.

## W4 — Autoware's MRM chain

Overlay contracts under `tests/fixtures/autoware/contracts/`, one per launch
file (`system`, `mrm_handler`, `mrm_emergency_stop_operator`, `control`),
every number from Autoware 1.5.0's parameter files:

```
detection 500ms  (mrm_handler: timeout_operation_mode_availability)
+ reaction route mrm_handler/call_mrm → mrm_emergency_stop_operator/stop
                 → vehicle_cmd_gate/gate_emergency = 244ms  (110 + 34 + 100)
+ settle 1200ms
= 1944ms   fits ftti 2s with 56ms of slack;  EXCEEDS ftti 1.5s
```

Three real nodes across three launch files, one `on_violation` edge and two
nominal hops — which is why W2's walk had to continue through nominal paths.
The test is guarded on an Autoware install like the parity tests.

## W5 — the adoption path

`play_launch check --emit diagnostics-params` prints `diagnostic_updater`
`FrequencyStatus`/`TimeStampStatus` parameters restated from every
subscriber's `min_rate_hz`/`max_rate_hz`/`max_age`. A consequence, printed,
never written back — the first thing a user with no hazard analysis gets from
declaring bounds they already have.

## Not done

- Live QoS event callbacks in `runtime_enforcement` (above).
- `all_of` guards on a running system: the demo has one lidar.
- The safe-state envelope (a reaction commanding −2.5 m/s² through a gate
  limited to −2.4): a value-domain check, declined in the design.
- Modes: Autoware's diagnostic graph is a mode graph; the walk works
  without one because a reaction is a path, not a mode transition.
