# Fault detection and reaction — the primitives worth folding in

Status: design, **proposed** (2026-09-06). Answers `contract-axes.md` §3.1
("adopt eventually — ISO 26262 requires it") and open question 6 ("is FTTI a
contract requirement or a hazard-level one?"). Builds on
`criticality-from-hazards.md` (hazards as the source of criticality) and
`contract-primitives.md` (facts and requirements, never consequences).

## The gap in one sentence

The contract can say *"this data must arrive at ≥50 Hz and be ≤60 ms old"*
and cannot say **what happens when it does not, how fast that must be
noticed, or how long until the system is safe.** Every rate requirement we
carry is a performance wish. ISO 26262 calls the number that turns it into a
safety requirement the **fault tolerant time interval**:

```
FTTI  ≥  FDTI (fault → detected)  +  FRTI (detected → safe state reached)
```

A hazard's FTTI comes from physics (how long a vehicle can drive blind). FDTI
and FRTI come from the software — and they are *consequences*: detection is a
QoS parameter the contract already carries, reaction is a route the graph can
already trace. The claim of this design is that most of the FTTI vocabulary is
**one requirement plus a derivation**, not a new sub-language.

## 1. Survey — what each source has, and which parts are ours to fold

The test for each primitive is the campaign's: is it a *fact* (what the code
does), a *requirement* (what it must achieve), or a *consequence* (derivable)?
Only the first two get a key. Every key gets a named consumer, or it does not
land (phase 70's rule).

### 1.1 ISO 26262

| concept | part | what it is | fold? |
|---|---|---|---|
| **FTTI** | 1, 3 | time from fault to hazardous event, absent reaction | **requirement**, on a hazard |
| **FDTI / FRTI** | 4 | the detect / react split of FTTI | **consequence** — derived from detector + route |
| **safe state** | 3, 4 | the operating mode the reaction reaches | **fact** on the reaction path: what it emits, and how long the plant takes to settle |
| **safety mechanism** | 4, 5 | the element that detects and reacts | a *node + path* we already describe; no key |
| **diagnostic coverage** | 5 Annex D | fraction of faults a mechanism detects | not folded — a hardware-metric concept; our detectors are omission/lateness only |
| temporal **freedom from interference** | 6 Annex D | one element's fault must not consume another's FTTI | already the mapper's job; FTTI gives it a number to check against |
| Part 6 timing monitoring: **alive**, **deadline**, **logical** supervision | 6 | the three supervision kinds | alive = `lease_duration`/`min_rate_hz`; deadline = `qos.deadline`/`max_age`; logical = our `trigger`/`output` structure. **All three already exist as facts.** |

The decisive observation: ISO 26262's three supervision kinds are the three
things the contract already declares. What is missing is not detection — it is
*what the detection is for*.

### 1.2 AUTOSAR Watchdog Manager (WdgM) and E2E

WdgM is the most concrete prior art, because it is a *shipped* specification
of exactly this vocabulary:

| WdgM | ours | note |
|---|---|---|
| `AliveSupervision`: expected indications per reference cycle, min/max margin | `min_rate_hz` / `max_rate_hz` on a subscriber | same shape, both bounds — which is why W2 added the upper one |
| `DeadlineSupervision`: min/max time between two checkpoints | path `min_latency` / `max_latency` | same shape; `jitter-range` is the min/max window |
| `LogicalSupervision`: allowed checkpoint graph | `trigger` → `output` causal structure | a run that publishes without its trigger is a logical-supervision failure; observable from interception, no key needed |
| `FailedSupervisionRefCycleTol`: how many failed cycles before EXPIRED | `miss: { tolerate: N / W }` | **weakly-hard**, already carried |
| global status EXPIRED → configured reaction (reset, safe state) | **missing** — the reaction | this is the fold |

E2E protection (counter, CRC, timeout, DataID) is transport integrity. The
timeout half is `lease_duration`; the counter half is `drop.max_consecutive`;
CRC and DataID are middleware concerns we do not describe. Nothing to fold.

### 1.3 AADL Error Model Annex (EMV2)

EMV2 contributes the **closed set of what a detector detects**. Its error
type library (`ServiceOmission`, `LateDelivery`, `EarlyDelivery`, `ValueError`,
`ItemOmission`, …) is the right vocabulary for the `on:` discriminator of a
reaction, and it is closed — which phase 69/70 taught us is the whole point.
We take the three we can detect from the middleware and decline the rest:

| EMV2 type | detected by | ours |
|---|---|---|
| `ServiceOmission` | liveliness lease expiring | `on: omission` |
| `LateDelivery` | QoS deadline / `max_age` violated | `on: late` |
| `ItemOmission` (consecutive) | `drop.max_consecutive` exceeded | `on: loss` |
| `ValueError`, `OutOfRange` | needs semantic inspection | **not folded** |

EMV2's *error propagation paths* and *recover transitions* are what our
derived route already is — a reaction path is an error propagation from the
detector to the actuator.

### 1.4 STPA

Leveson's four unsafe-control-action types are a useful cross-check that the
closed set above is complete for a *control* system: **not provided**
(omission), **provided too late/early** (late), **stopped too soon / applied
too long** (a duration property we do not carry). Three of four map; the
fourth is a value-domain question, declined with `ValueError`.

### 1.5 ROS 2

The runtime hooks exist and are unused by us: `on_deadline_missed`,
`on_liveliness_lost`/`changed`, `on_incompatible_qos` are subscription and
publisher **event callbacks**. `runtime_enforcement` already observes rate
and age; wiring it to these events makes it the FDTI observer for free.
Enforcement remains opt-in on the node side (`QosOverridingOptions`,
`contract-axes.md` §4) — derive always, apply where accepted, report where
not.

### 1.6 Autoware — the reaction chain that is already in the corpus

This is the evidence that the shape is right, because Autoware *implements*
FDTI + FRTI end to end with numbers in parameter files, and none of those
numbers can be written in our contract today:

```
topic_state_monitor      warn_rate 5.0 / error_rate 1.0 / timeout 1.0 s   ← FDTI, per topic
   ↓ diagnostic
diagnostic_graph_aggregator   and/or graph → /autoware/modes/{emergency_stop,comfortable_stop,pull_over}
   ↓ mode availability
mrm_handler              update 10 Hz, timeout_call_mrm_behavior 0.01 s     ← FRTI, part 1
   ↓ MRM request
mrm_emergency_stop_operator   30 Hz, target_acceleration −2.5 m/s²          ← FRTI, part 2 + safe state
   ↓ control_cmd
vehicle_cmd_gate         system_emergency_heartbeat_timeout 0.5 s,
                         emergency_acceleration −2.4, lon/lat limits          ← safe-state ENVELOPE
```

Three things worth naming. (1) Detection is *per topic* and stated as a rate
floor plus a timeout — exactly `min_rate_hz` + `lease_duration`. (2) The
reaction is a *path through several nodes*, i.e. a scope path from
`/diagnostics` to `/control/command/control_cmd`, and its latency is
derivable the way every scope path's is. (3) The safe state is a bounded
output envelope (acceleration, jerk, steer-rate limits), which is a fact
about the actuator gate that nothing upstream can change. The diagnostic
graph is also a **mode graph** — `contract-axes.md` §3.3 deferred modes; this
is where they would come back, and the design below does not need them yet.

## 2. The primitives

Three additions. Everything else is derived.

### 2.1 `hazards.<h>.ftti` — the requirement

`criticality-from-hazards.md` already proposes `hazards:` at scope level with
`severity` and `mitigated_by`. FTTI joins it, because that is where ISO 26262
puts it: the number belongs to the hazard, not to any node.

```yaml
hazards:
  drive_blind:
    severity: ASIL_D
    description: vehicle continues with no obstacle data
    guards: [/safety/scan]            # the topics whose loss/lateness IS the fault
    ftti: 500ms                       # physics: how long blind driving stays survivable
    reaction: safety.emergency_stop   # the scope path that reaches the safe state
```

`guards` names what is watched, `reaction` names a scope path (§2.3). Both
are references to things the contract already has; `ftti` is the only new
number.

### 2.2 `on_violation:` — the reaction, on the subscriber that detects

The subscriber is where a fault is *observed*, so it is where the obligation
to react is declared. This is the WdgM "EXPIRED → reaction" edge.

```yaml
nodes:
  brake_controller:
    sub:
      obstacles:
        min_rate_hz: 50
        max_age: 60ms
        qos: { liveliness: manual_by_topic, lease_duration: 100ms, deadline: 30ms }
        on_violation:
          on: [omission, late]        # EMV2 closed set: omission | late | loss
          reaction: emergency_stop    # a path on THIS node
          within: 50ms                # FRTI budget for this hop, requirement
    paths:
      emergency_stop:
        trigger: { input: [obstacles] }   # fires from the same subscription's event callback
        output: [brake_cmd]
        max_latency: 5ms
        safe_state:                   # fact: what this reaction produces
          emits: brake_cmd
          settle: 200ms               # plant time to reach the safe state; measured, not authored
```

- `on` is the **closed set** from §1.3. An unknown value is a parse error.
- `reaction` is a path name; the path's `trigger` must include the
  subscription (`reaction-unreachable` otherwise).
- `within` is a requirement on *this node's* part of FRTI.
- `safe_state.settle` is a fact of the actuator/plant, the one term in FRTI
  the software cannot derive. Expected to be measured (`rt_av_demo` can be an
  oracle for it too: the brake's settle is a parameter).

### 2.3 Reaction as a scope path — nothing new

A reaction that spans nodes (Autoware's diagnostics → MRM → gate) is a scope
path with a budget, which already exists:

```yaml
paths:
  safety.emergency_stop:
    trigger: { input: [/diagnostics] }
    output: [/control/command/control_cmd]
    max_latency: 150ms
```

The FRTI of that reaction is its derived critical-path latency plus the
terminal `safe_state.settle`. No new key.

## 3. What is derived, and the checks that consume it

| derived | from | rule |
|---|---|---|
| **FDTI** of a guarded topic | `max(lease_duration, 1/min_rate_hz + deadline, lifespan)` over the guard's subscribers — the *slowest* detector, because the fault is detected when the last mechanism notices | — |
| **FRTI** of a reaction | derived route latency (`scope-budget`'s machinery) + `safe_state.settle` | — |
| `fault-reaction-budget` | `FDTI + FRTI > ftti` | **error** — the hazard is not covered in time; names both terms |
| `hazard-unguarded` | a hazard whose `guards` topic has no subscriber carrying a detector (`lease_duration`, `deadline`, `max_age` or `min_rate_hz`) | **error** — nothing would ever notice |
| `reaction-unreachable` | `on_violation.reaction` path whose trigger does not include the subscription, or whose output does not reach the hazard's reaction sink | **error** |
| `reaction-unbudgeted` | a reaction path with no `max_latency` | **warning** — FRTI is then a lower bound, and the budget rule says so ("on INCOMPLETE EVIDENCE", phase 60's phrase) |
| `detector-tolerance` | `miss.tolerate` on the guard × period ≥ `ftti` | **error** — the tolerated misses alone exhaust the interval |
| `ftti-derivable`? | none | there is no consequence to write: FTTI is physics |

Every rule reads a field; every field is read by a rule. The census stays at
four.

**Runtime.** `runtime_enforcement` already emits `runtime_violations.jsonl`
for rate and age. It gains the ROS 2 event callbacks (§1.5) and stamps each
violation as an FDTI observation; `measure` gains the reaction: time from a
violation record to the first message on the reaction's output topic, which
is an observed FRTI. `rt_av_demo` becomes the oracle a third time by adding a
`die_after` parameter to the lidar driver: the true FDTI is the lease, the
true FRTI is the brake's path plus its declared settle, and the sum has a
known right answer.

## 4. Worked example — `rt_av_demo`

Today's contract says the chain must finish in 60 ms. It cannot say what
happens if the lidar stops. With the primitives:

```yaml
hazards:
  drive_blind:
    severity: ASIL_D
    guards: [/safety/scan]
    ftti: 500ms
    reaction: safety.stop

nodes:
  obstacle_detector:
    sub:
      scan:
        min_rate_hz: 50
        qos: { liveliness: manual_by_topic, lease_duration: 100ms }
        on_violation:
          on: [omission]
          reaction: declare_lost
          within: 10ms
    paths:
      detect: { trigger: { input: [scan] }, output: [obstacles], max_latency: 12ms }
      declare_lost:
        trigger: { input: [scan] }
        output: [obstacles]          # publishes an "unknown" obstacle message
        max_latency: 2ms

  brake_controller:
    sub:
      obstacles:
        max_age: 60ms
        on_violation:
          on: [late, omission]
          reaction: emergency_stop
          within: 20ms
    paths:
      emergency_stop:
        trigger: { input: [obstacles] }
        output: [brake_cmd]
        max_latency: 5ms
        safe_state: { emits: brake_cmd, settle: 200ms }

paths:
  safety.stop:
    trigger: { input: [/safety/scan] }
    output: [/safety/brake_cmd]
    max_latency: 100ms
```

What `check` derives:

```
FDTI(/safety/scan)         = lease 100ms                       (slowest detector on the guard)
FRTI(safety.stop)          = route 2 + 5 ms + settle 200ms = 207ms
fault-reaction-budget      : 100 + 207 = 307ms ≤ ftti 500ms    OK, 193ms slack
```

Change `lease_duration` to 400ms and it fails: `400 + 207 = 607 > 500`, with
the message naming which term to tighten. Delete `on_violation` from
`obstacle_detector` and `reaction-unreachable` fires: the hazard's reaction
path starts at `/safety/scan`, and no subscriber of it declares a reaction.

## 5. Worked example — an Autoware slice

The numbers are Autoware's own, read from its parameter files (§1.6).

```yaml
hazards:
  lost_localization:
    severity: ASIL_D
    guards: [/localization/pose_twist_fusion_filter/pose]
    ftti: 2s                                   # a HARA number; illustrative
    reaction: system.emergency_stop

nodes:
  topic_state_monitor_pose:                     # autoware_topic_state_monitor
    sub:
      pose:
        min_rate_hz: 1.0                        # error_rate
        qos: { lease_duration: 1s }             # timeout
        on_violation: { on: [omission, late], reaction: report, within: 100ms }
    paths:
      report: { trigger: { input: [pose] }, output: [diagnostics], max_latency: 10ms }

  mrm_handler:
    sub:
      operation_mode_availability:
        qos: { lease_duration: 500ms }          # timeout_operation_mode_availability
    paths:
      call_mrm:
        trigger: { input: [operation_mode_availability] }
        output: [mrm_request]
        max_latency: 110ms                      # 10 Hz update + 0.01 s call timeout

  mrm_emergency_stop_operator:
    paths:
      stop:
        trigger: { input: [mrm_request] }
        output: [control_cmd]
        max_latency: 34ms                       # 30 Hz
        safe_state: { emits: control_cmd, settle: 1.2s }   # −2.5 m/s² from 3 m/s, measured

paths:
  system.emergency_stop:
    trigger: { input: [/diagnostics] }
    output: [/control/command/control_cmd]
    max_latency: 300ms
```

```
FDTI = max(lease 1s, 1/1.0 Hz) = 1.0s
FRTI = 10 + 110 + 34 ms (+ diagnostic_graph_aggregator, unbudgeted → warning) + settle 1.2s ≈ 1.36s
fault-reaction-budget: 2.36s > ftti 2s   → ERROR: the 1 s detection timeout is the term to tighten
```

That is not a contrived failure. It is the arithmetic Autoware's parameter
files imply and nothing today performs.

## 6. Deliberately not folded

- **Value-domain faults** (`ValueError`, plausibility, STPA's "wrong
  duration"). Needs message inspection; out of the contract's reach.
- **E2E CRC / DataID.** Middleware integrity, not a system-level requirement.
- **Diagnostic coverage / SFF / proof-test interval.** IEC 61508 hardware
  metrics; our detectors are omission and lateness, whose coverage is by
  construction.
- **The safe-state envelope** (vehicle_cmd_gate's limits). A real fact and
  a real check (a reaction that commands −2.5 m/s² through a gate limited to
  −2.4 is clipped), but a value-domain one. Noted for after this lands.
- **Modes.** Autoware's diagnostic graph is a mode graph. Still deferred
  (`contract-axes.md` §3.3); §2 works without it because a reaction is a path,
  not a mode transition.

## 7. Kind and consumer, per key

| key | kind | consumer |
|---|---|---|
| `hazards.<h>.ftti` | requirement | `fault-reaction-budget` |
| `hazards.<h>.guards` | meta (reference) | `hazard-unguarded`, FDTI derivation |
| `hazards.<h>.reaction` | meta (reference) | `reaction-unreachable`, FRTI derivation |
| `sub.<e>.on_violation.on` | fact (closed set) | detector selection in FDTI; runtime event wiring |
| `sub.<e>.on_violation.reaction` | meta (reference) | `reaction-unreachable` |
| `sub.<e>.on_violation.within` | requirement | per-hop FRTI check |
| `paths.<p>.safe_state.emits` | fact | `reaction-unreachable` (the sink) |
| `paths.<p>.safe_state.settle` | fact (measured) | FRTI derivation |

Eight keys, eight consumers, one closed set. `scripts/field_census.py --check`
holds it.

## 8. Phase plan (71, proposed)

- **W1 — vocabulary, additive.** Field table rows with `kind`, parse with
  closed `on`, model lowering. Acceptance: every existing contract resolves
  byte-identically (phase 67's criterion).
- **W2 — derivation and rules.** FDTI/FRTI derivation in the resolver beside
  `scope-budget`; the five rules of §3. Acceptance: the two worked examples
  produce the verdicts written above, and `rt_av_demo`'s contract gains the
  hazard.
- **W3 — verify on a running system.** `lidar_driver` gains `die_after`;
  `runtime_enforcement` wires the QoS event callbacks; `measure` reports
  observed FDTI/FRTI; `just fault` compares them to the derived numbers. The
  oracle is exact: lease and settle are parameters.
- **W4 — the Autoware slice**, as a contract in `tests/fixtures/autoware`,
  with the parameter-file numbers. The first `fault-reaction-budget` error on
  a real system is the deliverable.
- **Retirement:** none. Nothing here replaces an existing key.

## Sources

- ISO 26262:2018 Parts 1 (3.61 FTTI), 4 (Annex B timing), 5 (Annex D), 6 (Annex D timing monitoring)
- AUTOSAR CP R22-11, *Specification of Watchdog Manager* (alive/deadline/logical supervision, `WdgMFailedSupervisionRefCycleTol`)
- AUTOSAR CP R22-11, *E2E Protocol Specification*
- SAE AS5506/3, *AADL Error Model Annex v2* (error type library, propagation, recovery)
- Leveson, *Engineering a Safer World* (STPA unsafe control actions)
- ROS 2 design: *QoS deadline, liveliness, lifespan*; `rclcpp` `SubscriptionEventCallbacks`
- Autoware 1.5.0: `autoware_topic_state_monitor`, `autoware_component_state_monitor/config/topics.yaml`, `autoware_diagnostic_graph_aggregator`, `autoware_mrm_handler`, `autoware_mrm_emergency_stop_operator`, `autoware_vehicle_cmd_gate` parameter files
- `docs/design/contract-axes.md` §3.1, §4, open question 6; `docs/design/criticality-from-hazards.md`
