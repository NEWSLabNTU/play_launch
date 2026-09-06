# Operational modes — the axis deferred three times

Status: design, **implemented** as phase 75 (2026-09-07; `docs/roadmap/phase-75-operational-modes.md`). One correction made during implementation is recorded there: a dotted override target cannot be split positionally, because names contain dots. Originally proposed 2026-09-06. Answers `contract-axes.md` §3.3
(deferred with a mitigation) and open question 7 ("do modes belong in the
contract at all?"). Supersedes phase 71 §9.4's ruling that a hazard has one
reaction.

## Why now

Three phases deferred it, and each one bumped into it:

- Phase 67 kept every requirement a scalar so a mode-keyed map would be an
  addition, not a rewrite (the §3.3 mitigation). That debt is unpaid.
- Phase 71 §9.4 ruled "one reaction per hazard, the most conservative",
  because Autoware's `mrm_handler` picks between `pull_over`,
  `comfortable_stop` and `emergency_stop` by what is still *available*, and
  the contract could not say what availability is.
- Phase 73's observer watches guards go silent and knows nothing about what
  the loss makes unavailable; it reports a fault, not a degraded system.

Every remaining item on the list is smaller than this one, and this one
keeps coming back.

## What a mode is, measured on the corpus

Autoware's diagnostic graph is the only shipped example in reach, and it is
three layers:

```
diag units      /sensing/lidars/front            type: diag  (a node's status)
function units  /functions/obstacle_detection    type: or    [lidar front, radar front]
                /functions/pose_estimation       type: and   [lidar top]
mode units      /autoware/modes/autonomous       type: and   [pose_estimation, obstacle_detection]
                /autoware/modes/comfortable_stop type: and   [obstacle_detection]
                /autoware/modes/emergency_stop   type: ok    (needs nothing)
```

The output is `OperationModeAvailability` — one boolean per mode — and
`mrm_handler` walks a fixed ladder for the best available reaction. Two
observations decide the design:

1. **A function is a guard group.** `or [lidar_front, radar_front]` is
   exactly phase 71's `{ all_of: [...] }` — the function is lost when every
   member is. `and [lidar_top]` is a bare guard. The contract already has
   this level; it only lacks a *name* for a group so several modes can
   share one.
2. **A mode is a set of functions it requires, plus the reaction ladder
   when it is lost.** Nothing about a node changes between modes in this
   graph — what changes is which *requirements apply*. That is the §3.3
   observation ("timing requirements differ per mode") from the other side.

`system_modes` (Bosch, ROS 2) agrees on the second point from a different
direction: a mode there is a parameter set plus lifecycle states, and its
distinctive contribution is **mode inference** — deciding the actual mode
from observed state rather than from what was commanded. Phase 73's
observer is already an inference engine over guards; a mode is what it
should infer.

AADL modes carry mode-specific connections and property values with
transitions on events; AUTOSAR's `ModeDeclarationGroup` and BswM carry
mode-dependent scheduling. Both agree modes are a *system* property with
*per-mode values of requirements* hung off them — which answers question 7:
modes live in the contract at **scope level**, beside `hazards:` and
`paths:`, and never on a node.

## The primitives

Three additions, all at scope level. Nothing on a node changes.

### `functions:` — named guard groups

```yaml
functions:
  pose_estimation:   { all_of: [/localization/ndt/pose, /localization/gnss/pose] }
  obstacle_detection: { all_of: [/sensing/lidar/front, /sensing/radar/front] }
  trajectory:        [/planning/trajectory]        # a bare list: any-of
```

A function is a guard group with a name. `hazards.<h>.guards` may name one
(`guards: [obstacle_detection]`) instead of repeating its members; the
resolution is the same `all_of` arithmetic phase 71 already performs.
**Kind: meta** — a name for something already expressible.

### `modes:` — what each mode requires, and the ladder

```yaml
modes:
  autonomous:
    requires: [pose_estimation, obstacle_detection, trajectory]
    fallback: [pull_over, comfortable_stop, emergency_stop]   # in order
  pull_over:
    requires: [pose_estimation, obstacle_detection]
    reaction: system.pull_over          # a scope path, as hazards.<h>.reaction is
  comfortable_stop:
    requires: [obstacle_detection]
    reaction: system.comfortable_stop
  emergency_stop:
    requires: []                        # needs nothing — the floor of every ladder
    reaction: system.emergency_stop
```

- `requires` is a list of functions (or bare guards). A mode is
  **available** while every one holds. **Kind: fact** — this is what the
  diagnostic graph computes.
- `fallback` is the ordered ladder to try when the mode is lost. Each rung
  is a mode; the runtime takes the first *available* one. **Kind:
  requirement.**
- `reaction` on a rung is the scope path that reaches its safe state — the
  same field a hazard has. A mode with `requires: []` and a reaction is a
  terminal safe state.

### Per-mode requirement values — `overrides:` keyed by mode

The §3.3 debt, paid the way the platform file already pays it: not by
turning every scalar into a map, but by a block that pins values per mode,
over the scalar that stays the default.

```yaml
modes:
  degraded:
    requires: [obstacle_detection]
    overrides:
      paths:
        lidar_to_brake: { max_latency: 100ms }     # 60ms in the default mode
      nodes:
        obstacle_detector:
          sub: { scan: { min_rate_hz: 10 } }       # 50 Hz in the default mode
```

Every requirement keeps exactly one value where it is declared. A mode's
`overrides:` names a requirement by path and pins another value *in that
mode*. The checker runs the mode's requirements with the override applied;
the default mode is the contract as written. This keeps the phase 67
promise — no reader of a scalar changes — and makes a mode-specific value a
diff against the default, which is how a safety reviewer reads it anyway
("in degraded mode we accept 100 ms instead of 60").

## What is derived, and the checks

| derived | from | rule |
|---|---|---|
| availability of a mode | its `requires` over the guards' health | runtime, `mode-availability` |
| the reaction a hazard takes | walk the `fallback` ladder, take the first mode whose `requires` does not include the faulted function | replaces phase 71's single `reaction` when the hazard names a mode instead of a path |
| `ladder-unterminated` | a ladder whose last rung requires something the ladder's own hazard can lose | **error** — there is no floor |
| `ladder-rung-budget` | each rung's `reaction` route + settle against the hazard's `ftti` **from that rung's detection** | **error** per rung that cannot make it; the graded reaction is checked in its own right, not only the worst |
| `fault-reaction-budget` | unchanged, computed against the **last** rung | phase 71's conservative verdict stays the floor |
| `mode-requires-unguarded` | a function in `requires` none of whose members has a reacting subscriber | **error** — the mode can never be declared lost |
| `override-target-missing` | an `overrides:` path naming no requirement | **error** |
| `mode-override-relaxes-only`? | — | **not a rule**: a degraded mode may tighten as well as relax (a stop mode wants a *shorter* reaction), so no direction is enforced |

Runtime: the phase 73 observer already tracks guard health per hazard.
Modes add one derivation on top — a function is lost when its group is,
a mode is lost when a required function is — and one event,
`mode-availability` (`autonomous: lost (obstacle_detection: /sensing/lidar/front
silent 103ms) → fallback comfortable_stop available`), and then the existing
`hazard-reaction` against the rung the ladder selected. `measure` reports
observed time-in-mode per mode.

## Worked example — Autoware, from its own graph

```yaml
functions:
  pose_estimation:    [/sensing/lidars/top]
  obstacle_detection: { all_of: [/sensing/lidars/front, /sensing/radars/front] }

modes:
  autonomous:
    requires: [pose_estimation, obstacle_detection]
    fallback: [pull_over, comfortable_stop, emergency_stop]
  pull_over:         { requires: [pose_estimation, obstacle_detection], reaction: system.pull_over }
  comfortable_stop:  { requires: [obstacle_detection], reaction: system.comfortable_stop }
  emergency_stop:    { requires: [], reaction: system.emergency_stop }

hazards:
  lost_localization:
    severity: ASIL_D
    guards: [pose_estimation]
    ftti: 2s
    reaction: autonomous          # a MODE: take its ladder
```

Derived: losing `pose_estimation` makes `autonomous` and `pull_over`
unavailable; the ladder selects `comfortable_stop` (its requirement still
holds) — which is what `mrm_handler` does at runtime, now stated where a
reviewer can see it. `ladder-rung-budget` checks `comfortable_stop`'s route
against 2 s from the lidar's detection, and `fault-reaction-budget` checks
`emergency_stop`'s as the floor. Lose `obstacle_detection` too and the
ladder falls to `emergency_stop`; a ladder without it would be
`ladder-unterminated`.

## Worked example — `rt_av_demo`

```yaml
modes:
  driving:   { requires: [/safety/scan], fallback: [stopped] }
  stopped:   { requires: [], reaction: safety.stop }
hazards:
  drive_blind: { guards: [/safety/scan], ftti: 500ms, reaction: driving }
```

Same arithmetic as today (the ladder has one rung, `safety.stop`), plus a
`mode-availability` event at the lease: `driving: lost → stopped`. `just
fault` gains one line to assert.

## Rulings

- **Modes live in the contract, at scope level** (question 7). A mode set is
  a system property; so are hazards and scope paths, and they live there
  too. A separate artifact would need its own discovery, overlay and
  parity story for no gain.
- **Per-mode values are overrides, not maps.** Every requirement stays a
  scalar where declared. This is the §3.3 mitigation cashed in.
- **A hazard's `reaction` may name a mode.** The ladder is then the
  reaction, and phase 71's single-path form is the one-rung case. Nothing
  written today changes.
- **`if`/`unless` stay launch-time.** A mode never conditions a node's
  existence; it conditions which requirements apply. A node that exists
  only in one mode is a launch-time question and already has an answer.

## Deliberately not folded

- **Mode transitions as first-class edges** (AADL's `in modes`,
  transitions on events, SysML state machines). A transition here is
  "requirement lost → ladder"; forward transitions (engage) are the
  transition manager's business and carry vehicle-physics conditions
  (speed, yaw, distance thresholds) the contract cannot express. Later, if
  ever.
- **Mode-dependent scheduling** (AUTOSAR BswM, a tier per mode). Belongs to
  the platform file, whose `overrides:` block could take the same
  mode-keyed shape. Out of this design's scope.
- **Commanded vs inferred mode disagreement** (`system_modes`'s core
  check). Needs the commanded mode from a topic, which is stack-specific.
  The observer infers; comparing to a command is a follow-up once one
  stack's topic is declared.

## Kind and consumer

| key | kind | consumer |
|---|---|---|
| `functions.<f>` | meta | guard resolution (phase 71), `mode-requires-unguarded` |
| `modes.<m>.requires` | fact | availability derivation, ladder selection |
| `modes.<m>.fallback` | requirement | ladder selection, `ladder-unterminated`, `ladder-rung-budget` |
| `modes.<m>.reaction` | meta | FRTI derivation (phase 71's walk) |
| `modes.<m>.overrides` | requirement | the checker, run per mode; `override-target-missing` |

Five keys, five consumers.

## Phase plan (75, proposed)

- **W1 — vocabulary, additive.** `functions:`, `modes:` with `requires`/
  `fallback`/`reaction`; `hazards.<h>.guards` and `.reaction` accept a name.
  Byte-identical resolution for every existing contract.
- **W2 — ladder selection and the three rules.** Ladder walk in the
  resolver beside phase 71's; `fault-reaction-budget` moves to the last
  rung; `ladder-rung-budget` per rung.
- **W3 — per-mode overrides.** `modes.<m>.overrides`, the checker run per
  mode, `override-target-missing`.
- **W4 — runtime.** The observer derives function and mode availability
  from guard health; `mode-availability` events; `measure` reports
  time-in-mode. `rt_av_demo` asserts `driving: lost → stopped`.
- **W5 — Autoware.** The graph above as an overlay contract; the first
  `ladder-rung-budget` verdict on a real stack.
