# Phase 72 — criticality is a consequence of the hazards

Status: **complete** (2026-09-06; manifest crate `v0.1.31`).
Design of record: `docs/design/criticality-from-hazards.md` (2026-08-07),
unblocked by phase 71's `hazards:`.

`nodes.<n>.criticality: high` was a bare ordering with no meaning — two
engineers would not agree on what makes a node `high`, and neither could be
shown wrong. Every safety standard answers the same way: criticality is never
a property of a component; it is allocated inward from an outcome. Phase 71
put the outcome in the contract (`hazards.<h>.severity`), so the label
becomes derivable, and `contract-primitives.md`'s rule applies: derived,
never written.

## What derives it

A hazard with a `severity` reaches a node three ways, and the node takes the
**max** over every hazard that reaches it (R1: max, never sum):

| role | which nodes |
|---|---|
| **feeds** | the guard topic's publishers and their upstream causal closure, state edges included — a stale map produces a hazardous plan as surely as a stale scan |
| **detects** | a subscriber of a guard that declares an `on_violation` |
| **reacts** | every node on the reaction walk to the safe state |

A node no hazard reaches derives nothing, and its label — if any — stands.
That is the underivable case, the same absence of information the rate
derivation reports as `Unknown`, and it is why `criticality` stays a live
key of `Kind::Consequence` rather than being deleted.

## The scale

`severity_levels:` is declared per manifest, ascending, defaulting to ISO
26262's `[QM, ASIL_A, ASIL_B, ASIL_C, ASIL_D]`; a team on IEC 61508 or
DO-178C names its own. The first entry derives no criticality. A severity
outside the scale is `severity-unknown` (error), not a silent `None` — the
failure `parse_criticality` used to have. The mapper's three buckets are a
fold of the scale: rank 0 → none, the rest split evenly with the top third
`High` (on the default scale A → Low, B → Medium, C and D → High).

## Rules

- `derivable-criticality` (info) — the label equals what the hazards derive:
  a second copy, deletable. Says which hazard and which role.
- `criticality-mismatch` (warning) — the label disagrees. The derivation
  wins for scheduling; the message names the hazard, the role and the
  severity.
- `severity-unknown` (error).

`sched_derive` reads `index.derived_criticality` before any label, in both
places a label used to be read.

## Measured

`rt_av_demo`: the three `criticality: high` labels are all reported
redundant — `brake_controller` reacts, `obstacle_detector` detects,
`lidar_driver` feeds `drive_blind` (ASIL_D). The derived schedule is
byte-identical. `contract_fault_late` carries a `low` label on the brake
and gets `criticality-mismatch`.

## A census blind spot this exposed

`hazards.<h>.severity` was write-only after phase 71 W1 and the census said
consumed: every `Diagnostic.severity` read in the tree matches the field
name. The grep errs toward reporting more reads by design, and this is the
shape of its false positive — a common name. Deriving criticality from it
makes it genuinely read. Worth remembering when the census says "consumed"
about a field named `name`, `value` or `severity`.

## Not done

- Retiring the labels from the corpus (the `derivable-criticality` infos
  name each one) — the same "take the tool's advice" step phase 70 did for
  rates.
- R6 (a mitigation barrier attenuates severity upstream) from the design:
  today the full severity propagates through every guard's ancestors.
