# Phase 75 — operational modes

Status: **complete** (W1–W5, 2026-09-07; manifest crate `v0.1.33`).
Design of record: `docs/design/operational-modes.md`.

The axis deferred in phases 67, 68 and 71. Phase 71 §9.4 ruled "one reaction
per hazard, the most conservative" precisely because a mode was not
expressible; this makes it expressible and lifts the ruling.

## W1 — vocabulary (`v0.1.33`)

Three additions, all scope level; nothing on a node changes.

- `functions.<f>` — a named guard group in phase 71's two shapes (a bare
  list is any-of, `{ all_of: [...] }` a redundant set). A hazard's `guards:`
  may name one; the **loader** resolves the name, because the parser sees
  one manifest and the loader sees the scope.
- `modes.<m>` — `requires` (fact), `fallback` (requirement: the ordered
  ladder), `reaction` (the scope path reaching its safe state),
  `overrides`.
- `modes.<m>.overrides` — the `contract-axes.md` §3.3 mitigation cashed in.
  A requirement keeps exactly one value where it is declared; a mode pins
  another **for that mode** by naming the requirement's contract path. No
  scalar becomes a map, so no reader changes. The nested mapping an author
  writes is flattened to `(dotted target, value)` at parse time.

## W2 — the ladder, and three rules

A reaction naming a scope path is a **one-rung ladder** — phase 71's form,
byte-identical. A reaction naming a mode walks its `fallback` in order.

- `ladder-rung-budget` (error, per rung) — each rung's own route + settle
  against the hazard's `ftti`. A graded reaction is a promise in its own
  right, not only a step toward the floor.
- `ladder-unterminated` (error) — no ladder, a rung with no `reaction:`, or
  a last rung requiring something this hazard's own guards remove. That
  last case is the subtle one: a floor that the fault takes away is not a
  floor.
- `fault-reaction-budget` — unchanged, and now measured against the **last**
  rung. Phase 71's conservative verdict stays exactly where it was.

## W3 — per-mode overrides

`mode-requires-unguarded` (error): a required function no subscriber of
which declares an `on_violation`. Nothing would notice it lost, so the mode
can never be declared unavailable and the ladder below it can never be
taken — a mode that cannot fall is not a mode.

`override-target-missing` (error): an override naming no declared
requirement. Checked against the **declaration**, not the merged index: an
override pins a value over what an author wrote.

**A defect the good fixture found immediately.** The first
`override-target-missing` fired on a *correct* contract. Targets are dotted,
and scope-path names in this corpus contain dots (`safety.stop`,
`system.emergency_stop`), so positional splitting read
`paths.safety.stop.max_latency` as a four-part path and matched nothing.
The target is now read section-from-the-front, field-from-the-back, with
whatever lies between as the name — dots and all. A rule that fires on a
correct contract is worse than one that does not fire at all.

## W4 — runtime

The observer already tracked which guard is silent per hazard. Modes add
one derivation on top: a function is lost when its group is, a mode is lost
when a required function is, and the ladder is walked for the first rung
still available. One event, `mode-availability`, reported **once per
transition** (a 50 Hz guard would otherwise emit it fifty times a second)
and cleared on recovery. A mode whose ladder has no available rung left is
an error rather than a warning.

`terminal_reaction_path` is shared by the observer and `measure`: a
mode-shaped reaction otherwise reads as *no* reaction, which is what
`measure` reported the first time the demo's hazard named a mode.

## W5 — Autoware

The overlay contract gained `operation_mode_availability` as a function and
the real four-mode ladder. `mrm_handler` walks that ladder at runtime; now
it is declared, and every rung is checked:

```
error[ladder-rung-budget]: hazard 'mode_unavailable': fallback rung
'comfortable_stop' cannot make the fault-tolerant time interval —
detection 500.00ms + reaction 4000.00ms + settle 0.00ms = 4500.00ms
against 2000.00ms.
```

That is the arithmetic a graded reaction implies and nothing performed:
comfortable stop decelerates over about four seconds, so it cannot cover a
two-second interval even though the emergency floor beneath it can.

## Measured

`just fault` in `rt_av_demo`, one run:

```
DDS reported the lapse           99.34ms after the last publish (lease 100ms)
live observer   reaction        100.67ms after the fault
measure (post hoc)              100.67ms → + settle 200ms = 300.67ms, fits ftti 500ms
mode transition   mode 'driving': LOST — falling to 'stopped'
```

## Not done

- **Forward transitions** (engage). The transition manager's conditions are
  vehicle physics — speed, yaw, distance thresholds — which the contract
  cannot express. Losing a function is a fact; regaining permission is not.
- **Mode-dependent scheduling** (a tier per mode). Belongs to the platform
  file, whose `overrides:` could take the same mode-keyed shape.
- **Commanded vs inferred mode.** `system_modes`'s central check needs the
  commanded mode from a stack-specific topic. The observer infers; nothing
  compares that to a command yet.
- **The checker does not yet RUN per mode.** `overrides:` is parsed,
  lowered, and its target validated; running the full rule set once per
  mode with the override applied is the remaining half of W3.
