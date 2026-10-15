# Phase 93 - supervised budgets: the keys, the overrun case, a period's range

Status: **in progress** (2026-10-15): S1 (rlm) and S2 (play_launch) in
progress; S5 in progress (period-range agent). Nothing released.

The grammar belongs to `ros-launch-manifest` (rlm,
`~/repos/ros-launch-manifest`); it is tracked here because the checker
that reads it is play_launch's, as phase 90 did. The runtime half (a
deadline-armed route monitor and its reaction hook) is nano-ros's; this
phase only defines what the model hands it (the lowering shape below).

## Why

A budget states where its number comes from (`source:`, rlm v0.1.58).
Most budgets of a deployed contract are `measured`: the largest value some
runs observed plus a margin. That is not an upper bound, and `check
--explain` says so on every total that contains one ("measured, not a
bound"). Two ways to make such a total a safety argument:

- analyse every term (a WCET tool, a model of the link): knowledge a launch
  contract should not have to carry;
- supervise it: the runtime watches the budget while the system runs and
  starts a fallback when it is broken, and the checker shows that "budget +
  detection + fallback" still fits the hazard's FTTI. This is deadline
  supervision as in a watchdog manager, and needs no analysis tool.

This phase takes the second. `source: analysed` stays as an upgrade for a
user who has an analysis; it is never required.

Each budget gets a **level**, derived, never written:

| source | `supervise:` | level |
|---|---|---|
| `analysed` | any | bound |
| any other, or unstated | stated | supervised |
| `measured` | none | unsupervised |
| `requirement`, `derived`, `allocated`, unstated | none | (none: not evidence about the implementation) |

## Units

The island's plan (simple-autoware-safety-island
`docs/roadmap/phase-11-supervised-budgets.md`, its units S1-S7) splits by
repository. This phase holds the toolchain units; the island's S3
(nano-ros monitor), S4 (board rung), S6 (island contract) and S7 (board
check) stay there.

| this phase | island phase 11 | repo | owner |
|---|---|---|---|
| S1 | S1 | rlm | this agent |
| S2 | S2 | play_launch | this agent |
| S5 | S5 | play_launch (+ rlm `allocated`, from S1) | period-range agent |

### S1. The supervision keys (rlm)

What exists (rlm main `ec5e4d8`, v0.1.60):

- one `Bound` per bound key, `{ kind, limit, value, source, param }`
  (`types/src/bound.rs:183`), its map form `{ duration, source, param? }`
  read by `yaml_bound` (`types/src/parse.rs:196`), serialized by
  `impl Serialize for Bound` (`bound.rs:250`);
- `SourceKind` requirement | measured | derived | analysed
  (`types/src/types.rs:105`), read by `parse_source` (`parse.rs:256`);
- `FaultKind` omission | late | loss | reported (`types.rs:552`, model
  `model/src/lib.rs:1075`), read by `parse_fault_kind` (`parse.rs:844`);
- a timer's `rate_hz` is a bare number (`parse_trigger`, `parse.rs:1877`),
  stored on `Trigger::Timer` (`types.rs:1293`); its parameter binding is
  kept on `PathDecl.timer_param` so the variant every consumer matches stays
  unchanged (`types.rs:1234`);
- the model's `PathContract` (`model/src/lib.rs:1462`) carries a path's
  `max_latency_ms`, which nano-ros turns into route monitor rows.

What changes:

- `supervise: { reaction | enters, mechanism }` beside `source:` in the map
  form of every budget key (every `BOUND_KEYS` row with a map form):

  ```yaml
  max_latency:
    duration: 149ms
    source: { measured: { margin: 0.2, terms: { tick: { period: 100ms, max: 14.84ms }, work: { max: 25.04ms } } } }
    supervise: { reaction: brake_now, mechanism: timer }
  ```

  `reaction:` names a path: a path of the budget's own node (as
  `on_violation.reaction` does), else a scope path of the manifest.
  `enters: <mode>` names a mode instead (its `reaction:` scope path is what
  runs). Exactly one of the two. `mechanism`: `executor` (checked in the
  spin loop: catches slowness, not a stuck callback) or `timer` (an
  independent timer and thread: catches both). `Bound.supervise:
  Option<Supervision>`.
- `FaultKind::Overrun` (`overrun`), types and model: the fault a
  supervision detects, for traces and verdicts. Not written in a `on:`
  list (a subscriber cannot observe it); the parser says to use
  `supervise:`.
- top-level `policy: { unsupervised_measured: warn | error }`, default
  warn: the severity of play_launch's `measured-unsupervised`.
- top-level `supervision: { tick, switch }`: the platform terms of the
  detection delay, stated once per contract (S2): `tick` the period the
  monitor's clock is read at (the executor's tick, the timer's resolution),
  `switch` the interrupt plus thread switch a `timer` mechanism adds.
  Durations. Until play_launch phase 91's profiles carry platform facts,
  this is where they are written.
- `SourceKind::Allocated` (`allocated`): a value chosen inside a range the
  checker derives (S5).
- `rate_hz` of a timer takes a map form, `rate_hz: { value: 10, source:
  allocated }`, so it can carry a source; the bare number stays. The source
  is kept on `PathDecl.timer_rate_source` (the same reason as
  `timer_param`), and `Manifest::budget_sources()` lists it as
  `...trigger.timer.rate_hz`.
- `Bound::level()` / `BudgetLevel` { bound, supervised, unsupervised }:
  the table above.
- the model: `PathContract.supervise: Option<SuperviseContract>` (see
  "The lowering" below), `FaultKind::Overrun`.
- field-table rows with SINCE 0.1.61; format reference regenerated;
  `scripts/key_inventory.py` rerun. All additive: no migrate rewrite.
  GRAMMAR_VERSION becomes 0.1.61 with an `## Unreleased` changelog section
  on top (the rlm convention: the version a release of these keys needs);
  no tag.

Gate: rlm round-trips the keys; field rows with SINCE; format reference
regenerated; the existing tests pass.

### S2. The overrun case in the checker (play_launch)

What exists (play_launch main `bdbfffa8`, 0.17.0 + rlm v0.1.60):

- `check_fault_reaction` (`src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs:2608`)
  pushes one `RungBudget` (`:325`) per ladder rung, `RungRole` (`:309`)
  rung | window | skipped | floor; the rung rows at `:3182`, the floor at
  `:3402`;
- `walk_terms` (`:382`) collects the stated keys a reaction walk charges,
  without their order or values; `walk_reaction` (`:2003`) walks from the
  guard topics only;
- `--explain`'s CHECKED BY (`ros/budget_book.rs:198` `checks_of`) says
  "runtime monitor (...)" from the key's shape (`:220`, `:269`, `:312`);
- `render_budgets` (`verbs/check.rs:394`) prints "TOTAL ... is measured,
  not a bound" (`:483`, `:544`);
- `path_contract` (`ros/model_builder.rs:358`) lowers a path to the model.

What changes:

- `walk_terms` returns `Vec<(TermKey, f64)>`: each stated key in the order
  charged, with the value charged.
- `walk_reaction` can start at a node path or at topics past the guard, so
  a supervision's reaction is walked by the same rules.
- New module `ros/supervision.rs`: the lowering, the detection delay, the
  overrun rows, the level.
- For each supervised budget charged on a rung or floor row, a
  `RungRole::Overrun` row: the time charged before the budget starts + the
  budget + the detection delay + the reaction's route + its settle, against
  the hazard's FTTI; rule `supervised-fit` (error when it does not fit, info
  with the arithmetic when it does). Overrun rows are built for a node
  path's `max_latency` on the walked route and for the row's settle
  `response`; other supervised keys get no overrun row (and no lowering,
  below).
- Detection delay, by mechanism:
  - `executor`: `supervision.tick` + the longest callback on that executor
    + the check (not stated, taken as 0). The executor is the budget's
    node; the longest callback is the largest measured `work` term over the
    node's paths (`max x (1 + margin)`, rounded up, as `budget-rule`
    sizes it). A node path with no `work` term makes it unknown, and the
    row is refused (`supervised-fit`, error) naming the path.
  - `timer`: `supervision.tick` + `supervision.switch`; either missing is
    refused.
  - `supervision.tick` missing is refused for both.
- Under `mechanism: executor`, a reaction whose first hop runs on the
  supervised budget's node is noted "covers slowness, not a hang": a stuck
  callback holds the check and the reaction both.
- CHECKED BY of a supervised budget is read from the lowering (`runtime
  monitor (route-deadline-runtime: <reaction>, <mechanism>)` only when the
  budget was lowered to a monitor row with a reaction), not from the key.
  A supervised budget that is not lowered is refused
  (`supervised-unlowered`, error).
- A LEVEL column in both `--explain` tables replaces "measured, not a
  bound": per stated budget its level; per row the weakest of its terms
  (unsupervised < supervised < bound).
- Rule `measured-unsupervised` per unsupervised budget, at the severity of
  `policy.unsupervised_measured` (warning by default).
- Fixtures: an overrun case over the FTTI is refused; a supervised budget
  with no lowered monitor is refused.

Gate: the island contract's `check --explain` does not regress; the
fixtures above; `cargo test` for the touched crates.

#### The lowering (the shape nano-ros reads)

A supervised budget is lowered when it is a node path's `max_latency`, the
path is input-triggered (the route monitor stamps the take of its first
input), and its reaction resolves. The model's node path gets:

```yaml
contracts:
  node_paths:
    /island/mrm_handler/call_mrm:
      max_latency_ms: 149.0
      supervise:
        route_deadline_ms: 149.0          # the budget: due = take + this
        reaction: /island/mrm_handler/brake_now   # a path id (below)
        mechanism: timer                  # executor | timer
```

- `route_deadline_ms`: the supervised budget, ms (today equal to
  `max_latency_ms`; kept separate so a deadline can differ from the
  completion budget later).
- `reaction`: a path id the reaction hook receives. A node path: its key in
  `contracts.node_paths` (`<node key>/<path>`). A scope path, or the
  `reaction:` scope path of an `enters:` mode: its key in
  `contracts.scope_paths`.
- `mechanism`: `executor` or `timer`, lowercase.

nano-ros (island S3) copies the three onto the route monitor row it already
builds from `max_latency_ms` (`contract_monitors.rs`, `MonitorRow`), as
`route_deadline_ms`, `reaction`, `mechanism`.

### S5. A feasible range for each timer's period (play_launch)

Owner: period-range agent. Uses S1's `SourceKind::Allocated` and the
`rate_hz` map form.

- What: every rung row records how many times each timer's period enters
  it (sampling hops, reported detection, a measured `tick: { period }`
  term, the window owner's notice): coefficients on `RungBudget`
  (`manifest_loader.rs:325`). Upper bound: the current period plus the
  smallest slack / count over the rows, cross-checked by re-running the walk
  with that period. Lower bound: sum of work / period per executor <= 1,
  from measured `work` terms; "unknown" when a timer on the executor has no
  work stated. A table in `--explain`; with `rate_hz: { value, source:
  allocated }` the checker refuses a period outside the range.
- Gate: the island's 10 Hz handler gets a printed range with its limits
  named.

## Status

- S1: in progress, branch `phase93-s1`.
- S2: in progress, branch `phase93-s2`.
- S5: in progress (period-range agent).

## Not in this phase

- The runtime: nano-ros's deadline-armed route monitor and reaction hook
  (island S3), the board's timer rung and watchdog (island S4).
- Lowering a supervised budget that is not a node path's `max_latency`
  (a settle's response needs a check on the plant's feedback; a transport
  needs a shared clock). They are refused, not dropped.
- Platform facts by name (play_launch phase 91's profiles):
  `supervision: { tick, switch }` is written per contract until then.
- A release: S1 needs rlm v0.1.61, S2 and S5 a play_launch release pinning
  it; coordinated outside this phase.
