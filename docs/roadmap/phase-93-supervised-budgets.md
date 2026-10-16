# Phase 93 - supervised budgets: the keys, the overrun case, a period's range

Status: **in progress** (2026-10-15; a supervised settle added
2026-10-16 in 0.18.1, see Status): S1 (rlm) done on rlm main
`33577ff`; S2 (play_launch) done on main `bfede19e`; S5 in progress
(period-range agent). Nothing released: S1 needs rlm v0.1.61, S2 a
play_launch release pinning it.

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
  path's `max_latency` on the walked route and (since 0.18.1: "A
  supervised settle" under Status) for the row's supervised settle, the
  whole of it; other supervised keys get no overrun row (and no lowering,
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
- Rule `supervised-executor-long-callback` (added 2026-10-16, after S2):
  a `mechanism: executor` supervision is flagged when a callback on the
  node's executor has no analysed bound (`max_latency` measured,
  supervised, or unstated). Its detection (tick + longest callback) then
  rests on an observed number, and when the supervised path's own callback
  is the one that overruns, the late output is published before the
  overdue check runs, so the miss is seen only at completion, late by the
  overrun. The finding recommends `mechanism: timer`. Severity is the
  policy's, as `measured-unsupervised`: warning by default, error under
  `policy: { unsupervised_measured: error }`. An analysed `max_latency`
  now also counts as a callback's length in the executor detection delay
  (before, only a measured `work` term did). Background: on the island
  board an executor-mechanism deadline never fired for a single 327 ms
  callback against a 149 ms budget; the nano-ros deadline monitor (S3)
  records a late completion, which is detection at completion, late by the
  overrun.
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

- S1: done (2026-10-15), rlm main `33577ff` (ff-pushed; unreleased,
  GRAMMAR_VERSION 0.1.61). `SourceKind::Allocated` and the `rate_hz` map
  form are there for S5. play_launch pins it by `rev` until v0.1.61.
- S2: done (2026-10-15), play_launch main `b0072401` (the rlm pin) and
  `bfede19e`. New module `src/ros-launch-resolve/resolve/src/ros/supervision.rs`;
  `manifest_loader.rs` gets `RungRole::Overrun`, the valued `walk_terms`,
  `walk_reaction_at` and two `overrun_rows` calls; `budget_book.rs` the
  LEVEL and the lowering-based CHECKED BY; `check.rs` the LEVEL columns;
  `model_builder.rs` the `supervise` lowering. Fixtures
  `contract_supervised` (both mechanisms fit; the executor one is noted
  "covers slowness, not a hang"), `contract_supervised_overrun`
  (`supervised-fit` error: 5687.72 ms against 3 s),
  `contract_supervised_unlowered` (`supervised-unlowered` on a timer path).
  Tests: `cargo test` in `src/ros-launch-resolve` passes except
  `producer::tests::the_default_checker_names_the_rlm_tag`, which needs a
  tag pin (it fails on every `rev` pin, and passes again at v0.1.61);
  `tests/manifest_check.rs` 66 of 66, run with the `ros-launch-resolve`
  CLI of this tree standing in for `play_launch` (the installed one is
  0.17.0).
  Island (`safety_island.launch.xml`, `check --explain`, against the same
  CLI built at main before S2): every verdict and every number unchanged;
  new are the LEVEL columns (all four rows `unsupervised`), the notes
  "TOTAL ... is unsupervised" in place of "measured, not a bound" (terms now
  listed in the order charged, the link first), and 8
  `measured-unsupervised` warnings (5 -> 13 warnings, 0 errors).
  `.github/check-contracts.sh` gives the same 16 verdicts with both.
  Not done here: an overrun row for a supervised budget charged inside a
  window's route (the rung below charges the same path later, which is the
  larger case); a supervised `max_transport` or `max_age` is refused as
  unlowered until a runtime check exists for it (a settle: since 0.18.1,
  below).
- `supervised-executor-long-callback`: done (2026-10-16), in
  `supervision.rs` (`executor_long_callback`, `policy_severity`); tests
  `an_executor_supervision_over_a_long_callback_follows_the_policy`,
  `an_executor_supervision_over_bounded_callbacks_is_not_flagged`, and a
  needle in `supervised_budgets_print_their_overrun_case`
  (`contract_supervised`'s planner `follow` is flagged). The island
  contract on main (no `supervise:`) and the S6 draft (`call_mrm`
  supervised with `mechanism: timer`) are not flagged; the draft with
  `mechanism: executor` would be (call_mrm's own callback, and
  driver_exit and on_timer on the same executor).
- S5: done (2026-10-15), branch `phase93-s5-r2` (`85c949b7`), not yet on
  main. `ros/period_range.rs`, called after `stated`; `--explain` prints a
  timer-period table, `period-range` refuses an allocated `rate_hz` outside
  its range. The counts are not stored on `RungBudget`: they are read off
  the rows by re-running the arithmetic with the period 1 ms longer and
  differencing DETECT, WINDOWS, ROUTE and SETTLE. The cross-check re-runs
  the period-reaching checks on probes (bisection); a new error or warning
  is a named limit, a finding on a `source: derived` budget is listed as
  following the period. Island: `/mrm_handler/on_timer` 100 ms in
  [31, 111.11] (its `hazard_lights_cmd` `min_rate_hz: 9` promise), then
  `max_age` 500, then the FTTI rows (odd_exit comfortable stop 4403.17,
  hpc_loss emergency stop 4934.33). Executors are per node or per
  container; the island's single spin thread is not visible to the
  checker, so MIN is per node there.

- A supervised settle: done (2026-10-16), rlm v0.1.62 (`settle: { ...,
  supervise: { enters | reaction, mechanism, observes } }`,
  `SafeState.settle_supervise`, `Manifest::supervised_settles()`,
  `Supervision.observes`, model `SafeStateContract.supervise`) and
  play_launch 0.18.1. The island's S6 left the comfortable stop's
  supervision in a contract comment: 0.18.0 raised `supervised-unlowered`
  on a supervised `response`, and its overrun row charged the response
  alone (before it 10454 + budget 943 + 2 + 188.33 + 4421.33 = 16008.67),
  not the settle the response ends. Now:
  - What is supervised is the whole settle, command to standstill: the
    derived profile (or written duration) + the response. The deadline
    runs from the safe state's command (the rung's route end) and adds how
    late the platform sees standstill.
  - The observation delay is derived, never written on the supervision:
    `observes: <node>/<subscriber>` names the input; the delay is its
    period (the slowest of the topic's `rate_hz` and the subscriber's
    `min_rate_hz`) + its `max_transport` (the subscriber's, else the
    topic's), each stated once with its own source. Missing any of them
    is `supervised-unlowered`, naming what to state.
  - The overrun row (`supervised-fit`): before the settle + the settle at
    the hazard's entry speed + the observation + the detection (tick +
    switch, or tick + longest callback) + the reaction's route and settle,
    against the FTTI. It contains the rung's own total, so when the rung
    already exceeds the FTTI the rung's row is the error and the overrun
    row is `info` saying so (the island's two variants keep their single
    `ladder-rung-budget` error).
  - LEVEL: the settle's terms (response, written duration) are
    `supervised` (an analysed one stays `bound`); `measured-unsupervised`
    no longer fires for them. CHECKED BY: `runtime monitor
    (settle-deadline: reaction <r>, <mechanism>, standstill seen on
    <subscriber>)`. A supervised `response` alone stays
    `supervised-unlowered`, pointing at the settle.
  - Model: `safe_state.supervise: { response_ms, observe_ms, observes,
    reaction, mechanism }` on the node path; the realizer arms due =
    command + settle (written, or the profile at the entry speed +
    `response_ms`) + `observe_ms`. No realizer reads it yet (the island's
    route 2 is configured by hand); the field census baselines
    `observe_ms` and `settle_supervise` with that reason.
  - Island: `odd_exit comfortable_stop (...operate.safe_state.settle)
    overrun`: before it 10454.00 + settle 10939.67 (profile 9996.67 +
    response 943) + observation 156 (/mrm_handler/kinematic_state: period
    100 + max_transport 56) + detection 2 + island.emergency_stop 188.33 +
    settle 4484.33 = 26224.33 ms of 30,000 (the island emergency response at 319 ms). Every other row unchanged.
  - Tests: `a_supervised_settle_charges_the_whole_settle`,
    `a_supervised_settle_without_its_observation_is_refused`,
    `a_supervised_settle_over_the_interval_is_refused_once`,
    `a_lowered_settle_supervision_reaches_the_model` (manifest_check, on
    `contract_supervised` edited); rlm `a_settle_is_supervised_whole`.

## Not in this phase

- The runtime: nano-ros's deadline-armed route monitor and reaction hook
  (island S3), the board's timer rung and watchdog (island S4).
- Lowering a supervised transport or staleness bound (a transport needs a
  shared clock). They are refused, not dropped. A settle lowers since
  0.18.1 (a standstill deadline, "A supervised settle" above).
- Platform facts by name (play_launch phase 91's profiles):
  `supervision: { tick, switch }` is written per contract until then.
- A release: S1 needs rlm v0.1.61, S2 and S5 a play_launch release pinning
  it; coordinated outside this phase.
