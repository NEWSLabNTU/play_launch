---
id: 46
title: "`max_age` cannot count toward the FDTI of an `on: omission` hazard, and
  `hazards.<h>.on` takes exactly one fault kind -- so a guard with both a lease
  and an age limit must choose which of the two the budget is allowed to see"
status: resolved
type: design-question
severity: medium
---

# 0046 - a semantic question about `on:`, plus one diagnostic that contradicts the rule it reports

**Repos:** `play_launch` at `9a610488` (checker binary 0.10.0), and
`ros-launch-manifest` (rlm) at the pinned `v0.1.40` for the grammar
(`HazardDecl`, `FaultKind`). Filed here because rlm has no tracker and the
rule is in this tree.
**Affects:** `src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs`
(`detector_interval_ms` at `:1113-1147`; the FDTI loop inside
`check_fault_reaction` from `:1406`, the per-guard detector search at
`:1545-1572`, the `hazard-unguarded` message at `:1574-1586`)

**This is a question, not a bug report, for its first half.** The rule may be
exactly right. The second half -- the `hazard-unguarded` message -- is wrong
whatever the answer to the first.

## Where it comes from

The Autoware Reference Design WG's "L4 Safety Island for Autoware - Software
Design", version 20260918, section 8.1, prints the eleven HPC-to-island
signals as a table with the columns Signal, Topic, Type, Rate, **Age limit**
and Protection. Every row has an Age limit: 300 ms for most, 60 ms for the
kinematic and acceleration references, 2 s for the refuge, and 20 ms for the
100 Hz heartbeat. The design gives no row without one.

Transcribed into a contract, the Age limit column lands on the subscriber's
`max_age`, which is the only key that means it. The heartbeat row, which is
also the guard of the design's own HPC-loss hazard, then reads:

```yaml
heartbeat:
  min_rate_hz: 100
  max_age: 20ms
  qos: { depth: 1, liveliness: manual_by_topic, lease_duration: 30ms }
  on_violation:
    on: [omission]
    reaction: declare_hpc_lost
    within: 30ms
```

with

```yaml
hazards:
  hpc_loss:
    guards: [/si/in/heartbeat]
    on: omission
    ftti: 70ms
    reaction: island.mrm_command
```

The 30 ms lease is section 9.2's "3 missed 100 Hz heartbeats". The 20 ms is
section 8.1's Age limit. Both are declared; only one is counted.

## Measured

Four runs of `play_launch check` against the same four-node stand-in launch
tree, changing one thing each time. Diagnostic text is pasted verbatim except
that the reaction route is elided as `...`: the tool prints it with a unicode
arrow between hops, and this file is ASCII. The `--` in run 4's message is the
tool's own em dash, transliterated for the same reason. No number is changed.

**1. As above.** Detection is the lease:

```
info[fault-reaction-budget]: hazard 'hpc_loss': detection 30.00ms
(/si_hpc_supervisor/heartbeat detects within 30.00ms) + reaction 40.00ms
(... = 40.00ms) = 70.00ms fits the fault-tolerant time interval 70.00ms with
0.00ms of slack
```

**2. Delete `max_age: 20ms` entirely.** Byte-identical diagnostic, still
`detection 30.00ms`. The age limit contributes nothing.

**3. Set `max_age: 5ms`, six times tighter than the lease.** Still
`detection 30.00ms`. It is not a minimum over the two; the age limit is not
in the set at all.

**4. Delete the `lease_duration` and keep `max_age: 20ms`.** The hazard now
fails:

```
error[hazard-unguarded]: hazard 'hpc_loss' guards '/si/in/heartbeat', but no
subscriber of it declares an `on_violation` reaction with a detector
(lease_duration, deadline, max_age or min_rate_hz) -- nothing would ever
notice
```

That message names `max_age` as one of the four things that would satisfy it,
on a contract that declares `max_age`.

## The rule, and why it is defensible

`detector_interval_ms` gates each mechanism on the hazard's fault class:

```rust
let wants = |k: F| kinds.is_empty() || kinds.contains(&k);
if wants(F::Omission) {
    // The lease is the only omission MECHANISM. `min_rate_hz` is a
    // requirement, not a detector: nothing fires when a period passes
    // unless a QoS deadline (below) or an application watchdog is
    // declared. Counting the period here made a 50 Hz floor "detect" a
    // dead lidar in 20ms while the real lease was 100ms.
    consider(qos.lease_duration.map(|d| d.as_millis_f64()));
}
if wants(F::Late) {
    consider(qos.deadline.map(|d| d.as_millis_f64()));
    consider(props.max_age.map(|d| d.as_millis_f64()));
}
```

`kinds` is the HAZARD's `on:`, not the subscriber's `on_violation.on` --
confirmed by a fifth run that set the subscriber to `on: [omission, late]`
and got `detection 30.00ms` unchanged.

The partition matches rlm's own type documentation:

```rust
pub enum FaultKind {
    /// No message at all -- liveliness lease expired.
    Omission,
    /// A message arrived, too late -- QoS deadline or `max_age` violated.
    Late,
    ...
}
```

So by the grammar's own definitions this is correct. `max_age` bounds the
staleness of a message that ARRIVED; omission is the absence of one. The
comment beside the omission branch is the scar of getting exactly this wrong
once already, for `min_rate_hz`, and the lesson it records -- a requirement on
data is not a mechanism that fires -- is the right lesson.

## The question

Is an age limit a mechanism, or a requirement?

The code answers both ways in one function. Under `Late` it is a MECHANISM,
counted as a detection interval. Under `Omission` it is absent, which reads as
"requirement". But a staleness bound on the last-received sample does fire
when messages stop: the sample goes stale 20 ms after the last one arrives
whether the next one is late or never comes. Omission is the limiting case of
lateness. If `max_age` is a real mechanism for `Late` -- if something in the
node is actually evaluating the age of the newest sample -- then that same
something notices an omission, and it notices it at 20 ms rather than at the
lease's 30 ms.

The counter-argument is just as strong, and is why this is filed as a
question: nothing in the contract says anything evaluates `max_age`
periodically. It may be checked only on arrival, in which case it fires never
during an omission, and counting it would be the `min_rate_hz` mistake again
with a different field. `DetectMechanism` in rlm distinguishes `Qos`,
`Diagnostics` and `Application`, so the vocabulary for saying which one a
subscriber uses already exists -- it just is not consulted here.

## The part that is not a question: `on:` takes one kind

`HazardDecl.on` is `Option<FaultKind>`, a single value. A list is a hard parse
error:

```
error[manifest-parse]: could not parse contract .../e.contract.yaml: at
'hazards.hpc_loss.on': expected a string, got a list.
```

So an author whose guard carries both a lease and an age limit has three
spellings and no good one:

| spelling | detection | what it claims |
| --- | --- | --- |
| `on: omission` | 30.00ms (lease) | the age limit is not a detector |
| `on: late` | 20.00ms (age) | the vehicle is reacting to lateness, not loss |
| `on:` omitted | 20.00ms (min over all) | any fault class -- a weaker claim than the design makes |

The third row is measured, not inferred: deleting `on: omission` from the
hazard gives

```
info[fault-reaction-budget]: hazard 'hpc_loss': detection 20.00ms
(/si_hpc_supervisor/heartbeat detects within 20.00ms) + ... = 60.00ms fits
the fault-tolerant time interval 70.00ms with 10.00ms of slack
```

That is the sharp end of it. **Saying LESS about the fault buys 10 ms of
slack.** A contract that names its fault class precisely is scored more
harshly than one that declines to, which inverts the incentive a safety
contract should create. Whatever is decided about `max_age`, that ordering is
worth looking at on its own.

## Impact

Narrow today, and it is arithmetic rather than a crash. It changes the FDTI
half of `fault-reaction-budget`, `ladder-rung-budget` and anything else
reading `fdti_worst`. On the L4 fixture it is the difference between 0.00 ms
and 10.00 ms of slack against a 70 ms FTTI, which is the difference between a
budget that is closed and one that is not.

It is also not the phase-70 unread-field shape: `max_age` is read in four
other places in this tree -- `causal_graph.rs:258`, `model_builder.rs:282`,
the `lifespan-age` rule at `manifest_loader.rs:3268-3286`, and the
`--emit diagnostics-params` restatement at `verbs/check.rs:431`. It is inert
in exactly one computation, for exactly one fault class.

## Fix direction

Three separable pieces, in confidence order.

1. **The `hazard-unguarded` message is wrong and should be fixed regardless.**
   It lists `lease_duration, deadline, max_age or min_rate_hz` as the
   detectors that would satisfy it; for `on: omission` only the first can, and
   `min_rate_hz` can never satisfy it under any fault class, since
   `detector_interval_ms` does not read the field at all. The message should
   name the mechanisms that count FOR THIS HAZARD'S `on:`, and say which
   declared-but-uncounted ones it saw. Run 4 above is the fixture: a contract
   that declares `max_age` failing with a message that recommends `max_age`.

2. **Decide and write down whether an age limit detects an omission.** If yes,
   `consider(props.max_age)` moves into the omission branch, or into both, and
   the comment beside it has to explain why `max_age` is a mechanism where
   `min_rate_hz` is not. If no, the docs should say so where an author writing
   an omission hazard will read it, because the design this came from gives
   every signal an age limit and the natural reading is that the age limit is
   what notices.

3. **Consider whether `on:` should take a set.** A guard whose loss is watched
   by a lease AND whose staleness is watched by an age limit is one hazard
   with two mechanisms, and today the grammar makes the author drop one. The
   subscriber's `on_violation.on` already takes a list; the hazard's does not,
   and the hazard's is the one the FDTI reads. That asymmetry is at least
   surprising. A set would also remove the incentive inversion above, since
   `on: [omission, late]` would be a strictly more informative declaration
   than omitting `on:` rather than an unspellable one.

## Why this is here and not in nano-ros

The rule is in this tree, at
`src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs`, and
`ros-launch-resolve` is no longer a separate repository. nano-ros consumes the
same contract file but does not implement this computation: its use of
`max_age` is the runtime age table (`age_rows` ->
`AgeMonitorSpec` -> `check_age`, the `max-age-runtime` violation), which is a
different consumer with no fault-class gate and is unaffected. The static FDTI
question is entirely this checker's.

## Provenance

Found 2026-09-24 while encoding the WG's L4 design as a contract
(simple-autoware-safety-island, `docs/roadmap/phase-6-l4-design-demo.md`, unit
phase6-W5 item 3). The five runs above were made against
`docs/demo-l4/l4_designed.launch.xml` and its contract sidecar, copied to a
scratch directory and edited one line at a time, with the 0.10.0 binary at
`install/play_launch/lib/play_launch/play_launch`. The fixture tree itself was
not modified.

## Ruled 2026-09-25 -- phase 82 W1-W3

Settled by `docs/roadmap/phase-82-what-a-detector-may-see.md`, units W1, W2
and W3, with the grammar half in `ros-launch-manifest` v0.1.43. All three
fix-direction items are taken, and the incentive inversion is closed by a
rule rather than by a choice about `max_age`.

**The rule.** A fault-detection interval must cover EVERY fault class the
hazard claims, so

    FDTI = max over the hazard's classes of (min over that class's mechanisms)

and a claimed class with no mechanism is `hazard-unguarded` for that class.
An omitted `on:` claims every class the guard's detectors can observe
(omission, late, loss -- never `reported`, which only the author can know)
and takes the max over those some detector covers. Saying less therefore
never buys slack.

**Item 3, `on:` is a set (W1).** `on: omission` and `on: [omission, late]`
both parse in rlm v0.1.43; `on: []` is a parse error ("state at least one
fault class or omit the key"). The three spellings in the table above now
agree on the L4 fixture: `on: omission`, `on: [omission, late]` and `on:`
omitted all read `detection 30.00ms` and close the 70 ms budget with 0.00 ms
of slack. The omitted spelling read 20.00 ms before.

**Item 2, the question: an age limit is a mechanism for an omission exactly
when something evaluates it (W3).** `max_age` counts toward `omission` if and
only if the subscriber's `on_violation.mechanism` is `diagnostics` or
`application`: the node evaluates the age of its newest sample on its own
clock, so it notices silence. Under `qos`, the default, only DDS liveliness
and deadline events fire, an age checked on arrival never fires while nothing
arrives, and counting it would be the `min_rate_hz` mistake again. Both
arguments in "The question" above were right, and `DetectMechanism` was the
vocabulary that separates them. Run 4 with `mechanism: application` added
reads `detection 20.00ms` with 10.00 ms of slack -- an honest 10 ms this
time, bought by declaring a detector rather than by declaring less.

**Item 1, the message (W2).** `hazard-unguarded` names only the mechanisms
that count for the class it reports, and then every detector the reacting
subscribers DO declare that this hazard cannot use. `min_rate_hz` is never
listed. Run 4 now reads:

```
error[hazard-unguarded]: hazard 'hpc_loss' guards '/si/in/heartbeat' `on:
omission`, but no subscriber of it that declares an `on_violation` reaction
has a detector that counts for omission -- nothing would notice. What
counts: a QoS `lease_duration` on the subscriber or the topic. `max_age:
20ms` is declared on '/si_hpc_supervisor/heartbeat' but counts only for `on:
late`: under `on_violation.mechanism: qos` nothing evaluates it while no
message arrives
```

Pinned by `tests/fixtures/contract_fault_kinds` (one hazard per rule) and
`a_hazard_is_timed_by_the_slowest_fault_class_it_claims` in
`tests/tests/manifest_check.rs`, and by four unit tests beside
`detector_interval_ms`.
