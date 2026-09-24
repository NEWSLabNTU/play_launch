---
id: 54
title: "four `manifest_check` tests fail against a current binary — all four encode pre-phase-78 scheduling semantics (three fixed, one diagnosed)"
status: open
type: correctness
severity: medium
---

# 0054 - the first honest run of this suite since 1c27ba5c

**Repo:** `play_launch` at `d20936d2`
**Affects:** `tests/tests/manifest_check.rs:941` (`w2_derived_overrun_reaches_the_model`),
`:695` (`check_legacy_toml_with_contradicting_contract_facts_warns_but_succeeds`),
plus `w2_reservation_is_refused_for_a_node_that_claims_concurrency` and
`w2_an_unenforceable_miss_action_is_reported_not_downgraded`

## Why this is only being filed now

The suite has not compiled on `main` since `1c27ba5c` (phase 82 W1-W3) left a
test without its closing brace, and before that its spawns inherited the
ambient environment, so most of it failed on the library path wherever the
invoking shell had not been sourced (issue #0051). Both are fixed as of
`d20936d2`, and the binary was rebuilt so `install/` matches the source. This
is what the suite says once it can actually run: **30 tests, 26 passed, 4
failed.**

Three of the four were previously dismissed as "pre-existing" in this
session's reports, which was true but incomplete — they are pre-existing AND
real. They were verified against two different manifest tags (v0.1.41 and
v0.1.42) with identical output, so they are not a pin artifact, and they
survive a full `just build`, so they are not issue #0020's stale-binary shape
either.

## The failures

**1-3. The three `w2_*` tests — a `miss:` declaration does not reach the model.**

```
a `miss:` declaration must reach the model as deadline_policy:
meta:
  version: 1
  ...
```

`w2_derived_overrun_reaches_the_model` asserts a declared `miss:` arrives as
`deadline_policy` in the emitted model; it does not.
`w2_an_unenforceable_miss_action_is_reported_not_downgraded` and
`w2_reservation_is_refused_for_a_node_that_claims_concurrency` are the same
family — model facts phase 70 W2 said it carried.

This matters beyond the tests: phase 68 W5's whole point was that a fact which
never reaches `model::PathContract` cannot be seen by nano-ros at all, because
it builds its `MapperPath` from the model rather than from the manifest. If
`miss` is not arriving, that seam is open again for exactly the reason W5
closed it.

**4. A rate contradiction between a legacy `system.toml` and a contract
produces no warning.**

```
expected a rate contradiction warning citing both nodes, got:
  ... Loaded 1 manifest(s) [1 overlay, 0 provider] (0 scopes ..., 0 errors, 2 warnings)
  Scheduling platform file [explicit]: /tmp/.../system.toml
  Scheduling (posix, mapper=manual): 2 tier(s)
```

Two warnings are emitted and neither is the contradiction the test asks for.
The legacy TOML bridge is deprecated but supported until nano-ros migrates
(phase 41.6), so a contradiction between it and a contract is exactly the case
the bridge needs to report.

## Fix direction

Take the three `w2_*` together — they are one question (does a `miss:`
declaration survive lowering?) and the answer is in `model_builder.rs`'s path
lowering, which is where `max_jitter` and `miss` were added in manifest
v0.1.19. Check whether the field is lowered at all, and whether the
`effective_trigger()` correction from phase 58 W2 moved the branch that reads
it.

The fourth is separate: find which two warnings ARE emitted and whether the
contradiction check runs at all on the `manual` mapper path.

Neither should be closed by adjusting the test until the product behaviour is
understood — three of these assert facts that shipped phases claim to carry.

## Provenance

Observed 2026-09-25 after fixing #0051 and rebuilding; the `w2_*` three were
also seen at v0.1.41 and v0.1.42 by an earlier agent, with identical output.

## Correction 2026-09-25 — the `w2_*` diagnosis above is wrong

I wrote that "a `miss:` declaration does not reach the model". It does.
Resolving the fixture and reading the model shows `miss:` present on the path
contract, with its `tolerate_n`, `tolerate_w` and `action`:

```
miss:
  tolerate_n: 1
  tolerate_w: 50
  action: abort
```

What is absent is `deadline_policy`, which is a TIER-level field, not a path
one — and it is absent because no reservation is derived. The resolve says so:

```
WARN scheduling: `/w/guard` priority 0 outside band [10, 40], clamping to 10
WARN scheduling: `/w/rt`    priority 0 outside band [10, 40], clamping to 10
sched_class: SCHED_OTHER
```

Priority 0 means the mapper had no timing fact to rank by. And it had none
because **`contract_w2` declares no timer anywhere** — every path is
`trigger: { input: [...] }`, and the only rate facts in the file are
`min_rate_hz: 50` promises on endpoints.

That is phase 78's ruling working exactly as documented:

> `MapperNode.rate_hz` becomes the fastest timer trigger among the node's
> paths and nothing else. […] The one visible change: `rate_monotonic` no
> longer ranks a node whose only rate fact is a promise. […] the alternative
> — a promise as a fallback rate — is the reading this phase exists to remove.

`SCHED_DEADLINE` needs a period, the period comes from `1/rate_hz` propagated
from a chain's source, and a promise is no longer a rate. So no period, no
reservation, no `deadline_policy`, and the three assertions fail.

**These tests are correct about the old model and wrong about the new one.**
The fixture predates phase 78 and encodes the semantics it removed. The fix is
to the FIXTURE — give it a timer trigger, or an `external: pub` source with a
declared rate — not to the product, and certainly not by relaxing the
assertions, which would delete the `miss`-to-`deadline_policy` coverage
entirely.

Worth stating because the earlier reports (and this issue's own first draft)
called them "pre-existing" and then "pre-existing AND real": they are real
failures of a real assertion, and the product is behaving as designed. The
honest label is *the fixture was not migrated with the phase that changed the
rule*.

The fourth failure — the missing rate-contradiction warning on the legacy
`system.toml` path — is untouched by this correction and still needs
diagnosis.

## Resolution 2026-09-25 (part 1) — `contract_w2` migrated to post-78 semantics

The fixture, not the assertions. Three edits, all in
`tests/fixtures/contract_w2/launch/`:

1. **A real source.** `/w/tick` was `external: pub` with a declared
   `rate_hz: 50`. It is now published by a new node `src`, whose one path is
   `trigger: { timer: { rate_hz: 50 } }` — the only rate fact post-78 will
   rank by. `src` is added to `bringup.launch.xml` as a fourth node.
2. **A scope path.** `paths.tick_to_out` (`/w/tick` → `/w/out`,
   `max_latency: 40ms`) makes a chain form, which is what carries the source's
   20 ms period to `rt` and `guard` — `sched_loader::node_period_us` takes a
   node's own `rate_hz` or, failing that, the period of its chain's Boundary.
   Without a chain there is no period, and without a period no reservation.
   40 ms holds the honest total: 20 ms sampling_cost + 13 ms event segment
   (rt 5 + guard 8) = 33 ms. (30 ms was tried first and reported
   `scope-budget` at 33.00ms — the checker doing its job.)
3. **`mapper: deadline_monotonic`** in `bringup.system.posix.yaml`, was
   `rate_monotonic`. This is the load-bearing one and it is forced:
   `RateMonotonicMapper` filters `input.nodes` on `n.rate_hz.is_some()`, and
   post-78 that is the fastest TIMER trigger. In this system only `src` has
   one, so `rate_monotonic` would rank the source alone and drop all three
   nodes the fixture is about onto the default tier. What `rt`, `guard` and
   `slow` actually declare is DEADLINES (5/8/40 ms), which
   `MapperNode.deadline_us` carries (min over `max_latency` and
   `srv_max_response`), so the mapper that reads the facts present is the one
   to use.

Consequentially the per-topic `rate_hz` copies and the publishers'
`min_rate_hz` promises were deleted (they are consequences of the timer —
phase 70 W3); the subscribers keep theirs, being requirements.

**What changed about what the fixture demonstrates.** Nothing was weakened,
one thing was added and one moved:

- `rt` refused a reservation for claiming concurrency: still demonstrated, and
  now actually reached — pre-migration the refusal was unreachable because the
  node never entered the RT band at all.
- `guard`'s `miss:` → `deadline_policy: fault` + `SCHED_DEADLINE`: still
  demonstrated.
- `slow`'s `max_jitter` on a best-effort node: unchanged (it fired before too;
  `w2_a_jitter_bound_on_a_best_effort_node_is_reported` was the one w2 test
  passing).
- **Added**: the fixture now also demonstrates period propagation along a
  derived route, which is what makes a reservation derivable at all. That is
  new coverage, and it is the phase-78 rule stated positively.
- **Moved**: the fixture no longer exercises `rate_monotonic`. It never
  exercised it meaningfully after 78 — it produced one ranked node and three
  defaulted ones.

Before → after (`cargo nextest run --test manifest_check`, 30 tests):

| test | before | after |
|---|---|---|
| `w2_derived_overrun_reaches_the_model` | FAIL | PASS |
| `w2_reservation_is_refused_for_a_node_that_claims_concurrency` | FAIL | PASS |
| `w2_an_unenforceable_miss_action_is_reported_not_downgraded` | FAIL | PASS |
| `w2_a_jitter_bound_on_a_best_effort_node_is_reported` | PASS | PASS |
| suite | 26 passed / 4 failed | 29 passed / 1 failed |

`check --sched` on the migrated fixture is otherwise clean: `1 clean, 0 with
errors (0 errors, 0 warnings)`, with exactly the three scheduling warnings the
fixture exists for and the `jitter-range` note.

**One follow-up this owes, outside the migrating agent's scope.**
`src/ros-launch-resolve/resolve/snapshots/contract_fixtures.ranked_plans.txt`
holds a committed `RankedPlan` for `contract_w2/launch/bringup.launch.xml`
(line 658) which `every_contract_fixture_ranks_to_its_committed_plan` (phase
78 W3's gate) compares byte for byte. A fourth node and a chain change that
plan, so the section must be re-taken with
`UPDATE_RANKED_PLAN_SNAPSHOTS=1`. It could not be done here: `cargo test -p
ros-launch-resolve` does not currently compile on `main` (`unresolved import
ros_launch_manifest_derive::TopicView` at `manifest_graph.rs:17`, another
agent mid-edit), and that file is outside this scope in any case. **Re-take
the snapshot and read the diff** — the old section's provenance reads
`derived(chain_aware: non-chain …)` for every member, which is precisely the
false-pass shape phase 68 W4 warned about, so the new one saying `segment
drain` is the point rather than churn.

## Diagnosis 2026-09-25 (part 2) — the legacy-TOML contradiction is the SAME phase-78 cause

Short version: **the contradiction check runs fine on the `manual` mapper
path; the legacy `system.toml` bridge is not at fault. The test's own contract
states its rates as promises, and post-78 a promise is not a rate.** This is
job 1's defect in a second fixture, not a separate defect — contrary to the
"still needs diagnosis / genuinely separate" reading the section above leaves
open.

### Which two warnings ARE emitted

Both are `dangling-entity`, from the cross-scope pass, and neither has
anything to do with scheduling:

```
warning[dangling-entity]: topic '/chatter' has 0 subscribers across the manifest tree (declared in 1 scope(s)) — may be consumed by an external system
warning[dangling-entity]: topic '/status'  has 0 subscribers across the manifest tree (declared in 1 scope(s)) — may be consumed by an external system
```

The test's contract gives both topics a publisher and no subscriber, so these
are correct and incidental. The `2 warnings` in the `Loaded 1 manifest(s)`
line is the same pair counted at load. **There is no scheduling-warning block
at all** in the failing output — not a wrong warning, an absent one.

### Does the contradiction check run on the `manual` path?

Yes, unconditionally. `sched_loader.rs` calls
`rate_priority_contradictions(&input, &plan)` and
`deadline_priority_contradictions(...)` after mapping, outside any
mapper-specific branch (the loop at ~:977, the message at ~:999). The `manual`
mapper only decides how `plan` was built; the scan then compares `plan`'s
priorities against `input`'s facts regardless.

Proven by reconstructing the test by hand and changing ONE thing. Same launch
file, same `system.toml` (talker pinned to 10, listener to 40), same
`--contracts` overlay — only the contract's rate facts restated as timer
triggers:

```yaml
# was:  pub: { chatter: { min_rate_hz: 100 } } + topics.chatter.rate_hz: 100
# now:
talker:
  pub: { chatter: {} }
  paths: { tick: { trigger: { timer: { rate_hz: 100 } }, output: [chatter] } }
listener:
  pub: { status: {} }
  paths: { tick: { trigger: { timer: { rate_hz: 10 } }, output: [status] } }
```

```
Scheduling (posix, mapper=manual): 2 tier(s)
1 scheduling warning(s)
  warning[sched:contradiction]: scheduling: `/pure_test/talker` vs `/pure_test/listener`
  priority order contradicts their `rate_hz` order (contract: …/pure_nodes.contract.yaml
  [overlay]; platform file: …/system.toml)
```

Both halves of the test's second assertion (`contract:` and `platform file:`)
are present too. The bridge works.

### Why it stopped

`rate_priority_contradictions` (rlm `sched/src/validate.rs`) is
`input.nodes.iter().filter_map(|n| n.rate_hz.map(...))` — a node with no
`rate_hz` is simply not in the scan. Post-78 `MapperNode.rate_hz` is the
fastest timer trigger and nothing else (rlm `derive/src/lib.rs:216`). The
test's contract declares `pub.<ep>.min_rate_hz` and `topics.<t>.rate_hz` and
no path at all, so both nodes arrive with `rate_hz: None` and the scan is
empty. `deadline_priority_contradictions` is likewise empty — there is no
`max_latency` or `srv.max_response` anywhere in that contract either.

### Did it ever work, and when did it break

It worked from its introduction until phase 78 W2.

- Introduced `70cec9d1` (2026-07-16, phase 41.4, `check --explain` provenance
  + `contract eject`). `git log -S` on the test name returns that one commit:
  the test has never been edited since.
- Broken by **`07be64dd`** (2026-09-22, phase 78 W2), the commit where
  `derive_sched_plan` switched to
  `ros_launch_manifest_derive::mapper_input_from_model`. At `07be64dd~1` the
  live derivation was `sched_derive::extract_rate_hz`, whose own doc comment
  read "the **max** of every declared rate fact across the node's own
  publications — both the topic-level `rate_hz` … and the endpoint-level
  `pub.<ep>.min_rate_hz`" — exactly the two things this contract declares. So
  talker arrived at 100 Hz, listener at 10 Hz, and the inverted TOML pin
  contradicted.
- `f7adf641` (phase 78 W3) then deleted that function; by then the behaviour
  had already moved.

Nobody noticed because the suite did not compile from `1c27ba5c` (phase 82)
until `d20936d2`, which spans the whole of phase 78.

### Fix direction

Same ruling as part 1: **migrate the test's contract, not the product.**
Replace the two `min_rate_hz` promises and the two `topics.<t>.rate_hz`
copies with a timer path on each node (100 Hz and 10 Hz), as in the probe
above — the topic rates then derive, and the `dangling-entity` pair stays
(harmless, and the test does not assert on warning count).

Two things worth deciding rather than assuming while doing it:

1. The test's name says "contradicting **contract facts**". After 78 the facts
   a contradiction can be raised against are timer rates and declared
   deadlines. Rewriting it with timers keeps it honest; rewriting it with
   `max_latency` instead would exercise `deadline_priority_contradictions`,
   which currently has no integration-level coverage at all.
2. Nothing anywhere now tests that a **promise** does NOT raise a
   contradiction. That is phase 78's actual ruling, and the fixture in front
   of us is exactly the case. Consider keeping the promise-only contract as a
   negative test beside the migrated positive one.

Note the two unit tests that kept passing through this
(`derive_rate_contradiction_from_override_is_warned`,
`contradiction_warning_cites_contract_and_platform_file` in
`sched_loader.rs`) build their index with
`Trigger::Timer` directly (`index_with_rates`, :2445), so they never exercised
the promise path and could not have caught the change. The integration test
was the only thing that read a rate the way a contract author writes one.
