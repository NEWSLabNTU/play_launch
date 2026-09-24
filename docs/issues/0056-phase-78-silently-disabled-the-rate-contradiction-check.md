---
id: 56
title: "phase 78 silently disabled `rate_priority_contradictions` for every contract that states rates the way authors write them"
status: open
type: correctness
severity: high
---

# 0056 - a check that no longer fires, and a design question about why

**Repo:** `play_launch` at `5455fda3`
**Affects:** `src/ros-launch-resolve/resolve/src/ros/sched_loader.rs`
(`rate_priority_contradictions`, the loop at ~`:977` and its message at
~`:999`); broken by `07be64dd` (phase 78 W2), which switched
`derive_sched_plan` to `mapper_input_from_model`

## Symptom

`check_legacy_toml_with_contradicting_contract_facts_warns_but_succeeds`
expects a warning citing both the contract and the platform file when a
hand-written `system.toml` gives a slow node higher priority than a fast one.
No such warning is emitted. The two warnings that ARE printed are unrelated
`dangling-entity` findings; there is no scheduling-warning block at all.

An absent warning, not a wrong one.

## Cause

`rate_priority_contradictions` is `filter_map(|n| n.rate_hz.map(...))`. Since
phase 78, `MapperNode.rate_hz` is **the fastest timer trigger and nothing
else** (rlm `derive/src/lib.rs:216`). The test's contract declares its rates
the way the documentation teaches an author to — `pub.<ep>.min_rate_hz` plus
a topic-level `rate_hz` — and has no paths at all, so both nodes arrive with
`rate_hz: None` and the scan iterates over nothing.

Before `07be64dd` the live derivation was `sched_derive::extract_rate_hz`,
documented as "the **max** of every declared rate fact … both the topic-level
`rate_hz` … and the endpoint-level `pub.<ep>.min_rate_hz`" — exactly the two
things this contract declares. `f7adf641` (W3) then deleted that function.

**It did work**: introduced in `70cec9d1` (2026-07-16, phase 41.4) and never
edited since. And nothing noticed for three days because the only test that
exercised it lives in `manifest_check`, which did not compile from `1c27ba5c`
until `d20936d2` — a window that spans all of phase 78.

The unit tests stayed green throughout because
`derive_rate_contradiction_from_override_is_warned` and
`contradiction_warning_cites_contract_and_platform_file` build their index
with `Trigger::Timer` directly (`index_with_rates`, `sched_loader.rs:2445`),
so they never exercised the promise path. The integration test was the only
thing reading a rate the way a contract author writes one.

## The design question, which must be answered before the fix

Phase 78's ruling is about RANKING: a promise is not a period, so
`rate_monotonic` must not rank a node whose only rate fact is a promise. That
argument is sound and this issue does not reopen it.

But a contradiction check is not a ranking. It compares what the AUTHOR
declared against what the platform file assigns, and its value is precisely
that it catches a hand-written priority table disagreeing with the contract's
own stated intent. An author who writes `min_rate_hz: 100` on one node and
`10` on another, and then a `system.toml` that inverts them, has written a
contradiction — whether or not either number is a period the mapper may rank
by.

So: should `rate_priority_contradictions` read the DECLARED facts
(`topics.<t>.rate_hz`, `pub.<ep>.min_rate_hz`) rather than the mapper's
`rate_hz`? My reading is yes — the check exists to compare an author's stated
intent against a hand-written table, and phase 78 narrowed a different
question. But it is a ruling, not an obvious fix, and it belongs in rlm's
design log beside #55 and #56.

## Impact

High for what it is: a silent loss of a safety-adjacent check on the legacy
`system.toml` bridge, which is the path nano-ros still uses (phase 41.6 keeps
it until they migrate). A contradiction between a hand-written priority table
and the contract is exactly the class of error that bridge needs to report,
and it has reported nothing since 2026-09-22.

## Fix direction

1. Answer the question above. If declared facts are the right input, give the
   scan its own accessor rather than widening `MapperNode.rate_hz`, so phase
   78's ranking ruling stays intact.
2. Whatever the answer, the integration test must exercise the shape a
   contract author actually writes.
3. Two gaps worth closing in the same change:
   `deadline_priority_contradictions` has NO integration coverage at all, and
   nothing anywhere asserts that a promise does **not** raise a contradiction
   — which is phase 78's actual ruling. Keep the promise-only contract as a
   negative test beside a migrated positive one.

## Provenance

Diagnosed 2026-09-25 by a one-variable probe: the same launch file, the same
`system.toml` and the same overlay, changing only the contract's rate facts
from promises into timer paths — the warning appears immediately, with both
file citations the test asserts.
