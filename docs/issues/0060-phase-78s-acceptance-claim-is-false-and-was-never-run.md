---
id: 60
title: "Phase 78's `min_rate_hz` acceptance claim is false, and the grep it names was never automated"
status: resolved
type: doc-defect
severity: low
---

# 0060 - an acceptance gate that was a sentence

**Repo:** `play_launch` at `873b0744`
**Affects:** `docs/roadmap/phase-78-one-derivation-two-consumers.md` (the
acceptance list); `src/ros-launch-resolve/resolve/src/ros/sched_loader.rs`
**Found:** reviewing the three items left unfiled after the bugfix campaign.

## What it claims

The last line of phase 78's acceptance list:

> - No `min_rate_hz` read remains in any scheduling path of this repository
>   (`git grep min_rate_hz src/ros-launch-resolve/resolve/src/ros/sched_*`
>   returns nothing).

That grep returns five hits in `sched_loader.rs` and three in
`sched_derive.rs`. One of them is a live read:
`declared_rate_facts` at `sched_loader.rs:1116`.

## Why the claim is false, and why that is correct

Not drift. Issue #0056 put the read back **on purpose**. Phase 78's narrowing —
a node's `rate_hz` is the fastest timer trigger it declares, never a declared
floor, because a promise is not a period — also reached
`rate_priority_contradictions`, the one rule whose entire subject is an author
who writes `min_rate_hz: 100` on one node and `10` on another and then expects
the first to outrank the second. With the declared rates gone the rule had
nothing to compare and went silent: a check disabled by a fix to a different
problem.

The resolution was to keep both: `declared_rate_facts` collects the authored
rates for `input_with_declared_rates`, its only caller, which builds a
**widened copy** of the mapper input and hands it to that rule alone.
`MapperNode.rate_hz` stays narrow, so no promise reaches the ranking.

So the defect is not the read. It is that the acceptance list still states an
invariant the code deliberately broke, and a reader who trusts it would delete
the one licensed read and silence the contradiction scan a second time. A claim
that is FALSE is worse than one that is missing, because a reader acts on it —
the argument `just check-rt-docs` was built on.

## Why the existing gate did not catch it

`check-rt-docs` is scoped on purpose to `docs/guide` and the two CLIs' clap
definitions — what a USER reads. Its own comment rules roadmap entries out:
*"they are records of what was decided then, and stamping them superseded is
the right fix there, not rewriting them."* That ruling stands, so the
correction is a supersession stamp, not a rewrite.

What was genuinely missing is the automation. The phase named a `git grep` and
nothing ran it: no recipe, no CI job, only a sentence. Same shape as #0040 (an
IR suite nothing built) and #0046 (an index advertising solved work) — the
check exists as prose, so it cannot fail.

## Fix

Two parts.

1. The claim is stamped **superseded by #0056** in place, naming the one read,
   its reason, and the rule that needs it. The invariant that survives is
   "exactly one read, and it is named".

2. `scripts/check_sched_rates.py` + `just check-sched-rates`, wired into
   `just check`. It fails when a **second** read appears — one read with a
   decision behind it is a decision; two is the phase being undone one call
   site at a time. The allowlist is keyed by **function name, not line
   number** (a line rots on the next edit above it, and a gate that fails for
   being out of date gets deleted) and each entry carries its reason, so the
   exception is written down rather than forbidden. It also fails in the other
   direction: an allowlisted read that disappears or is renamed is reported,
   so the script cannot outlive the thing it licenses. A new `sched_*.rs` file
   it has never seen is a failure too, rather than a silent skip.

Occurrences inside `#[cfg(test)]` are not reads. `sched_derive.rs` builds a
manifest carrying `min_rate_hz: Some(30.0)` as the input that pins phase 78's
own guarantee — the test asserting a promise is never ranked needs a promise to
offer — so counting it would fail the gate on the evidence for the rule it
enforces.

## Verification

The gate corrected its own author on first run: I had allowlisted
`input_with_declared_rates`, and the read is one level down in
`declared_rate_facts`, which it named and refused. Both halves fired — the
unlicensed read, and the allowlist entry with nothing matching it.

Falsifiability checked rather than assumed: injecting
`fn injected_regression(x: &SomeProps) -> Option<f64> { x.min_rate_hz }` above
the test module in `sched_derive.rs` fails the gate naming file, line, function
and line text; restoring the file passes. Current state:

```
  ok: 2 sched file(s), 1 licensed `min_rate_hz` read(s): sched_loader.rs:1116 (declared_rate_facts)
```
