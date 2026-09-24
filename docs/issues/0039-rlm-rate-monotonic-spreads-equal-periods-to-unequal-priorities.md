---
id: 39
title: "rlm: `rate_monotonic` gives two 30 Hz nodes priorities 40 and 30, by name; `chain_aware` collapses the same tie"
status: resolved
type: design-question
severity: low
---

# 0039 - is an alphabetical priority between equal-period nodes a decision or an accident?

**Repo:** `ros-launch-manifest` (rlm), `origin/main` at `ea5cbea`; filed here
because rlm has no tracker.
**Affects:** `sched/src/mapper.rs:214-222` (`spread_priority`), `:262-288`
(`RateMonotonicMapper`), `:425-440` (test `rate_monotonic_ties_broken_by_name_asc`);
`docs/scheduling.md:194-197`, `:239-240`

## Symptom

The island platform file (brief C section 2.3) ranks four nodes with
`rate_monotonic` in band 10-40: two at 30 Hz, two at 10 Hz. The plan is
`mrm_emergency_stop_operator` 40, `stop_mode_operator` 30,
`mrm_comfortable_stop_operator` 20, `mrm_handler` 10. The two 30 Hz nodes
differ by 10 priority levels, and the two 10 Hz nodes by another 10, for
no reason the contract states: the order within a tie is the node name.

## Cause

`mapper.rs:273-283` sorts by `rate_hz` descending, `then_with(|| a.name.cmp(&b.name))`,
and `build_ranked_plan` hands each RANK its own tier via `spread_priority`
(`:214-222`): rank `i` of `n` gets `band.max - round(span * i / (n-1))`.
Nothing looks at whether rank `i` and `i+1` have the same rate. The test at
`:429-440` asserts only the ORDER of two 50 Hz nodes (`/a` before `/b`),
not that they share a priority, so the current output is what the tests
protect.

`docs/scheduling.md:194-197` documents the mapper as "rate descending, ties
by node name" and `:196` says "a narrow band produces ties, never
inversions", so a tie in the output is not considered wrong; the mapper
just never produces one on purpose. `deadline_monotonic` (`:293-320`) has
the same shape.

The crate's own third mapper does it the other way: `chain_aware` "Items
with exactly equal (criticality, budget) collapse into one rank
(`tie_group`)" (`docs/scheduling.md:239-240`,
`chain_aware_mapper.rs:171-177`, `:381`) and then decides `SCHED_RR` vs
`SCHED_FIFO` for the tied set, warning `UnmitigatedPriorityTie` when FIFO
cannot be made fair (`:524`). So one mapper says an exact tie is a fact to
preserve and mitigate; the other silently orders it by name.

`docs/design-issues.md` has no entry on it (grepped `rate_monotonic`,
`tie`, `equal period`).

## Why it matters

Rate-monotonic theory assigns equal periods equal priority; any fixed
order among them is schedulable, but a DIFFERENT priority is a policy
statement (this node preempts that one), and here the policy is the
alphabet. Renaming a node changes who preempts whom. The nano-ros realizer
consumes the same `SchedPlan` for the RTOS side of the island, where
distinct priorities also change thread-pool grouping. And the linear spread
wastes band: four nodes with two distinct rates take four levels of a
31-level band where two would do, which matters once overrides and
reservations compete for the same band.

## Options

1. Design choice, keep it: document in `scheduling.md` and in the
   `RateMonotonicMapper` doc comment that ties are deliberately broken to
   distinct priorities by name, and why (determinism without RR). Add a test
   that asserts the distinct priorities so the behaviour is protected on
   purpose rather than by omission.
2. Defect, fix it: collapse exact `rate_hz` (and `deadline_us`) ties into
   one tier with all tied nodes as `members`, spread over the number of
   DISTINCT values, and reuse `chain_aware`'s `SCHED_RR`-if-the-slice-fits
   decision plus `UnmitigatedPriorityTie` for the FIFO case. Update
   `rate_monotonic_ties_broken_by_name_asc` to assert equal priority and
   stable member order.

Option 2 makes the three mappers agree on what a tie means and is the one
this issue leans toward; either way the answer belongs in
`docs/design-issues.md`.

## Provenance

Brief A (`brief-A-rlm-grammar.md`, section 4.3) and brief C (section 2.3
and "Facts for the deck": "equal-rate nodes do not get equal priorities"),
2026-09-18; re-verified against rlm `origin/main` `ea5cbea` (worktree
`rlm-gaps`) on 2026-09-21.

## Resolved 2026-09-22 — rlm v0.1.38, option 2

`rate_monotonic` and `deadline_monotonic` collapse an exact tie into ONE tier
carrying its members, spread over the number of DISTINCT values, and take
`chain_aware`'s `SCHED_RR`-if-the-slice-fits decision for the tied set.
Renaming a node no longer changes who preempts whom.

Two things the issue did not anticipate. `deadline_monotonic` had the
identical test protected by the same omission and needed the same fix. And
`rr_policy_for_ties` had to take a period CLOSURE rather than `&MapperInput`,
because it read periods off `node.paths`, which the two simple mappers never
populate — so they would have seen `None` on every tie and could never have
derived RR.

Band-compression ties take the same decision, deliberately: judging a
mapper-created tie by a different rule than a derived one would be a third
policy.
