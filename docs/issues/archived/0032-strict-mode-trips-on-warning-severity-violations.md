---
id: 32
title: "`--enforce-rules strict` stops the run on the first violation of ANY severity, so `/rosout` ends it 57 ms in"
status: resolved
resolved_in: fix(#0032) on main, 2026-09-21
type: correctness
severity: medium
---

# 0032 - strict mode could not tell a warning from an error

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affected:** `src/play_launch/src/runtime_enforcement/mod.rs` (`emit`,
`emit_repeatable`)

## Symptom

Under `--enforce-rules strict` the run ended 57 ms after the talker's first
publisher was created, on the `graph-deviation-runtime` WARNING for
`/rosout`; the `rate-hierarchy-runtime` ERROR the contract was written for
needs a 0.5 s window and never got one (2026-09-18, 0.10.0 binary at
`155ed78b`). Every ROS 2 node creates `/rosout` and `/parameter_events`
publishers before user code runs, so any contract that did not list both
under `external_topics:` tripped strict at node start.

## Cause

`emit()` and `emit_repeatable()` set `strict_violated` before looking at
`severity`; the argument only reached the log line and the JSONL record.
The engine's own vocabulary (`Warning < Error`) was not honoured by the one
consumer that acts on it.

## Fix

Phase 79 W2, in full:

- `RuleEngine::trip_strict(rule_id, fqn, severity)` flips the flag only at
  or above a threshold, default `Severity::Error`; both emitters call it.
  Warnings are still logged, written to `runtime_violations.jsonl` and
  counted. `--strict-on warning|error` beside `--enforce-rules` sets the
  threshold for users who want warnings fatal.
- The first violation that crossed the threshold is remembered as
  `"<rule_id> on <fqn>"`; the strict watcher logs it and the run's exit
  message names it.
- `RuleEngine::new` seeds `/rosout`, `/rosout_agg` and `/parameter_events`
  into the runtime topic map as implicitly external
  (`RCL_INTERNAL_TOPICS`), so `graph-deviation-runtime` reports only topics
  an author could plausibly have declared. `ContractView::externals` is
  untouched.

Pinned by `runtime_enforcement::tests::strict_mode_trips_on_error_severity_only`
(a graph-deviation warning leaves the flag alone, a rate error sets it and
is named), `strict_on_warning_makes_a_warning_fatal`,
`rcl_internal_topics_are_not_graph_deviations`, and by
`tests/tests/runtime_enforcement.rs::strict_mode_keeps_running_on_warning_severity_violations`
(a contract with no `topics:` block under strict: warnings recorded, none
for `/rosout` or `/parameter_events`, nodes and supervisor still running)
plus the `severity == "error"` assertion in
`strict_mode_ends_the_run_non_zero_and_stops_the_nodes`.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, section 2.5, observation 1), runs of
2026-09-18 with the 0.10.0 binary at `155ed78b`; captures under
`scratchpad/playlaunch/runtime/{warn,strict,strict2}/`. Re-verified against
the 0.11.0 source at `5eaa3191` on 2026-09-21.
