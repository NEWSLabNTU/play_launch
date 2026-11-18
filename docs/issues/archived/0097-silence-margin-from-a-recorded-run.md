---
id: 97
title: "check --against does not report how close a run came to a subscriber's silence bound"
status: resolved
type: feature
severity: medium
---

# 0097 - how close a run came to each silence bound

**Repo:** `play_launch` 0.39.0
**Reported:** 2026-11-13, GitHub #25 (L4 plan item PL-11, review F6, exit
criterion FC11)

## What is missing

The L4 safety island trips when the HPC's heartbeat is silent for 30 ms (a
100 Hz publisher). Before relying on that, we need evidence that a loaded
system does not trip it falsely. A contract states the bound (a
subscriber's QoS `deadline` or `lease_duration`, or its `max_age`), and a
run recorded with `--record-events` holds every take. Nothing compares the
two: `check --against` reads `events.jsonl` only for link transports, and
`measure` only for path costs.

## Wanted

From a recorded run, for each contracted subscriber that states a silence
bound:

- its largest inter-arrival gap;
- its 99.9th percentile gap;
- each bound, and the margin between it and the worst the run showed.

`check --against` warns when a margin is under a stated threshold.

## Resolution

A `check --against` step beside the link check
(`ros/silence_margins.rs`):

- Gaps are measured between consecutive takes by the subscriber's process
  on the topic (a take of a stamp already taken is the same message, from
  a second subscription).
- `deadline` and `lease_duration` are judged on the gap. `max_age` is
  judged on the age a message reached before the next one arrived: the
  next take minus that message's publish, when the run recorded the
  publish, else the gap (a lower bound, and the report says so).
- Diagnostics: `silence-exceeded` (a gap or age past its bound) and
  `silence-margin` (less than `--silence-margin` percent left, default 20),
  both warnings, which `--deny` can make errors; `silence-unmeasured` when
  the run has no take by the subscriber.
- Recording stopped and restarted leaves no gap: a segment's first message
  re-emits its topic's name, and a repeated name restarts that topic's
  gaps.

Found on the way: `--deny` ran straight after the contracts loaded, before
the platform plan, recovery and `--against` added their findings, so none
of those could be denied (`--deny transport-exceeded` was as inert as
`--deny silence-margin`). It runs after the last finding now, and still
before `acknowledged:`.

Tests: the unit tests in `ros/silence_margins.rs`, and
`a_run_close_to_a_silence_bound_warns_silence_margin` in
`tests/tests/manifest_check.rs` on `contract_takeover_named_link/run_quiet`
(a 450 ms gap against a 500 ms lease). Its `--deny` case failed before the
ordering fix.
