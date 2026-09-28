---
id: 58
title: "the hazard observer records a reaction only on a host publish, so an island's reaction is invisible"
status: resolved
type: correctness
severity: medium
---

# 0058 - the reaction happened on an MCU, and the host saw only the take

**Repo:** `play_launch` at `22b6baf3`
**Affects:** `src/play_launch/src/runtime_enforcement/mod.rs`
(`observe_hazards`, `reaction_of`), `src/play_launch/src/interception/measure.rs`
(`observe_hazards`), `src/play_launch/src/commands/measure.rs`

## Symptom

Both hazard observers -- the live one and `measure` after the fact -- take
the reaction to be the first provenance-free PUBLISH on a sink after the
fault. The Autoware Safety Island publishes `/system/emergency/control_cmd`
from an S32K344; the host's interception layer never sees that publish, only
`vehicle_cmd_gate`'s TAKE of it. So a reaction the board measured at
555-589 ms from the last availability sample would be reported as "NOTHING
reacted" by `measure`, and not at all by the live observer.

## Fix (phase 83)

A sink no instrumented process publishes -- declared `external: pub` in the
contract (`HazardWatch.off_host_sinks`), or simply never seen published by a
host process during the run -- is judged by its host takes, with the same
provenance test (a take carrying the guard's last stamp is the tail of
normal operation, not the reaction). The reaction is labelled where it was
seen: `"observed_at":"take"` in `runtime_violations.jsonl`, and in both
messages "the link hop is inside the number", so the link's latency is never
silently credited to the island. A sink a host process does publish is never
judged by its takes.

Tests: `runtime_enforcement::tests::an_off_host_sink_is_observed_at_its_first_host_take`,
`a_host_published_sink_is_not_judged_by_its_takes`, and
`interception::measure::tests::an_off_host_sink_is_observed_at_its_first_host_take`.
