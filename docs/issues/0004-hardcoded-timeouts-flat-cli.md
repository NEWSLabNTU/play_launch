---
id: 4
title: "12+ hardcoded timeout Durations, none configurable; 59-flag flat CLI"
status: resolved
type: friction
severity: medium
github: https://github.com/NEWSLabNTU/play_launch/issues/4
---

`container_actor/mod.rs:39-70` hardcodes 12 Durations (incl.
`LOAD_TOTAL_BUDGET` 600s and the `POST_SERVICE_READY_WARMUP` 200ms
sleep-guess, `service_calls.rs:147-157`); more in `run.rs`,
`signal_handler.rs`, `web/sse.rs`, `rt_helper_client.rs`. No CLI/config
exposure. `cli/options.rs` is 903 LOC / 59 flags with no
`#[command(flatten)]` grouping.

Fix: `timing.rs` consolidation wired from grouped config — plan
`docs/roadmap/phase-52-config-and-failure-surfacing.md`.

## Resolution (phase-52, 2026-07-25)

`LoadTimings` (config `composable_node_loading` + `--load-total-budget` /
`--load-node-timeout`) drives the whole LoadNode path; defaults = the old
consts; effective values logged at startup. `CommonOptions` split into
flattened concern structs (flags unchanged). Non-load Durations
(SSE reconnect, signal cadence) intentionally left as consts.
