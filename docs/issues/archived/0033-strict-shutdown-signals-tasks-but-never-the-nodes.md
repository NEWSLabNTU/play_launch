---
id: 33
title: "strict-mode \"initiating shutdown\" flips the watch channel and nothing else; the supervisor sits in `child.wait()` forever"
status: resolved
resolved_in: fix(#0033) on main, 2026-09-21
type: correctness
severity: high
---

# 0033 - the strict watcher did a third of a shutdown

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affected:** `src/play_launch/src/commands/up.rs` (strict watcher),
`src/play_launch/src/member_actor/regular_node_actor.rs`,
`src/play_launch/src/member_actor/container_actor/process_lifecycle.rs`,
`src/play_launch/src/commands/signal_handler.rs`

## Symptom

Under `--enforce-rules strict` the last line play_launch printed was
`[runtime] Strict enforcement violated -- initiating shutdown`; the talker's
output stopped, but the `play_launch` process was still alive 45 s later and
was killed by the `timeout` wrapper (exit 124). Reproduced twice on
2026-09-18 with the 0.10.0 binary at `155ed78b`; the cited lines were
unchanged at 0.11.0 (`5eaa3191`).

## Cause

The shutdown protocol has three levers and the strict watcher pulled one.
The signal path did `kill_process_group(pgid, SIGTERM)`,
`shutdown_tx.send(true)` and `member_handle.shutdown()`; the strict watcher
did `shutdown_tx.send(true)` only. `RegularNodeActor` (and the container
actor) reacted to the watch channel by assuming the group SIGTERM had been
sent ("killed by PGID") and blocked in `child.wait()` on a healthy child, so
the runner task never completed and the main loop never broke. The output
forwarders, which do listen to the channel, went quiet, which is why the
capture looked like the nodes had stopped.

## Fix

- `signal_handler::initiate_shutdown(pgid, shutdown_tx, member_handle)` is
  the one teardown; the first-signal branch, `--on-startup-failure exit`,
  the `on_exit=Shutdown()` hook and the strict watcher all call it, so the
  levers cannot drift apart again.
- The strict watcher records the violation in a run-level flag and `play()`
  returns `Err("runtime contract violated (--enforce-rules strict)")`, so
  the process exits non-zero the way `--on-startup-failure exit` does. The
  watcher also ends when the run shuts down for another reason.
- Neither actor assumes its child was signalled by someone else any more:
  the Running-state shutdown branch waits 2 s for the child, then runs the
  ladder the Stop/Restart control events already use (SIGTERM, five
  seconds, SIGKILL) on it directly.

Pinned by `tests/tests/runtime_enforcement.rs::
strict_mode_ends_the_run_non_zero_and_stops_the_nodes`: a 1 Hz talker
against `min_rate_hz: 1000` under strict must exit non-zero within 40 s,
name the violation on stderr, and leave none of the recorded node pids
alive.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, section 2.5, observation 2), runs
of 2026-09-18; captures under `scratchpad/playlaunch/runtime/strict/` and
`strict2/`.
