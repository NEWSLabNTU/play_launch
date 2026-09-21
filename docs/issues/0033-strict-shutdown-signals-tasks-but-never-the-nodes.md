---
id: 33
title: "strict-mode \"initiating shutdown\" flips the watch channel and nothing else; the supervisor sits in `child.wait()` forever"
status: open
type: correctness
severity: high
---

# 0033 - the strict watcher does a third of a shutdown

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affects:** `src/play_launch/src/commands/up.rs:1589-1602` (strict watcher);
`src/play_launch/src/member_actor/regular_node_actor.rs:367-380`;
`src/play_launch/src/commands/signal_handler.rs:96-134`, `:136-152`, `:182-184`

## Symptom

Continuing #0032's strict run (2026-09-18, 0.10.0 binary at `155ed78b`,
run under `timeout -s INT 14` and again under `timeout -s INT 45`):

```
2026-09-18T12:56:25.550878Z  WARN play_launch::commands::up: [runtime] Strict enforcement violated -- initiating shutdown
```

is the last line play_launch prints. The talker's 1 Hz "Publishing" lines,
present in the warn-mode capture, never appear after that point. The
`play_launch` process is still alive when the wrapper's SIGINT arrives 14 s
(and, in the second run, 45 s) later; the wrapper reports exit 124.
Reproduced twice; `scratchpad/playlaunch/runtime/strict/` and `strict2/`
hold `stdout.log`, `stderr.log` and the `play_log/` tree of each run.

## Cause

Two halves of the shutdown protocol live in different places and the
strict path calls only one of them.

The signal path (`signal_handler.rs:109-118`) does three things on the
first SIGINT/SIGTERM:

```rust
kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
let _ = ctx.shutdown_tx.send(true);
let _ = ctx.member_handle.shutdown();
```

The strict watcher (`up.rs:1589-1602`) does the middle one only:

```rust
warn!("[runtime] Strict enforcement violated -- initiating shutdown");
let _ = shutdown_tx_strict.send(true);
```

`RegularNodeActor` in `Running` state reacts to the watch channel at
`regular_node_actor.rs:367-380` by ASSUMING the first half happened:

```rust
// On Unix, kill_process_group() already sent SIGTERM to all processes
// We just need to wait for this child to exit
match child.wait().await {
```

Nobody sent SIGTERM, so `child.wait()` blocks on a healthy child; the actor
never finishes; the runner task never completes; the main loop at
`signal_handler.rs:136-152` (which breaks only on runner completion or a
real signal) spins on. `member_handle.shutdown()` is not called either.
The nodes keep running (the "no talker output" in the capture is the
output forwarders, which do listen to the watch channel, going quiet; the
process itself was never signalled).

## Impact

`--enforce-rules strict` is documented as "First violation triggers
shutdown and non-zero exit (CI mode)" (`cli/options.rs:806`). What happens
instead is a hung supervisor with its nodes still up and their output no
longer forwarded, until an outside SIGINT. In CI that is a job timeout
with a misleading transcript; on a vehicle it is worse, because the
operator's view goes dark while the system keeps driving.

## Fix direction

- Give the strict watcher the same three-step teardown the signal path
  uses: `kill_process_group(pgid, SIGTERM)`, `shutdown_tx.send(true)`,
  `member_handle.shutdown()`; better, factor those three lines into one
  `initiate_shutdown(reason)` that both callers use so they cannot drift
  again.
- Do not let an actor assume its child was signalled by someone else: in
  the `shutdown_rx` branch, if the child is still alive after a bounded
  wait, send SIGTERM itself (then SIGKILL after the grace period).
- Carry the reason to the exit code: strict violation must end the run
  non-zero, the way `--on-startup-failure exit` already does at
  `up.rs:1679-1683`.
- Test: an integration test that plants a violated contract under strict
  mode and asserts the process exits non-zero within a few seconds. Today
  the only strict test asserts the flag, not the exit.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, section 2.5, observation 2), runs of
2026-09-18 with the 0.10.0 binary at `155ed78b`; re-verified against the
0.11.0 source at `5eaa3191` on 2026-09-21 (the cited lines are unchanged
between the two).
