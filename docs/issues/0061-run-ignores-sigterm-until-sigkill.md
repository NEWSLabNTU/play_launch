---
id: 61
title: "`play_launch run` never exits on SIGTERM or SIGINT: it signals the process group and the task channel, but never the actors"
status: resolved
type: defect
severity: high
---

# 0061 - `play_launch run` ignores SIGTERM until SIGKILL

**Repo:** `play_launch` at `615cf3c7`
**Affects:** `src/play_launch/src/commands/run.rs` (the main signal loop and
the drain after it)
**Found:** noted as "still open" in the #0024 entry of `CLAUDE.md`; filed and
reproduced 2026-10-09.

## What happens

`play_launch run <pkg> <exec>` handles the first SIGTERM (or Ctrl-C), stops
the node, and then never exits. It logs the shutdown and goes quiet:

```
INFO  Shutting down gracefully (SIGTERM)...
INFO  Press Ctrl-C again to force terminate
WARN  Background task finished early (clean exit)
DEBUG Draining remaining background tasks...
DEBUG [node:/talker] Unregistered PID 2705254
DEBUG State event: Exited { name: "node:/talker", exit_code: Some(0) }
DEBUG State event: Terminated { name: "node:/talker" }
```

Measured with `run --disable-all demo_nodes_cpp talker`: still running 10 s
after `kill -TERM <pid>`, and 15 s in the regression test. A second Ctrl-C
only re-signals the group; a third is `exit(1)`. Under systemd that means
waiting out `TimeoutStopSec` and taking a SIGKILL, every time.

## Why

Shutting a run down takes three levers, each reaching a different party:
SIGTERM to the process group (the node), the run-level watch channel (the
background tasks), and `member_handle.shutdown()` (the actors, which listen
on a separate channel the builder created).
`signal_handler::initiate_shutdown` exists to pull all three together,
because issue #0033 was a caller pulling only the middle one.

`run` has its own copy of the signal loop and was never moved onto it. It
pulled the first two levers and never the third. So the node exited, and its
actor moved to `Stopped`. `Stopped` deliberately keeps the actor alive so the
web UI can restart the node, and it ends only when the actors' own shutdown
channel fires. Nothing sent on that channel, so the runner (which waits for
every actor) never completed. The loop had already broken on the anchor task
finishing, and the drain after it waited on the runner forever.

The drain had the same hole on its own. The loop also breaks when a
background task finishes by itself (the web server failing, say), and the
code after the loop sent on the task channel only, so that path would have
hung identically with no signal involved.

## Why the tests did not see it

Every `run` integration test stops `run` through `ManagedProcess`. Its
cleanup sends SIGTERM to the group, waits two seconds and sends SIGKILL, so a
`run` that hangs and one that exits cleanly look the same from the test.
`run_interception` even documents SIGTERM as "what makes `run` signal its own
shutdown", which was only true because the summaries are written by a task
that does listen on the task channel.

The SIGKILL also leaked processes. A node lives in `run`'s anchor process
group, not the test's, so killing `run` left the node running. Three
`play_launch run --interception on … talker` processes from earlier suite
runs were still alive on the development machine, with their talkers still
publishing.

## Fix

- The first signal calls `signal_handler::initiate_shutdown(pgid,
  &shutdown_tx, &member_handle)`, the same function `up` uses, instead of
  pulling two of its three levers by hand.
- After the loop, `member_handle.shutdown()` is sent alongside the task
  channel, so the drain cannot hang however the loop ended.
- `Background task finished early (clean exit)` is a `debug!` when the
  shutdown was requested. It used to print a WARN on every Ctrl-C for the
  anchor task doing what it was asked.

## Verification

`tests/tests/run_signal.rs` sends SIGTERM, and separately SIGINT, to `run`
**alone**. That is `kill <pid>`, the case where `run` has to bring its own
children down. Each test watches for the exit itself via a new
non-escalating `ManagedProcess::try_wait`, and asserts that the node's PID is
gone afterwards.

- Negative control (fix reverted, rebuilt): both tests fail with `still
  running 15s after SIGTERM` and the log tail above.
- With the fix: both exit in about 300 ms with status 0, and the talker is
  gone.
- All `run_*` suites pass (21 tests). `just test`: 176 passed, 19 skipped.
  `just check` is green.

Related: #0033 (same three-lever omission, in the strict-enforcement
shutdown) and #0024 (the `run` web server on a throwaway channel; that fix is
what left this one visible).
