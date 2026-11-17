---
id: 94
title: "Three integration tests fail only when the whole suite runs, and pass alone"
status: resolved
type: friction
severity: low
---

# 0094 - three integration tests fail only under suite load

**Repo:** `play_launch` main at `ad7849e4`
**Affects:** `tests/tests/control_channel.rs`, `tests/tests/manifest_check.rs`,
`tests/tests/interception.rs`
**Found:** 2026-11-17, four `just test-all` runs on two trees, while a second
worktree was building and testing on the same host

## What happens

Each of these failed in one `just test-all` run and passed in the next on
the same tree, and each passes alone:

| test | in the suite | alone |
|---|---|---|
| `control_channel::test_slow_constructor_reports_liveness_not_a_timeout` | failed twice (retry included): "the load result did not arrive over the control channel" | 23 s, ok |
| `manifest_check::a_settle_response_check_outside_the_profile_is_refused` | timed out at 60 s | 24.5 s, all CPU (four `check` runs proving worst cases) |
| `interception::test_triggered_edges_form_a_chain_and_an_old_stamp_is_reported` | 92 s, `chains.json` empty | 10 s, ok |

## Why it matters

A gate that fails one run in two on load teaches people to rerun until green,
which is how a real failure gets through.

## Expected

Each test asserts what it means, not a race:
- the control-channel test accepts the load result from whichever channel
  wins, or forces the socket path, since both report channels are live by
  design (CLAUDE.md, phase 64);
- the settle test gets a nextest override matching its CPU cost, or does
  less proving per case;
- the chains test waits for the edges it needs instead of a fixed run time.

## Progress (2026-11-17)

- **control_channel: fixed.** The failing run's log shows the race: the load
  was sent over the socket and the container's `ComponentEvent LOADED`
  arrived 85 ms before the socket's own report. Both channels are live by
  design and the first wins, so the test now asserts what is deterministic:
  the load was sent over the control channel, and a load result arrived
  by either channel.
- **manifest_check: fixed.** The binary gets a 60 s x 3 nextest bound
  (`tests/.config/nextest.toml`); its settle tests are CPU-bound Z3 proving.
- **interception: not reproduced; instrumented.** This is not slowness: the test polls up to
  90 s, and `chains.json` had no edges at all, so nothing was intercepted or
  nothing ran. The test used to send play_launch's output to `/dev/null`;
  it now keeps it and prints its tail on failure, so the next occurrence
  names the cause.
- **shutdown_semantics::ros2_launch_respawns_on_another_code: fixed**
  (found later the same day, merging #0095). The STOCK side failed: a
  Ctrl-C landed while `ros2 launch` was respawning the stub, so launch
  walked its full ladder (SIGINT, 5 s, SIGTERM, 10 s, SIGKILL), which the
  test's 20 s bound did not cover under load. The bound is 40 s; the test
  is about the respawns before the Ctrl-C.

## Resolution (2026-11-18)

The interception chains test could not be made to fail again: 16 runs, 8 at
a time, and 8 more at a time beside 48 busy loops on a 48-core host (load
average 44), all passed in 10-14 s against the test's 90 s bound. The one
failure on 2026-11-17 happened while a second worktree was building and
testing on the same host, and its output went to `/dev/null`, so the cause
is not recoverable. The test now keeps play_launch's stdout and stderr and
prints their tail on failure; a recurrence is a new issue, with that log.
The other three tests are fixed (see Progress).
