---
id: 94
title: "Three integration tests fail only when the whole suite runs, and pass alone"
status: open
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
- **interception: still open.** This is not slowness: the test polls up to
  90 s, and `chains.json` had no edges at all, so nothing was intercepted or
  nothing ran. The test used to send play_launch's output to `/dev/null`;
  it now keeps it and prints its tail on failure, so the next occurrence
  names the cause.
