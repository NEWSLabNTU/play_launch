---
id: 14
title: "test_isolated_external_subscriber hard-codes ROS_DOMAIN_ID=199 and hides the real error behind 2>/dev/null"
status: resolved
type: test-infrastructure
severity: medium
---

# 0014 — A stale `ros2-daemon` makes an unrelated test claim DDS is broken

**Repo:** `play_launch`
**Affects:** `tests/tests/container_events.rs` (`test_isolated_external_subscriber`)
**Found:** phase-55 W1 verification, 2026-08-02

## Summary

The test fails whenever a `ros2-daemon` for domain 199 is left running by an
earlier run, and the failure names the wrong cause:

```
External subscriber did not receive data from isolated container.
This indicates DDS cross-process communication is broken for fork+exec isolated children.
```

DDS was fine. The `ros2` CLI was talking to a stale daemon whose cached graph
no longer matched reality, so `ros2 topic echo` exited 1 immediately and
`ros2 topic list` returned nothing.

## Two separate defects

**1. Hard-coded domain.** The test pins `ROS_DOMAIN_ID=199`:

```rust
// Use a known domain ID so the external subscriber joins the same DDS domain
let domain_id = "199";
```

The comment explains why a *known* domain is needed — the external `ros2
topic echo` has to join the same one — but not why it must be a *constant*.
Every other test gets a unique domain per invocation from
`fixtures::play_launch_cmd()` (PID + counter) precisely so concurrent nextest
processes cannot cross-talk. This test opts out of that and so inherits
whatever state domain 199 was left in, by this test or any other run on the
machine.

**2. The diagnostics hide the error.** Both fallback commands discard stderr:

```rust
echo_cmd.arg("timeout 15 ros2 topic echo /chatter std_msgs/msg/String --once 2>/dev/null");
...
list_cmd.arg("ros2 topic list 2>/dev/null");
```

So the failure block dutifully prints an empty stdout, an empty stderr and an
empty topic list, and the assertion asserts a cause it never checked. The
`2>/dev/null` is there to suppress `ros2`'s routine warning chatter on the
success path; on the failure path it suppresses the only useful information.

## Repro

```sh
# leave a daemon on the test's domain
ROS_DOMAIN_ID=199 ros2 daemon start
cargo nextest run -E 'test(test_isolated_external_subscriber)'   # FAIL
ROS_DOMAIN_ID=199 ros2 daemon stop
cargo nextest run -E 'test(test_isolated_external_subscriber)'   # PASS
```

Confirmed 2026-08-02: three consecutive failures, then a pass immediately
after `ros2 daemon stop`, with no other change.

## Cost

Several verification cycles during phase-55 W1, because a repo-layout change
was in flight and the message pointed at process isolation — plausible enough
to look causal. A test that names a wrong cause confidently is worse than one
that fails with no explanation.

## Suggested fix

- Derive the domain the same way the rest of the suite does (unique per
  invocation) and pass it to the external subscriber, rather than pinning a
  constant. `--no-daemon` on the `ros2` calls, or stopping the daemon for the
  chosen domain first, closes the remaining window.
- Capture stderr instead of discarding it, and print it only in the failure
  block, so the success path stays quiet and the failure path is diagnosable.
- Soften the assertion message: report what was observed (`ros2` exit status,
  captured stderr, topic list) and let the reader conclude, rather than
  asserting "DDS cross-process communication is broken".

Neither change touches product code.

## Resolution (2026-08-03) — fixed

- `fixtures::next_domain_id()` is now `pub`; the test calls the same allocator
  every other test uses and hands the result to `ros2`. No constant.
- Both `ros2` invocations gained `--no-daemon`, so the CLI cannot consult or
  spawn a daemon at all. With a per-invocation domain a stale daemon is
  already unlikely; `--no-daemon` removes the vector rather than relying on
  the domain being fresh.
- stderr is captured and reported in a single `diagnosis` block (domain, exit
  status, both streams of both commands) instead of being sent to
  `/dev/null`. The success path stays quiet; the failure path is diagnosable.
- The assertion reports what was observed and tells the reader how to read it
  — an empty `topic list` or non-zero exit means the CLI failed and the result
  says nothing about DDS. It no longer asserts "DDS cross-process
  communication is broken", which was wrong every time this fired.

**Verified with a before/after, not by re-running until green.** Daemons
planted on domains 199, 44 and 88 — the exact state that failed the old test
three times in a row — then the fixed test run three times: 3/3 PASS.
