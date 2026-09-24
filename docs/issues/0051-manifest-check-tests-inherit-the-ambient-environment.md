---
id: 51
title: "`manifest_check.rs` spawns the binary with a bare `Command::new`, so the whole suite dies unless the shell happens to have sourced `install/setup.bash`"
status: open
type: test-defect
severity: medium
---

# 0051 - 13 of 27 tests fail for the environment, not for the code

**Repo:** `play_launch` at `a6506261`
**Affects:** `tests/tests/manifest_check.rs:53`, `:107`, `:130`;
compare `play_launch_cmd()` in `tests/src/fixtures.rs`

## Symptom

```
$ cargo nextest run --test manifest_check
...
13 of 27 failed:
  /…/install/play_launch/lib/play_launch/play_launch: error while loading
  shared libraries: libplay_launch_msgs__rosidl_typesupport_c.so: cannot open
  shared object file: No such file or directory
```

Measured 2026-09-23. The same suite passes when the invoking shell has sourced
`install/setup.bash` first.

## Cause

The file builds its commands with a bare `Command::new(bin)` and inherits
whatever environment the runner happens to have, instead of going through
`play_launch_cmd()` in `tests/src/fixtures.rs` — the helper every other suite
uses, which sets the library path and also assigns a unique `ROS_DOMAIN_ID`
per invocation so concurrent nextest processes do not cross-talk over DDS.

## Impact

It cost real time twice in one session. The failures look like product
breakage and were investigated as such before being traced to the harness; and
because they are the DEFAULT state of the suite, they mask genuine failures —
a real regression in `manifest_check` is indistinguishable from the 13 that
always fail. The suite is also missing the DDS isolation the helper provides,
which is its own latent flake.

## Fix direction

Route all three call sites through `play_launch_cmd()`. If that helper does
something these tests do not want (it may set flags a `check` invocation does
not need), the fix is to factor the environment half out of it rather than to
keep a second, weaker spawn path — the environment is the part that must not
differ between suites.

Verify by running the suite from a shell that has NOT sourced anything.

## Provenance

Hit 2026-09-23 while validating a manifest pin bump, and again 2026-09-24 by a
second agent, which reported it as "unrelated to this work, but it masks real
results". Filed now.
