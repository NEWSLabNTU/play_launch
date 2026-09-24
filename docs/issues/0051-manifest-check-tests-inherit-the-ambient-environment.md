---
id: 51
title: "`manifest_check.rs` spawns the binary with a bare `Command::new`, so the whole suite dies unless the shell happens to have sourced `install/setup.bash`"
status: resolved
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

## Resolution 2026-09-25

Measured from a genuinely bare environment
(`env -i PATH=/usr/bin:/bin HOME=$HOME <test binary>`), which is the failing
condition the report names:

| | passed | failed |
|---|---|---|
| before | 13 | 17 |
| after  | 23 | 7 |

Zero `error while loading shared libraries` lines remain (16 before).

**The report's line numbers were only half of it.** `:53`, `:107` and `:130`
spawn the standalone `ros-launch-resolve` binary, which links no ROS
libraries and was never the thing that failed. The 16 library-path failures
came from five OTHER bare spawns — `Command::new(fixtures::play_launch_bin())`
at `:788`, `:844`, `:899`, `:935` and `:1180`, two of them the
`check_fixture`/`check_with_sched` helpers most of the phase-67..82 tests run
through. All 18 sites are routed now: the five through
`fixtures::play_launch_cmd(test_env())`, the thirteen resolve ones through a
local `resolve_cmd()`.

`fixtures.rs` did need the factoring the report allowed for, for one reason:
there is no `fixtures` helper that targets layer 2's standalone binary
(`fixtures::ros_launch_resolve_bin()` returns `play_launch_bin()`), so
`manifest_check` must keep its own path lookup — the one that skips cleanly
when the binary is unbuilt. `apply_test_env(&mut Command, &env)` is split out
of `play_launch_cmd`, which now calls it, so the environment (clear, sourced
ROS + colcon install, `PYTHONPATH`, unique `ROS_DOMAIN_ID`, FastDDS profile)
is shared while the binary and arguments stay the caller's. `play_launch_cmd`
is behaviour-identical. The env is sourced once per process through a
`OnceLock`, since sourcing shells out to `bash`.

### Two things found on the way, neither caused by this work

1. **`tests/tests/manifest_check.rs` did not compile at `1c27ba5c`** — the
   phase-82 test `a_server_that_latches_the_request_is_a_sampling_hop` is
   missing its closing brace, and the error points at the next `}` 145 lines
   later. So the whole `play-launch-tests` crate's `manifest_check` binary was
   unbuildable on `main`, which is why both the before and after measurements
   here had to add that one brace first. It is added.

2. **Three of the seven remaining failures are a stale `install/` binary**, not
   a product defect: `a_hazard_is_timed_by_the_slowest_fault_class_it_claims`,
   `a_reaction_crossing_a_service_is_walked_like_one_crossing_a_topic` and
   `a_server_that_latches_the_request_is_a_sampling_hop` drive
   `install/play_launch/lib/play_launch/play_launch`, which predates phase 82
   and rejects its own fixture with
   `at 'hazards.scan_classes.on': expected a string, got a list`. A `just
   build` fixes them. Same family as #0020 — the artifact looks right and
   behaves like last month.

The other four (`check_legacy_toml_with_contradicting_contract_facts_warns_but_succeeds`
and the three `w2_*`) fail for separate, pre-existing reasons and are
untouched here.
