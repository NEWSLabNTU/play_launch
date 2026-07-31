# 0008 — Composable-node unload returns HTTP 500; 4 `parallel_loading` tests fail

**Status:** Open
**Filed:** 2026-08-01
**Severity:** Real defect, pre-existing, deterministic
**Affects:** `POST /api/nodes/:name/unload`, container actor unload handling
**Failing tests:** `tests/tests/parallel_loading.rs` —
`test_unload_via_web_api`, `test_unload_and_reload`,
`test_unload_during_construction`, `test_fast_not_blocked_by_slow`

## Summary

Unloading a composable node through the web API returns HTTP 500 instead of
200/202. Four `parallel_loading` integration tests fail on it. A companion
symptom in the same suite is `Actor not found: slow_node`.

This is **not** a flake and **not** environmental. It was bisected during
the `machine=` removal work (2026-07-31) and reproduces deterministically.

## Evidence

Three runs, same four tests, same failure signatures, near-identical
timings each time (9.3s / 23.7s / 36.7s):

| # | Conditions | Result |
|---|---|---|
| 1 | Loaded machine: load avg 34.40, 102 ROS processes | 3 passed / 4 failed |
| 2 | Idle machine: load avg 13.6, 1 ROS process | 3 passed / 4 failed |
| 3 | **Pre-branch binary** (`play_launch 0.8.2`, built 2026-07-28) | 3 passed / 4 failed |

Run 2 rules out resource contention — a 500 returned 9.3 seconds in, on an
otherwise idle machine, reproducing to the second, is not a timing race.

Run 3 rules out the `machine=` removal branch as the cause. The installed
wheel binary from three days earlier was swapped into
`install/play_launch/lib/play_launch/` (the path `tests/src/fixtures.rs`
resolves) and failed identically. The branch binary was restored afterwards
and verified byte-identical.

Corroborating: `git diff --name-only <branch-point>..HEAD` on that branch
touched only `src/play_launch/src/commands/{resolve_compat.rs,run.rs}` and
`src/play_launch/src/execution/node_cmdline.rs` — nothing in
`member_actor`, `container_actor`, or the web unload handlers those tests
exercise.

## Reproduction

```sh
cd tests
cargo nextest run -E 'binary(parallel_loading)' --no-fail-fast
```

Expected today: 7 tests run, 3 passed, 4 failed. Representative output:

```
Unloading fast_talker via POST /api/nodes/fast_talker/unload
Unload response status: 500
thread 'test_unload_via_web_api' panicked at tests/parallel_loading.rs:388:5:
Expected 200/202 from unload, got 500
```

Both nextest retries fail, so it is not retry-flaky.

## Where to start

The 500 comes from the unload endpoint rather than from the container
itself refusing, and `Actor not found: slow_node` suggests the actor
registry lookup fails at unload time — plausibly a name-keying mismatch
between how members are registered and how the unload handler looks them
up. Note the 2026-03-12 change making nodes with `name=None` use
`exec_name` as the FQN map key (`node_cmdline.rs`, `builder.rs`); the
`parallel_loading` fixtures are a good place to check whether registration
and lookup agree.

Not yet investigated beyond the bisect — the bisect established *what* is
and is not responsible, not the root cause.

## Why this was not fixed in the branch that found it

The `machine=` removal branch (spec:
`docs/superpowers/specs/2026-07-31-machine-attr-removal-design.md`) did not
cause this and does not touch the unload path. It is filed here so the
bisect is not repeated from scratch.
