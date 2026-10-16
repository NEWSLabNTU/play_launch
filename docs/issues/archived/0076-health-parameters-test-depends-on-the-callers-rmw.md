---
id: 76
title: "The health parameters test read the caller's RMW and panicked on a web UI not yet listening"
status: resolved
type: defect
severity: low
---

# 0076 - The health parameters test depended on the caller's RMW and start-up time

**Repo:** `play_launch` at `622613df`
**Affects:** `tests/tests/interception.rs` only
(`test_health_compares_running_parameters_with_the_launch`, and the `http`
helper every web-UI test in that file uses). The product is not affected.
**Found:** 2026-10-17, before 0.18.3, by the carla-scenario-bridge session:
the test failed run alone, with `"limit": null` where 120 was expected.
**Not a regression:** v0.18.2 (`23e6e9bf`, the parent of #0075's fix) fails
the same way; the published 0.18.2 binary, run by the 0.18.2 test sources,
gives the identical `Null` vs `120`.

## What happened

Two independent faults, both in the test, met in this one test.

### 1. The test inherited the developer's RMW

The test's last assertion is that the discovery view's `limit` is 120,
"against Fast DDS's 120 a domain". That limit is only a Fast DDS fact:
`health/discovery.rs::view` sets it for `rmw_fastrtps*`/`rmw_fastdds*`, sets
`MaxAutoParticipantIndex + 1` for Cyclone with `ParticipantIndex auto`, and
`None` for anything else -- including Cyclone with no `ParticipantIndex`,
whose default (`none`) has no participant limit at all. `None` is the right
answer there.

`fixtures::install_env` sources ROS from a `bash -c` that inherits the
caller's environment, so a shell with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(any workstation that works on a Cyclone-based stack, as this one does) runs
the launched system on Cyclone, and the assertion reads `null`. On CI and on
a shell with no `RMW_IMPLEMENTATION` the default is Fast DDS and it passed,
which is why it went unnoticed.

The sibling test `test_health_reports_a_dds_config_the_rmw_does_not_read`
already pins the default RMW with `cmd.env_remove("RMW_IMPLEMENTATION")` for
the same reason; this one did not.

### 2. `http()` panicked on a refused connection

`http` did `TcpStream::connect(..).expect("connect to the web UI")`. Every
caller is a polling loop that sleeps 0.5-1 s and retries on a non-200, but a
refused connection never reached the loop: it panicked. play_launch binds the
web port only after parsing and resolving the launch, about a second here
(and more on a loaded machine), so the first poll at 0.5 s was refused and
the test died with `Connection refused` before it got to the assertion above.
This is the "connection refused" flake the integration suite showed under
parallel load; on this host (load ~13) it reproduced run alone, every time.
`dds_assist.rs`'s copy of the helper already returned status 0 instead.

## Fix

- `http` returns `(0, "")` when the connection is refused, like any other
  non-200, so the polling loops retry until their own deadline.
- The test removes `RMW_IMPLEMENTATION` from the launched process, so the
  launch runs on the distribution's default RMW (Fast DDS), which is what its
  `limit == 120` assertion describes.

The assertion itself is right and is kept: for Fast DDS the limit is 120.

## Verification

| tree | RMW | before | after |
|---|---|---|---|
| `622613df` | caller's Cyclone | panic: connection refused | pass |
| `622613df` | default | panic: connection refused | pass |
| `622613df` + `http` fix only | caller's Cyclone | `limit` Null vs 120 | -- |
| `23e6e9bf` (v0.18.2) + `http` fix only | caller's Cyclone | `limit` Null vs 120 | -- |
| `23e6e9bf` (v0.18.2) + `http` fix only | default | pass | -- |
