# Phase 39: RT Example Workspace

**Status:** ✅ Complete (39.1–39.4, 2026-07-15)
**Design of record:** [docs/superpowers/specs/2026-07-15-rt-workspace-fixture-design.md](../superpowers/specs/2026-07-15-rt-workspace-fixture-design.md)
**Builds on:** Phase 38 (RT scheduling apply-layer, complete).

## Overview

A small real colcon workspace at `tests/fixtures/rt_workspace/` that is both
the runnable example behind `docs/guide/rt-scheduling.md` and an integration
fixture for the RT apply-layer against our own nodes. Survey result
(2026-07-15): nothing like it exists — existing fixtures are launch-file-only
(nodes borrowed from `/opt/ros`), `io_stress` is special-purpose, `autoware`
is external/gated, and no fixture commits a `system.toml`.

## Work items

- [x] **39.1** `rt_demo` package — `rclcpp`+`std_msgs` ament_cmake package:
  `sensor_node` (100 Hz timer pub), `control_node` (sub→pub; the FIFO-20
  node), `filter_component` (`rclcpp_components` composable).
- [x] **39.2** Fixture wiring — `bringup.launch.xml` (2 standalone + 1 container +
  composable), `bringup.contract.yaml` provider sidecar + a `contracts/`
  user-overlay example (Phase 40 shipping), `system.toml` (control /
  perception tiers + `[[assign]]`), per-fixture `justfile`
  (build/dump/compare/run), README.
- [x] **39.3** Integration test `tests/tests/rt_workspace.rs` — build gate, dump
  parity, `check` on the committed manifest+sched files, sched-apply smoke
  (standalone + composable), privileged per-TID assertion (auto-skipped when
  uncapped). Excluded from `just test`, included in `just test-all` (the
  `io_stress` pattern).
- [x] **39.4** Docs — point the RT guide's quick-start at the fixture; add the
  fixture to CLAUDE.md's test-workspace list.

## Order and dependencies

39.1 → 39.2 → 39.3 → 39.4. Phase 38 machinery is consumed as-is. 39.2 depends
on Phase 40.1–40.2 (contract sidecar/overlay resolution) — land those first so
the fixture ships only the new layout.

## Out of scope

Custom messages, lifecycle nodes, multi-container topologies, nano-ros/RTOS
wiring (the committed `system.toml` is already portable; the RTOS side lives
in the nano-ros repo).
