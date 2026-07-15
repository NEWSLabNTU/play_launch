# RT Example Workspace — Design

**Date:** 2026-07-15
**Status:** Approved (design), pending implementation
**Repo:** `play_launch`

## Goal

A small, real, buildable ROS 2 workspace that serves two roles at once:

1. **Example** — the thing `docs/guide/rt-scheduling.md` (and the slide deck)
   can point at: clone, build, `--sched`, observe `SCHED_FIFO` on every thread.
   Today the guide's example files are illustrative snippets with no runnable
   counterpart.
2. **Test fixture** — an integration fixture where the RT apply-layer is
   exercised against *our own* nodes rather than borrowed system packages, and
   where the committed `system.toml` + manifest are themselves under test.

## Current state (survey, 2026-07-15)

- All `tests/fixtures/*` except two are **launch-file-only** fixtures: XML +
  `record.json`, with nodes borrowed from `/opt/ros` system packages
  (`demo_nodes_cpp`, `composition`). No buildable source.
- `tests/fixtures/io_stress/` is the only real in-repo ament package —
  purpose-built for I/O stress, excluded from the fast suite.
- `tests/fixtures/autoware/` is a real workspace but external and gated.
- **No fixture carries a `system.toml`** — the sched integration tests generate
  theirs inline via tempfile.
- There is no `examples/` directory.

## Decisions

- **Location: `tests/fixtures/rt_workspace/`.** Matches the existing fixture
  conventions (per-fixture justfile, test plumbing, `test_workspace_path()`);
  the user-facing guide links to it rather than duplicating it under a new
  `examples/` tree. (Open to revisiting if a general `examples/` dir appears.)
- **Nodes: a new tiny `rt_demo` package** rather than reusing `io_stress` or
  system packages. Full control over node/composable shape lets the fixture
  mirror the guide's example exactly (a control/perception split), and the
  package doubles as reference code for "what does a node under RT look like".

## Layout

```
tests/fixtures/rt_workspace/
├── justfile                      # build / dump-rust / dump-python / run recipes
├── README.md
├── src/rt_demo/                  # ament_cmake, rclcpp + std_msgs only
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── src/
│       ├── sensor_node.cpp       # standalone: 100 Hz timer publisher
│       ├── control_node.cpp      # standalone: sub → pub (the "RT" node)
│       └── filter_component.cpp  # composable (rclcpp_components), sub → pub
├── launch/
│   ├── bringup.launch.xml        # 2 standalone nodes + 1 container w/ the composable
│   └── bringup.contract.yaml     # provider sidecar (Phase 40 shipping): types, rates, max_age_ms
├── contracts/                    # user-overlay example (Phase 40): overrides the sidecar
│   └── rt_demo/launch/bringup.contract.yaml
└── system.toml                   # tiers: control (FIFO 20, core 0), perception (FIFO 10)
                                  # [[assign]]: control_node → control; /perception scope → perception
```

Everything mirrors the guide's three-file story: launch (what runs), contract
(what must hold — shipped per Phase 40: provider sidecar + user overlay),
`system.toml` (how it is scheduled) — with the same names
used in `docs/guide/rt-scheduling.md` slide/examples so the docs become
literal.

## The package (`rt_demo`)

- `rclcpp` + `std_msgs` only — builds anywhere the repo builds.
- `sensor_node`: wall-timer publisher on `/perception/points_raw`
  (`std_msgs/msg/String` stand-in payload; the manifest documents the intended
  rate).
- `control_node`: subscribes `points_raw`, publishes `cmd`. This is the node
  the `control` tier pins to FIFO 20 / core 0.
- `filter_component`: an `rclcpp_components`-registered composable
  (sub→pub passthrough) loaded by the launch file's container — exercising the
  composable scheduling path (isolated mode → own process → per-TID apply on
  LOADED).
- No custom messages, no parameters beyond topic names — YAGNI.

## Build & test integration

- Fixture `justfile`: `build` (colcon build of `src/` into the fixture's own
  `install/`), `dump-rust` / `dump-python` / `compare-dumps` (matching other
  fixtures), `run` (launch with `--sched system.toml`).
- Repo `justfile`: `test`/`test-all` gain a build step for the fixture ONLY in
  `test-all` (keep the fast suite fast); or follow the `io_stress` pattern
  (opt-in binary, excluded from the default nextest filter). Decision: follow
  the `io_stress` pattern — a `rt_workspace` nextest binary excluded from
  `just test`, included in `just test-all`.
- New integration test `tests/tests/rt_workspace.rs`:
  1. build gate: skip with a clear message if the fixture isn't built (mirror
     `require_autoware()`).
  2. dump parity: rust vs python parse of `bringup.launch.xml`.
  3. `check`: contract + `--sched` validation passes on the committed files; one case asserts the overlay overrides the provider sidecar.
  4. sched smoke: launch with `--sched system.toml --sched-apply warn`,
     assert the apply path engages for the standalone node AND the composable
     (reusing the tolerant privileged/unprivileged assertions from
     `sched_apply.rs`).
  5. (privileged hosts only, auto-skipped otherwise) per-TID assertion:
     every thread of `control_node` shows FIFO 20 / cpu 0.
- `docs/guide/rt-scheduling.md`: replace the abstract quick-start with the
  fixture path; the guide's example blocks stay but gain a "runnable at
  `tests/fixtures/rt_workspace/`" pointer.

## Non-goals

- No custom message packages, no lifecycle nodes, no multi-container topology.
- No Autoware-scale realism — that fixture already exists.
- Not a nano-ros example (the shared `system.toml` is usable there, but wiring
  the RTOS side belongs to the nano-ros repo).

## Testing the fixture itself

The committed `system.toml` and manifest are validated in CI by step 3 above,
so spec/schema changes that would break the documented example fail loudly.
