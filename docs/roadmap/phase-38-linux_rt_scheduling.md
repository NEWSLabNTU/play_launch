# Phase 38: Linux Real-Time Scheduling Apply-Layer

**Status:** 📋 Planned
**Design of record:** [docs/superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md](../superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md)
**Builds on:** shared scheduling crate `ros-launch-manifest-sched` + `play_launch check --sched` (validate-now), already on `main`. Crate design: [docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md](../superpowers/specs/2026-07-01-shared-scheduling-crate-design.md).

## Overview

The shared scheduling crate already lets a user author a portable, per-platform
scheduling spec (`[tiers.*]` + `[[assign]]`), and `play_launch check --sched
<file.toml>` resolves it for the `posix` target and reports the tier table.
That is **validate-now**. Phase 38 is **apply**: during replay, set the Linux
scheduling policy, real-time priority, and CPU affinity on each spawned node
process according to its resolved `posix` tier.

Scope table + FQN matching are already solved (`ros/sched_loader.rs`
`sched_nodes_from_dump` / `join_fqn`), and `ros_launch_manifest_sched::resolve`
already yields the `ResolvedTierTable` carrying `sched_class`, `priority`, and
`core`. Phase 38 adds the syscall layer, the flag surface, and the actor hook.

## Architecture

```
--sched <file.toml> + --sched-apply {off|warn|strict}   (CommonOptions)
        │
        ▼
SchedPlan::build(dump, path, mode)          execution/sched_plan.rs
  sched_nodes_from_dump(dump) ──► resolve(.., "posix") ──► ResolvedTierTable
  invert members → HashMap<FQN, AppliedTier>
  warn: composable-in-non-default-tier; time_triggered → no SCHED_DEADLINE v1
        │  (per-member AppliedTier resolved at ActorConfig build time)
        ▼
ActorConfig { sched: Option<AppliedTier>, sched_mode }   member_actor/state.rs
        │
        ▼  post-spawn, by PID
regular_node_actor (after child.id())   container_actor::handle_running
        │
        ▼
apply_tier(pid, &AppliedTier)               execution/sched_apply.rs
  sched_setscheduler(SCHED_FIFO|RR, priority)   +   sched_setaffinity(core)
```

**Applied units (v1):** regular node processes and container processes, by PID,
post-spawn (re-applied on respawn because the hook is on the spawn path).

**Composables:** in `isolated` mode each composable is a `component_node`
process whose PID lives only in the C++ container manager (`ComponentEvent.msg`
has no `pid`). v1 does **not** schedule composables; a composable assigned a
non-`default` tier warns at plan-build. See 38.9 (fast-follow).

**Mechanisms (v1):** `SCHED_FIFO`/`SCHED_RR` + priority + core affinity.
`SCHED_DEADLINE` deferred (a `time_triggered` tier warns).

## Work items

### 38.1 `execution/sched_apply.rs` — syscall layer (planned)

Pure PID-in/result-out module. Types: `SchedApplyMode { Off, Warn, Strict }`
(clap `ValueEnum`, default `Warn`), `SchedPolicy { Fifo, Rr, Other }`,
`AppliedTier { policy, priority: i32, core: Option<u32>, tier_name }`,
`SchedApplyError { PermissionDenied, InvalidPriority, Syscall }` (thiserror).

- `apply_tier(pid, &AppliedTier) -> Result<(), SchedApplyError>`:
  `Fifo`/`Rr` → `libc::sched_setscheduler` with `sched_param{ sched_priority }`
  (validate priority `1..=99` → `InvalidPriority` without syscall); `core` →
  `libc::sched_setaffinity` (`CPU_ZERO`/`CPU_SET`); `Other` → affinity only,
  nonzero priority `debug!`-skipped.
- `has_sched_privilege() -> bool`: `euid == 0` OR `CAP_SYS_NICE` set in
  `/proc/self/status` `CapEff`.
- Unit tests: priority-range validation; `Other`+core path; privileged
  self-test (set `SCHED_FIFO` on a throwaway child, read back via
  `sched_getscheduler`/`sched_getparam`); unprivileged EPERM→`PermissionDenied`.

### 38.2 `execution/sched_plan.rs` — resolve + invert (planned)

`SchedPlan { by_fqn: HashMap<String, AppliedTier>, mode }` +
`SchedPlan::build(dump, sched_path, mode) -> eyre::Result<SchedPlan>` reusing
`sched_loader::sched_nodes_from_dump` + `ros_launch_manifest_sched::resolve(..,
"posix")`. Maps each `ResolvedTier` → `AppliedTier` (`sched_class` string →
`SchedPolicy`). Build-time warnings: composable member in a non-`default`
tier; `time_triggered` tier (SCHED_DEADLINE not applied v1). `for_fqn(&str)`
lookup. Unit tests for inversion, string→policy mapping, composable warning.

### 38.3 CLI flags (planned)

Add to `CommonOptions` (`cli/options.rs:245`): `sched: Option<PathBuf>` and
`sched_apply: SchedApplyMode` (default `Warn`). Mirrors the existing `--sched`
on `CheckArgs` and the `EnforceMode` enum pattern (`options.rs:340`). `off` =
resolve+report only. Absent `--sched` ⇒ apply-layer inert.

### 38.4 ActorConfig threading (planned)

Add `sched: Option<AppliedTier>` + `sched_mode: SchedApplyMode` to
`ActorConfig` (`member_actor/state.rs:13`). Build `SchedPlan` once in
`replay.rs`/`run.rs` (only when `--sched` set); at the `ActorConfig` build
sites (`replay.rs:536/564`, `run.rs:214`) look up each member's FQN
(`plan.for_fqn`) and store the resolved `Option<AppliedTier>`. The actor never
reconstructs an FQN.

### 38.5 Apply hooks (planned)

- Regular node: after `let pid = child.id()` (`regular_node_actor.rs:~164`),
  beside `process_registry.insert`/`write_pid_file`.
- Container: in `handle_running` after `spawn_process()`
  (`container_actor/mod.rs:485`).
- If `config.sched.is_some()` and `sched_mode ≠ Off`: `apply_tier(pid, tier)`,
  handle result per mode. Respawn re-applies automatically (hook on spawn path).

### 38.6 Preflight + mode error handling (planned)

Once at replay/run start (only if `--sched` set, mode ≠ `Off`):
`has_sched_privilege()`. Missing + `warn` → single `warn!` with hint `setcap
cap_sys_nice+ep <bin>`; apply still attempted. Missing + `strict` → abort
before spawning. Per-process: `warn` → log + continue; `strict` → abort the run.

### 38.7 Absorb latent `ProcessConfig::apply` (planned)

`cli/config.rs:284` has a never-called `apply(pid)` doing affinity + nice.
Move its affinity helper into `sched_apply` and delete the dead duplicate so
there is one affinity path.

### 38.8 Tests (planned)

Unit tests per 38.1/38.2. Integration smoke (`tests/`): `run <fixture>
--sched sys.toml --sched-apply warn` — assert the resolved-table log; when
privileged, `chrt -p <pid>` / `/proc/<pid>/stat` shows the policy for a
targeted node; when unprivileged, the warn-path hint appears and the run still
completes. Use `ManagedProcess` + unique `ROS_DOMAIN_ID` fixture patterns.

### 38.9 Composable scheduling — fast-follow (planned, after v1)

Extend `ComponentEvent.msg` with `int32 pid` (C++ already holds `child_pid` at
fork, `clone_isolated_component_manager.cpp:580`); Rust's `component_events.rs`
matches by `full_node_name` and calls `apply_tier(pid, ..)` on `LOADED`. Event-
driven, no polling. Touches C++/msgs + colcon rebuild — deliberately separated
from the all-Rust v1.

## Order and dependencies

38.1 → 38.2 → (38.3, 38.4 in parallel) → 38.5 → 38.6 → 38.7 → 38.8. 38.9 after
38.8. 38.1 is self-contained (no play_launch build coupling); 38.5/38.6 are the
only actor-lifecycle touch points.

## Risks and unknowns

- **Priority↔policy correctness:** RT priority range and FIFO vs RR semantics
  are fixed by the tier's `sched_class`+`priority`; malformed values are caught
  in 38.1 (`InvalidPriority`), not at the syscall.
- **CAP_SYS_NICE ergonomics:** most dev machines lack it. `warn` default keeps
  the run working (unscheduled) with a clear `setcap` hint; only `strict`
  aborts.
- **Affinity vs cgroups:** `sched_setaffinity` is per-process and does not
  compose with external cpuset cgroups; documented as a v1 boundary.
- **Timing race:** by-PID post-spawn is race-tolerant for FIFO/RR (process
  exists at `child.id()`, ROS init takes ms). No pre_exec needed.

## Out of scope for Phase 38

- `SCHED_DEADLINE` / `sched_setattr` (period/budget/deadline apply).
- Composable-node scheduling in v1 (38.9 fast-follow).
- Any crate schema/resolver change (consumes existing `ResolvedTierTable`).
- A privileged helper binary — play_launch applies from its own process.

## Tests

See 38.8. Gate: `apply_tier` + `SchedPlan::build` unit tests pass; smoke test
green on both privileged and unprivileged paths.
