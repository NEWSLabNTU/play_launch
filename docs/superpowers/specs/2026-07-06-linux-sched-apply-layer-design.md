# Linux Scheduling Apply-Layer — Design

**Date:** 2026-07-06
**Status:** Implemented (38.1–38.8) + composable scheduling (38.9) — see `docs/roadmap/phase-38-linux_rt_scheduling.md`. NOTE: the "no composable-node scheduling" non-goal below was v1-only; 38.9 lifted it (composables are scheduled in `isolated` mode via `ComponentEvent.pid`).
**Repo:** `play_launch` (Linux)
**Builds on:** `ros-launch-manifest-sched` crate + `check --sched` (validate-now).
Design of record for the spec itself:
`docs/superpowers/specs/2026-07-01-shared-scheduling-crate-design.md`.

## Goal

Turn the shared scheduling spec from **validate-now** into **apply**: when
`play_launch` replays a launch, set the Linux scheduling policy, real-time
priority, and CPU affinity on each spawned node process according to the
resolved `posix` tier. This is "phase 2" from the shared-crate design's
Consumers section.

## Scope (v1)

- **Applied units:** regular node processes and container processes, scheduled
  **by PID** immediately after spawn.
- **Composable nodes:** in the default `isolated` container mode each composable
  is a separate `component_node` process, but its PID lives only in the C++
  container manager (`ComponentEvent.msg` carries no `pid`). v1 does **not**
  apply scheduling to composables; a composable assigned a non-`default` tier
  produces a warning at plan-build time. Composable coverage is a documented
  fast-follow (extend `ComponentEvent.msg` with `pid` — "option C").
- **Mechanisms:** `sched_setscheduler(SCHED_FIFO|SCHED_RR, priority)` +
  `sched_setaffinity(core)`. `SCHED_DEADLINE` (from `deadline_us`/`period_us`/
  `budget_us`) is **deferred** — validated but not applied; a `time_triggered`
  tier warns.

## Non-goals (v1)

- No SCHED_DEADLINE / `sched_setattr`.
- No composable-node scheduling (isolated-mode PIDs unreachable from Rust).
- No changes to the crate schema or resolver — this consumes the existing
  `ResolvedTierTable`.
- No new privileged helper binary; play_launch applies from its own process
  (which already spawns the children).

## Flag surface

Added to `CommonOptions` (`src/play_launch/src/cli/options.rs:245`) so
`replay` / `launch` / `run` all pick them up:

- `--sched <PATH>` — the system scheduling TOML (same file `check --sched`
  validates). Mirrors the existing `CheckArgs.sched` field.
- `--sched-apply <off|warn|strict>` — `SchedApplyMode` enum (mirrors the
  existing `EnforceMode` at `options.rs:340`). **Default `warn`** when
  `--sched` is given.
  - `off` — resolve + report the tier table, apply nothing (no syscalls).
  - `warn` — apply per process; on `EPERM`/failure log a warning and continue
    (that process keeps default scheduling).
  - `strict` — any capability/apply failure aborts the run.

If `--sched` is absent, the apply-layer is entirely inert (current behavior).

## Components

### `execution/sched_apply.rs` (new)

The syscall layer. Pure, PID-in / result-out, unit-testable.

```rust
pub enum SchedApplyMode { Off, Warn, Strict }   // clap ValueEnum, default Warn

/// Pre-resolved, platform-lowered knobs for one node (built from ResolvedTier).
pub struct AppliedTier {
    pub policy: SchedPolicy,        // Fifo | Rr | Other
    pub priority: i32,              // RT priority (1..=99) for Fifo/Rr; ignored for Other
    pub core: Option<u32>,
    pub tier_name: String,          // for diagnostics
}

pub enum SchedPolicy { Fifo, Rr, Other }

#[derive(thiserror::Error)]
pub enum SchedApplyError {
    PermissionDenied { pid: u32 },              // EPERM
    InvalidPriority { pid: u32, priority: i32 },// EINVAL on priority
    Syscall { pid: u32, call: &'static str, errno: i32 },
}

/// Apply policy+priority+affinity to an already-spawned PID.
pub fn apply_tier(pid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError>;

/// Preflight: can this process set RT scheduling at all?
/// euid == 0 OR CAP_SYS_NICE present in /proc/self/status CapEff.
pub fn has_sched_privilege() -> bool;
```

Mapping rules inside `apply_tier`:
- `Fifo`/`Rr` → `libc::sched_setscheduler(pid, policy, &sched_param{ sched_priority })`
  after clamping/validating priority to `1..=99` (`InvalidPriority` otherwise).
- `core = Some(n)` → `libc::sched_setaffinity` with a `cpu_set_t` holding `n`
  (reuse the `CPU_ZERO`/`CPU_SET` logic from the latent `ProcessConfig::apply`
  at `cli/config.rs:307` — absorb it here, don't keep two affinity paths).
- `Other` → affinity only; a non-zero priority is ignored with a `debug!` note.

`SchedPolicy` is derived from the tier's `sched_class` string when building
`AppliedTier` (see plan build): `"SCHED_FIFO"→Fifo`, `"SCHED_RR"→Rr`,
`"SCHED_OTHER"`/absent→`Other`.

### `execution/sched_plan.rs` (new, or a submodule of sched_apply)

Bridges the crate's `ResolvedTierTable` to a per-node lookup.

```rust
pub struct SchedPlan {
    by_fqn: HashMap<String, AppliedTier>,   // FQN -> knobs
    pub mode: SchedApplyMode,
}
impl SchedPlan {
    /// Resolve the TOML for `posix`, invert members→AppliedTier, warn on
    /// composable members in non-default tiers.
    pub fn build(dump: &LaunchDump, sched_path: &Path, mode: SchedApplyMode)
        -> eyre::Result<SchedPlan>;
    pub fn for_fqn(&self, fqn: &str) -> Option<&AppliedTier>;
}
```

`build` reuses `sched_loader::sched_nodes_from_dump` + `ros_launch_manifest_sched::resolve(.., "posix")`.
A `time_triggered` tier (deadline/budget present) logs a one-time `warn!` that
SCHED_DEADLINE is not applied in v1. A member FQN that corresponds to a
`load_node` (composable) record and lands in a non-`default` tier logs
`warn!("composable <fqn>: Linux scheduling not applied in v1")`.

### Threading into the actor system

- `ActorConfig` (`member_actor/state.rs:13`) gains
  `sched: Option<AppliedTier>` + `sched_mode: SchedApplyMode`. The per-member
  tier is resolved **once at build time**, so the actor never reconstructs an
  FQN or holds the whole plan — it just applies what it was handed.
- `SchedPlan` is built once in `replay.rs`/`run.rs` (only when `--sched` is
  set), right after the dump + runtime config load.
- At the `ActorConfig` build sites (`replay.rs:536`, `replay.rs:564`,
  `run.rs:214`) the member's FQN is already known from its record (the same
  `effective_ns`/`join_fqn` the loader uses; the container already has
  `full_node_name()` at `process_lifecycle.rs:78`). Look it up via
  `plan.for_fqn(fqn)` and store the resulting `Option<AppliedTier>` on the
  config.

### Apply hook (lifecycle)

- **Regular node:** in `regular_node_actor.rs` right after
  `let pid = child.id()...` (`:164`), beside the existing
  `process_registry.insert` / `write_pid_file`. If `config.sched` is `Some`
  and `config.sched_mode ≠ Off`, call `apply_tier(pid, tier)` and handle the
  result per mode.
- **Container:** after `spawn_process()` returns, in the running transition
  (`container_actor/mod.rs:485 handle_running`), same apply from
  `config.sched`.
- **Respawn:** because the hook is on the (re)spawn path, a respawned process's
  new PID is re-scheduled automatically — no extra bookkeeping.
- **Timing:** by-PID post-spawn is race-tolerant for FIFO/RR (the process
  exists at `child.id()`; ROS node init takes milliseconds during which the
  policy is already set).

## Error handling & permissions

- **Preflight** once at replay/run startup (only if `--sched` set and mode ≠
  `Off`): call `has_sched_privilege()`.
  - Missing + `warn` → single `warn!` with hint
    `setcap cap_sys_nice+ep <play_launch binary>` (or run as root); apply still
    attempted (each will EPERM→warn), so the run proceeds unscheduled.
  - Missing + `strict` → return an error before spawning any node.
- **Per-process** in the actor hook:
  - `warn` mode → `warn!("sched apply failed for <fqn> (pid): <err>")`, continue.
  - `strict` mode → propagate an error that aborts the coordinator/run.
- `SchedApplyError` classifies `EPERM` (`PermissionDenied`), priority `EINVAL`
  (`InvalidPriority`), and other errnos (`Syscall`).

## Testing

- **`apply_tier` unit tests** (`execution/sched_apply.rs`):
  - Priority validation: out-of-range → `InvalidPriority` without syscall.
  - `Other` policy with `core` set → affinity attempted, priority skipped.
  - Privileged self-test gated on `has_sched_privilege()`: set `SCHED_FIFO`
    prio 10 on the test's own PID, read back via `libc::sched_getscheduler` /
    `sched_getparam`, assert; when unprivileged, assert the call maps `EPERM`
    to `PermissionDenied` (spawn a throwaway `sleep` child and target it).
- **`SchedPlan::build` unit tests:** FQN→AppliedTier inversion; `SCHED_OTHER`
  vs `SCHED_FIFO` string mapping; composable-in-non-default-tier emits the
  warning (assert via a captured log or a returned diagnostics list).
- **Integration smoke** (`tests/`): `run <fixture> --sched sys.toml
  --sched-apply warn` on a small fixture launch. Assert: the resolved-table
  log prints; when privileged, `chrt -p <pid>` / `/proc/<pid>/stat` shows the
  policy for a targeted node; when unprivileged, the warn-path hint appears and
  the run still completes. Follow the existing `ManagedProcess` + unique
  `ROS_DOMAIN_ID` fixture patterns.

## File-level summary

| File | Change |
|---|---|
| `src/play_launch/src/execution/sched_apply.rs` | NEW — `apply_tier`, `AppliedTier`, `SchedPolicy`, `SchedApplyError`, `has_sched_privilege` |
| `src/play_launch/src/execution/sched_plan.rs` | NEW — `SchedPlan` (resolve + invert + composable warn) |
| `src/play_launch/src/cli/options.rs` | ADD `sched: Option<PathBuf>` + `sched_apply: SchedApplyMode` to `CommonOptions`; `SchedApplyMode` enum |
| `src/play_launch/src/member_actor/state.rs` | ADD `sched: Option<AppliedTier>` + `sched_mode: SchedApplyMode` to `ActorConfig` |
| `src/play_launch/src/member_actor/regular_node_actor.rs` | apply hook after PID capture (~:164) |
| `src/play_launch/src/member_actor/container_actor/mod.rs` | apply hook in `handle_running` |
| `src/play_launch/src/commands/replay.rs`, `run.rs` | build `SchedPlan`; populate `ActorConfig.sched` |
| `src/play_launch/src/cli/config.rs` | absorb the latent `ProcessConfig::apply` affinity helper into `sched_apply` (remove the dead duplicate) |
