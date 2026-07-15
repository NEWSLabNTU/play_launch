# Phase 38: Linux Real-Time Scheduling Apply-Layer

**Status:** ✅ 38.1–38.10 complete (RT applies, verified on-kernel; non-root RT via a privileged helper + per-TID fix).
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
process. **As of 38.9 these are scheduled**: `ComponentEvent` carries the child
`pid`, and the container actor applies the composable's resolved tier on
`LOADED`. In `observable`/`stock` modes composables are in-process (no separate
PID) and are not individually schedulable — the LOADED apply is guarded on
`event.pid > 0`, so those modes skip it. (Original v1, 38.1–38.8, applied only
to regular nodes + containers and warned on composables; 38.9 removed that
limitation.)

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
`has_sched_privilege()`. Missing + `warn` → single `warn!` hinting to run as
root; apply still attempted. Missing + `strict` → abort before spawning.
Per-process: `warn` → log + continue; `strict` → abort the run.

**Obtaining the privilege — and why `setcap` on the main binary is NOT an
option.** Granting a file capability to an ELF makes the kernel run it in
secure-execution mode (`AT_SECURE=1`), which makes the dynamic linker ignore
`LD_LIBRARY_PATH`. The main `play_launch` binary links ~22 ROS shared libraries
found only via `LD_LIBRARY_PATH` (from ROS's `setup.bash`) and carries no
`DT_RUNPATH`, so `setcap cap_sys_nice+ep` on it makes it fail at startup with
`error while loading shared libraries: libcomposition_interfaces...so`. This is
precisely why the existing `play_launch_io_helper` (which links **zero** ROS
libraries) is the thing that carries `CAP_SYS_PTRACE`.

**Fixed by 38.10:** `play_launch setcap` / `just setcap` now grant `CAP_SYS_PTRACE`
to the I/O helper **and** `CAP_SYS_NICE` to the ROS-free `play_launch_rt_helper`
(main binary still stripped/uncapped), and `verify` checks all three. RT
scheduling (`--sched`) therefore works **unprivileged** — no `sudo -E play_launch
...` required. Running the whole ROS stack as root just to set a scheduling
policy was a bad trade; delegating the syscalls to a small capped helper avoids
it entirely.

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

### 38.9 Composable scheduling — done

`ComponentEvent.msg` gained `int32 pid`, populated at LOADED/CRASHED in
`clone_isolated_component_manager.cpp` (captured before the `child` move).
Each composable's `AppliedTier` is resolved at the builder (same `fqn_for`
key as `SchedPlan`) and stored on `ComposableNodeMetadata.sched`; the
container actor's LOADED handler applies it via the entry (matched by
`unique_id`), guarded on `sched_mode != Off && event.pid > 0`. Event-driven,
no polling. Verified end-to-end on a ROS host (`tests/tests/sched_apply.rs::composable_scheduling_engages_on_isolated_container`).

> ⚠ **Known defect — composables only get their MAIN thread scheduled.**
> Linux scheduling attributes are **per-thread**: `sched_setscheduler(pid, …)`
> affects only the thread whose TID == pid. A composable is scheduled on the
> `LOADED` event, i.e. *after* `component_node` has initialized, by which point
> it already has ~11 threads (the ~10 DDS/rmw threads spawn 0.1–0.5s after
> exec). The 10 worker threads therefore stay `SCHED_OTHER` — and they are the
> ones doing the work. Regular nodes/containers are unaffected: they are
> scheduled at ~+1ms when only the main thread exists, and later threads
> **inherit** the policy (glibc default `PTHREAD_INHERIT_SCHED`).
> **Fixed by 38.10**, which applies per-TID across `/proc/<pid>/task/*`.
> Note also that `chrt -p <pid>` reads only the main thread, so PID-level
> verification does not catch this.

**v1 strict boundary:** unlike regular nodes/containers (whose strict apply
failure aborts the run), a *per-composable* strict apply failure mid-run logs
at `error!` and leaves that composable running unscheduled — it does not tear
down the container. This is narrow: build-time config validation
(`SchedPlan::build`) + the pre-spawn `CAP_SYS_NICE` preflight already catch the
deterministic and permission causes before any composable loads, leaving only
an unusual per-process EPERM (e.g. composable-specific rlimit). A hard
coordinated shutdown from the async ComponentEvent handler is deferred (would
be a 38.10).

### 38.10 Privileged RT helper — non-root RT scheduling — done

Design of record:
[docs/superpowers/specs/2026-07-14-rt-helper-design.md](../superpowers/specs/2026-07-14-rt-helper-design.md).

Removes the root requirement, and fixes the per-thread defect noted in 38.9.

**Principle:** keep capabilities off the main functional binary (a file cap sets
`AT_SECURE`, the loader drops `LD_LIBRARY_PATH`, and the ~22 ROS libs vanish).
Put each privileged operation in a small ROS-free helper holding exactly one
capability — the pattern `play_launch_io_helper` already established.

```
play_launch  (uncapped, ROS, unprivileged)
    │                                  │
    │ ApplySched{pid, tier}            │ ReadProcIo{pids}
    ▼                                  ▼
play_launch_rt_helper              play_launch_io_helper
  CAP_SYS_NICE only                  CAP_SYS_PTRACE only
```

A **separate** RT helper (not `CAP_SYS_NICE` bolted onto the io_helper) so caps
never pool: monitoring-only runs carry no RT privilege, RT-only runs carry no
ptrace privilege.

**Why the capability is needed at all:** `RLIMIT_RTPRIO` is 0 for a normal user,
so any RT request from an unprivileged process is `EPERM`. `CAP_SYS_NICE`
bypasses that limit. File caps do not change UID — the helper runs as the
invoking user, so same-owner already holds and the cap does exactly one job.

**Per-TID apply (the correctness fix).** Scheduling attributes are per-thread.
The helper enumerates `/proc/<pid>/task/*` and applies to every TID. Threads
created afterwards inherit the policy (glibc default `PTHREAD_INHERIT_SCHED`,
verified empirically), so the sweep plus inheritance covers the process. This
repairs composables, which are scheduled after `component_node` init when ~11
threads already exist.

**Work items:**
- **38.10.1** — done. Lib split: syscall core lives in `pub mod sched` (serde,
  no clap) so `src/bin/*` helpers can use it; `apply_tier` is per-TID.
  `execution/sched_apply.rs` keeps `SchedApplyMode` + re-exports.
- **38.10.2** — done. `ipc::sched_protocol` (own `Request`/`Response`, reusing
  the length-prefixed bincode framing) + `[[bin]] play_launch_rt_helper`;
  ROS-free (`ldd` = 0 ROS libs); wired into colcon + wheel bundle.
- **38.10.3** — done. `RtHelperClient` + owner task + `SchedHelper(mpsc::Sender)`
  handle (`Clone + Debug`, so it fits `ActorConfig`); lifetime: `PR_SET_PDEATHSIG`,
  `kill_on_drop`, graceful `Shutdown`→ack→`wait()`-with-timeout→kill, and a
  per-request timeout so a wedged helper degrades per `--sched-apply` rather
  than hanging. Spawned only when `--sched` is used (independent of monitoring).
  All 3 apply sites reroute through it (helper-first, direct `apply_tier` only
  when running as root and the helper is unavailable).
- **38.10.4** — done. `setcap` grants `cap_sys_nice+ep` to rt_helper and
  `cap_sys_ptrace+ep` to io_helper (main binary still stripped, self-heals a
  stray cap); `verify` does per-binary/per-cap `contains` checks (accepting
  both `getcap`'s `=ep` and `setcap`'s `+ep` forms) across all three binaries.
  The pre-spawn preflight runs **unconditionally, before the helper is
  spawned** (privilege is whether the apply can succeed — root **or** a
  capped rt_helper — independent of whether the helper process starts; an
  uncapped helper spawns fine but every apply would EPERM). Its warning/abort
  message always contains `cap_sys_nice` and points at `play_launch setcap`.
  If the helper then fails to spawn and the process is not root, Strict
  aborts at that boundary too (Warn logs and proceeds unscheduled).
- **38.10.5** — done. `just verify-sched-rt` now runs fully **unprivileged**
  (no sudo) and checks **every TID** of every node under
  `/proc/<pid>/task/*` (not just `chrt -p <pid>`, which only reads the main
  thread and would miss composables), failing non-zero if any thread of a
  scheduled node isn't `SCHED_FIFO`.

**Resolved — composable PID reuse (TOCTOU), via start-time identity check
(follow-up T1).** Regular nodes and containers are safe by construction: the
Rust side holds the un-reaped `Child`, so the kernel cannot recycle that PID
before the apply. Composables are scheduled from `event.pid` on a
`ComponentEvent`, with no pidfd/Child held on the Rust side — the C++
container reaps independently — so if the composable died between the LOADED
publish and the apply, a recycled PID could get scheduled instead.

Fix: `ComponentEvent` carries a new `uint64 start_time` field — the process's
`/proc/<pid>/stat` field 22 (clock ticks since boot), captured by the
container right after fork, at the same time as `pid`. This value uniquely
identifies a PID *incarnation*: a recycled PID gets a different `start_time`.
Before applying sched to a composable, Rust re-reads `/proc/<pid>/stat` for
the live process and compares; a mismatch (or the process being gone) skips
the apply with a warning instead of scheduling the wrong process.

**Wire-compat rule:** `start_time == 0` means "no identity check possible"
(older container binary that doesn't populate the field, or a CRASHED/
LOAD_FAILED event where the child is already reaped) — Rust treats 0 as
"unknown" and falls back to today's behavior (apply guarded on `pid > 0`
only). The check only skips the apply when `start_time > 0` **and** it no
longer matches the live `/proc` value.

**Residual (accepted):** if the child dies in the sub-microsecond window
between its ready-pipe "OK" and the container's `proc_start_time` read, the
event carries `start_time = 0` and that one apply falls back to the pid-only
guard. This shrinks the original IPC-hop-sized window to microseconds rather
than eliminating it outright.

**IPC cost:** control-plane only — once per process at spawn/respawn/LOAD
(~200 round-trips for Autoware, at startup). Zero steady-state cost; the kernel
does the scheduling thereafter.

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
- Any crate schema/resolver change (consumes existing `ResolvedTierTable`).

## Tests

See 38.8. Gate: `apply_tier` + `SchedPlan::build` unit tests pass; smoke test
green on both privileged and unprivileged paths.
