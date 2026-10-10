# Phase 37 — Crate Split by Dependency Class

**Status:** 📋 planned, redesigned 2026-10-10. The original plan (2026-05-13,
seven sub-phases, none started) is in git history at `61074ca3`; §Review
below says what happened to each of its parts.

## Goal

Split the `play_launch` crate so that the parts with no business depending on
ROS stop depending on it, and so that those parts can be reused, or published,
on their own.

The motivating question is not "how big is this crate" but **"what does a
consumer of this code have to install to build it?"** Today the answer for
every line of `play_launch` is: a sourced ROS 2 workspace, colcon-generated
message crates, libpython and z3. That is true of the `sched_setattr(2)`
wrapper, the cgroup v2 delegation probe and the process-group supervisor, none
of which call into ROS.

No behavioural change. Every wave keeps the binary working and the suite
green.

## Non-goals

- **Faster incremental builds.** The original plan's fourth goal. Measured
  2026-10-10: a one-line edit in `web/` rebuilds in 5 s, one in
  `runtime_enforcement/` in 4 s. Not a reason to split.
- **A line-count target.** The old "~3k lines left in `play_launch`" was sized
  against 19k lines; the crate is now 43.6k. The success criterion is the
  dependency class of each crate, not its size.
- Changing the CLI, the on-disk artifacts, the SPSC event layout or the
  container control protocol's wire format.
- Actually publishing anything to crates.io. This phase makes crates
  *publishable* (`cargo publish --dry-run` passes) and stops there; when and
  under what names to publish is a separate decision.

## Review of the 2026-05-13 plan

| Old | What it was | Now |
|----:|-------------|-----|
| 37.1 | `play_launch_record_format` | **Obsolete.** `record.json` was retired in phase 47. The one leftover duplicate, `ScopeEntry`, is defined twice *inside* layer 2 (`parser/.../record/types.rs`, `resolve/src/ros/launch_dump.rs`) and is a small cleanup there, not a crate. |
| 37.2 | `play_launch_interception_types` | **Kept, as W1.** `EventKind` is still defined in both `src/play_launch/src/interception/mod.rs:33` and `src/play_launch_interception/src/event.rs:10`, held together by a comment. The coordination risk the plan named (the parser submodule) is gone since phase 55. |
| 37.3 | `play_launch_manifest_index` | **Done elsewhere.** The manifest index lives in `ros-launch-resolve` (layer 2); `play_launch::ros` holds no manifest code. |
| 37.4 | Per-verb CLI arg structs | **Mostly done.** `LaunchArgs` has 7 public fields, not 30. What remains is the reverse problem: `cli::config` is imported by six lower modules (see W0). |
| 37.5 | `play_launch_enforcement` | **Kept, as W5**, reframed: it is ROS-free today, but depends on the `ros-launch-manifest` git dependency. |
| 37.6 | `play_launch_executor` | **Rejected as framed.** It bundled `execution/` (ROS-free) with `member_actor/` (the most ROS-bound code in the tree) into one crate, so the new crate would have needed ROS exactly as the old one does. The ROS-free parts are extracted instead (W2–W4). |
| 37.7 | `play_launch_web` | **Deferred** (W6). Axum only, no ROS, but its data model is `member_actor`'s, and nobody has asked to reuse it. |

The old plan also proposed a `play_launch::compat` re-export module for the
transition. Not needed: nothing outside this repository links the
`play_launch` library (the integration tests drive the binary as a
subprocess), so each move can update its importers in the same commit.

## The axis: dependency class

The old plan split by *feature* (executor, web, enforcement). The split that
serves reuse is by *what a crate drags in*:

| Class | Depends on | Can go to crates.io |
|-------|-----------|---------------------|
| **A — Linux runtime** | `libc`, `serde`, crates.io only | yes |
| **B — contract-aware** | `ros-launch-manifest` (a git dependency) | not until that repo publishes |
| **C — ROS-bound** | `rclrs`, colcon-generated message crates, `pyo3` | never; ships via colcon and the wheel |

Measured from source (references per module to `rclrs`, `composition_interfaces`,
`play_launch_msgs`, `rcl_interfaces`, `diagnostic_msgs`, `lifecycle_msgs`):

| Module | Lines | ROS refs | Class today, by content |
|--------|------:|---------:|-------------------------|
| `member_actor/` | 9,892 | 51 | C |
| `execution/` | 7,693 | 2 (`startup_order.rs`) + `pyo3` (`node_cmdline.rs:228`) | mostly A, with a C seam |
| `commands/` | 6,397 | 8 | C (orchestration) |
| `runtime_enforcement/` | 3,306 | 0 | B |
| `cli/` | 2,745 | 0 | C (it configures everything) |
| `interception/` | 2,419 | 0 | A (consumer) + B (`measure.rs`) |
| `web/` | 2,033 | 0 | A by deps, C by data model |
| `monitoring/` | 1,972 | 0 | A |
| `ros/` | 1,708 | 49 | C |
| `sched.rs` | 1,627 | 0 | A |
| `diagnostics/` | 1,277 | 2 | C |
| `ipc/` | 908 | 5 (`container_protocol.rs`: a `From<rcl_interfaces::Parameter>`) | A with a C feature |
| `process/` | 291 | 0 | A |

Roughly 14k lines are class A by content and live in a crate that is class C.

The privileged helpers make this concrete. `play_launch_rt_helper`
(`CAP_SYS_NICE`) and `play_launch_io_helper` (`CAP_SYS_PTRACE`) are `[[bin]]`
targets of the `play_launch` package. The linker already discards what they do
not use (their `NEEDED` lists are libc only), but the *source* dependency set
a reviewer has to trust for a capability-carrying binary is the whole
package: axum, pyo3, rclrs, 30-odd crates. After W2 (rt helper) and W6 (io
helper), each builds from a crate whose dependencies are `libc`, `serde` and
one class-A library.

## Target layout

```
class A  (crates.io-ready; no ROS, no Python)
  spsc_shm                        exists; SPSC ring over memfd + eventfd
  rcl-interception-event      W1  #[repr(C)] event, EventKind, topic chunk codec
  linux-sched-attr            W2  sched_setattr, SCHED_DEADLINE, uclamp, TID sweep,
                                  cpuset partition probe
  cgroup-delegate             W3  delegation probe, per-group limits, join-at-birth,
                                  memory.events
  process-group-supervisor    W3  process groups, anchor, pdeathsig, pre_exec step
                                  reporting, MemAvailable spawn admission
  ros2-container-control      W4  play_launch_container framing + messages
                                  (feature `ros`: rcl_interfaces conversions)
  proc-sampler                W6  /proc + NVML sampling, CSV writer (optional)

class A, not published (play_launch's own privileged binaries)
  play_launch_rt_helper       W2  CAP_SYS_NICE helper + its wire protocol
  play_launch_io_helper       W6  CAP_SYS_PTRACE helper + its wire protocol

class B
  ros-launch-manifest-monitor W5  RuleEngine + ContractView over interception events

class C
  play_launch                     member_actor, ros, diagnostics, commands, cli,
                                  launch-semantic parts of execution, web
```

## Naming and licensing (decided 2026-10-10)

**Names are neutral**: they say what the crate does, not which project it
came from, because the strongest candidates (W2, W3) are useful to anyone
doing Linux process or RT work in Rust and that is what such a user searches
for. Every name above was unregistered on crates.io on 2026-10-10 (each
checked against the API, with `serde` and `procfs` as a positive control).
Unregistered is not reserved: re-check before the first real publish.

- Crate names are hyphenated (the crates.io convention); directories under
  `src/` use the same spelling. Code imports them with underscores, as cargo
  maps one to the other.
- `spsc_shm` keeps its name. It is already neutral, it is unregistered too, and
  renaming it buys nothing.
- `ros-launch-manifest-monitor` joins the `ros-launch-manifest-*` family
  because it is a consumer of that schema, and would be published alongside
  it.
- The two helpers keep `play_launch_` names: they are this project's
  privileged binaries, not libraries, and are never published. They are
  class A by dependencies so that a reviewer of a capability-carrying binary
  reads a small crate.

**Every crate this phase creates is `MIT OR Apache-2.0`**, the crates.io
convention, so a downstream user can take either. Each crate carries its own
`LICENSE-MIT` and `LICENSE-APACHE` (cargo packages only the crate directory,
so a repository-level file does not travel) and
`license = "MIT OR Apache-2.0"` in its manifest. `spsc_shm` was relicensed
from `Apache-2.0` this way on 2026-10-10, and `cargo publish --dry-run`
passes for it with both files in the package.

This does not relicense `play_launch` itself (BSD-3-Clause),
`play_launch_interception` or `play_launch_parser` (Apache-2.0). Code moving
from `play_launch` (BSD-3-Clause) into a new dual-licensed crate is
relicensed by its author as part of the move. That is fine for code written by
the project's own authors. Check `git log` per moved file for outside
contributors before each wave.

**Where they live.** A crate used only by `play_launch` becomes a member of the
root workspace under `src/`. A crate also used by the standalone interception
cdylib (W1's event crate) follows `spsc_shm`'s pattern instead: its own
`[workspace]`, listed in the root manifest's `exclude`, with a `/target/`
`.gitignore` (CLAUDE.md, Building & Testing). A path dependency that is a
member of one workspace cannot be built from another. colcon ignores all of
these, as it already ignores `spsc_shm`: no `package.xml`, reached as path
dependencies through `cargo-ament-build`.

## Waves

### W0 — Untangle the back-edges (no new crates)

A crate boundary cannot cross a cycle. The module graph today (references
counted from `crate::<mod>` paths) has these edges pointing the wrong way:

| Edge | Refs | Cause | Fix |
|------|-----:|-------|-----|
| `util` → `cli` | 14 | `logging::is_verbose()` matches on the parsed `Options` | Set a flag once at startup; `util` stops knowing the CLI |
| `execution`, `interception`, `diagnostics`, `member_actor` → `cli::config` | 10 | Each module's settings struct is defined in `cli/config.rs` | Each settings struct moves to the module that consumes it; `cli/config.rs` composes them |
| `cli` → `execution` | 4 | `SchedApplyMode`, `StartupLimits` | Falls out of the row above |
| `cli` → `commands` | 1 | `MeasureArgs` defined in `commands/measure.rs` | Move the args struct to `cli` |
| `execution` → `member_actor` | 4 | `startup_order.rs` drives `MemberHandle` | `startup_order` is orchestration: move it to `commands/` (or a `runtime` module) |
| `monitoring` → `web` | 2 | The resource monitor pushes to `SystemMetricsBroadcaster` | Monitor publishes on a channel; the web layer subscribes |
| `monitoring` → `commands` | 1 | `io_helper_client` calls `cap_status::diagnose` | Move `cap_status` down beside the helpers it diagnoses |
| `monitoring` → `execution` | 1 | `CgroupTree::memory_current_for_pid` | Fine once cgroup is its own crate (W3): a downward edge |
| `ros` → `execution` | 1 | `parameter_conversion` calls `node_cmdline::eval_python_str` | Move the Python `$(eval)` fallback out of `node_cmdline`, into a small module class C owns |
| `interception` → `runtime_enforcement` | 3 | The consumer loop owns and ticks the `RuleEngine` | Consumer takes a sink trait (`on_event`, `tick`); the engine implements it |
| `runtime_enforcement` → `cli` | 1 | `EnforceMode` | Moves with the engine |

`ipc` → `sched` and `member_actor` → `execution` point downward and stay.

**Gate.** A `just check-crate-layers` recipe, added to `just check`, written in
the style of `check-layer2-isolation`: for every class-A crate,
`cargo tree -e normal` contains no `rclrs`, no `*_msgs`/`*_interfaces`, no
`pyo3`, no `ros-launch-manifest-*`; for class B, no ROS and no `pyo3`. Until
the crates exist (W0 itself), the gate runs over a module allowlist instead,
failing on any of the edges above reappearing. It fails in both directions: an
allowlisted edge that no longer exists is an error too, so the licence cannot
outlive the edge (the `check-sched-rates` pattern).

**Acceptance.** The edges above are gone, the gate is in `just check`, and
`just test` and `just check` are green.

### W1 — Interception event crate

`InterceptionEvent`, `EventKind`, the topic-name chunk codec and the layout
tests, from both copies, into `rcl-interception-event`: one `#![no_std]` crate
with no dependencies. Despite the name it links nothing from rcl. It describes
the record that the rcl hooks write, and that is all.
`play_launch_interception` (cdylib) and `play_launch::interception` both
depend on it. The consumer, aggregation, `trace` and `measure` stay where they
are.

**Acceptance.** One definition of `EventKind` in the tree (a grep in the
gate). A size/offset test pins the 56-byte layout, so the two sides cannot
drift again. The interception integration tests pass against the rebuilt
`.so`.

### W2 — Linux scheduling crate

`linux-sched-attr`: `sched.rs` (the `sched_setattr(2)` declaration that `libc`
lacks, the 56-byte `sched_attr` layout and its offset test, `SCHED_DEADLINE`,
uclamp, the TID sweep) and `execution/cpuset.rs` (partition probe with
readback). `play_launch_rt_helper` becomes its own unpublished crate holding
the binary and `ipc/sched_protocol.rs`, depending on `linux-sched-attr` and
`serde` only. The helper protocol is play_launch's, so it stays out of the
published crate.

This is the best reuse candidate in the tree: `sched_setattr` with
`SCHED_DEADLINE` and uclamp is something anyone doing Linux RT in Rust has to
write by hand today, and the partition readback (`root` succeeding and reading
back `root invalid`) is non-obvious knowledge that belongs with the call.

**Acceptance.** The helper builds with the new crate's dependencies only.
`cargo publish --dry-run` passes. `just setcap` still finds the helper at the
same install path. The sched integration tests pass.

### W3 — Process supervision and cgroup crates

Two crates, because they are independently useful:

- `cgroup-delegate`, from `execution/cgroup.rs`: the delegation probe (attempt `mkdir`, because the
  controller list cannot tell a login shell from a delegated scope),
  `subtree_control` one level at a time, limits written before the fork,
  join-at-birth by writing `"0"`, zero-charge-is-absent, `memory.events`.
- `process-group-supervisor`, from `process/`, the `pre_exec` step machinery and allocation-free failure
  reporting from `node_cmdline.rs`, `start_delay.rs`, and the `MemAvailable`
  admission in `startup_governor.rs`.

`node_cmdline.rs`'s rendering of a ROS command line from the model stays in
`play_launch`: that is launch semantics, not process supervision.

**Acceptance.** As W2. The anchor-PGID tests, the #0024 spawn-race tests and
`run_signal.rs` (#0061) pass unchanged.

### W4 — Container control protocol crate

`ros2-container-control`, from `ipc/container_protocol.rs`, the protocol of record with
`play_launch_container`, with the `rcl_interfaces::Parameter` conversions
behind a `ros` feature. That lets a different supervisor talk to
`play_launch_container` without play_launch, which is the reuse case. It also
gives the C++ side's JSON codec one Rust file to be checked against.

**Acceptance.** Default features have no ROS crates in `cargo tree`.
`just test-cpp` and the `load_policy` and control-channel tests pass.

### W5 — Enforcement crate (class B)

`ros-launch-manifest-monitor`, from `runtime_enforcement/` (the `RuleEngine`
and `ContractView`), depending on W1
and the manifest crates. Its unit tests run with `cargo test` and no sourced
ROS environment, which is not true today, because they live in a crate that
needs one.

Open on placement: this workspace, or `ros-launch-resolve` so nano-ros can run
the same observer. The recommendation is this workspace until nano-ros asks.
It consumes play_launch's interception events, not anything layer 2 produces.

**Acceptance.** `cargo test -p ros-launch-manifest-monitor` passes in a shell with no ROS
sourced. `just fault` still reports live equal to post-hoc.

### W6 — Optional: monitoring and web

`monitoring/` becomes `proc-sampler` cheaply after W0 (NVML behind a
feature), and `play_launch_io_helper` moves out with `ipc/protocol.rs` the way
the rt helper does in W2. `web/` needs its own view types, with an adapter from
`member_actor`'s, before it can move. Do this only if someone wants either
piece separately; maintainability alone does not justify the adapter.

## Success criteria

- [ ] `just check-crate-layers` is in `just check` and green.
- [ ] Every class-A crate passes `cargo publish --dry-run` and has no ROS, no
  `pyo3` and no git dependency in `cargo tree`.
- [ ] Every new crate is `MIT OR Apache-2.0` and ships both licence files (the
  gate checks the manifest field and the files).
- [ ] `EventKind` is defined exactly once.
- [ ] Both privileged helpers build from class-A crates only.
- [ ] `just test-all` and `just check` are green after every wave, and the
  wheel (`just build-wheel`) installs and runs `play_launch launch` unchanged.

## Open questions

Names and licences are decided (above).

1. **Versioning.** Path dependencies need a `version` to publish. One
   lockstep version for all class-A crates, or independent versions?

## Risks

- **The colcon build.** `cargo-ament-build` resolves path dependencies, as it
  does for `spsc_shm` today, but every new crate is another place for the
  `[[patch.unused]]` churn and the `.cargo/config.toml` walk to reach. Run
  `just build` (not just `cargo build`) after each wave, and check the wheel.
- **Moving tests with code.** Several modules carry long `#[cfg(test)]`
  blocks whose fixtures reach into `cli::config`. W0 has to move those
  fixtures with the settings structs, or the test modules become the back-edge.
- **Lockfiles carry the colcon patch list.** `spsc_shm`'s committed
  `Cargo.lock` already holds `[[patch.unused]]` entries for ROS message crates,
  picked up from the colcon `.cargo/config.toml` that cargo's config walk
  reaches from inside the repository. A published library's lockfile is
  ignored by consumers, so this is noise, not breakage. But run
  `cargo publish --dry-run` from a clean clone (or with `--config` pointing
  away from that file) before trusting it.
- **A `pub` surface by accident.** Today everything is `pub` because one crate
  is one privacy domain. A new crate's API is whatever is `pub` at the moment
  of the move. Review each crate's public items before calling it publishable.
