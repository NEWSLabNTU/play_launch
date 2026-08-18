# CLAUDE.md

Guide for Claude Code when working with this repository.

## Project Overview

ROS2 Launch Inspection Tool - Records and replays ROS 2 launch executions for performance analysis:
- **play_launch_parser** (Rust): Parses launch files into the resolved SystemModel (`system_model.yaml`, the ONE user-facing artifact — Phase 47 hard-removed `record.json` as a dump/replay artifact).
- **dump_launch** (Python): Alternative parser for maximum compatibility
- **play_launch** (Rust): Replays with resource monitoring
- **play_launch_analyzer** (Python): Analyzes and visualizes logs
- **play_launch_container** (C++): Custom component container with event publishing
- **play_launch_msgs** (C++): ROS 2 message definitions (ComponentEvent.msg)

## Launch File Parsing

**Default**: Rust parser (`play_launch_parser`, 3-12x faster, 381+ tests, 100% Autoware)
**Alternative**: `play_launch launch <pkg> <file> --parser python`

**CRITICAL RULE**: When Rust and Python parser behaviors differ, **Python's behavior is always correct**. Fix Rust, not Python. Validate with `just compare-dumps` in test workspaces.

**Model parity (Phase 46, moved onto models in Phase 47.B1)**: both parsers produce the SAME complete SystemModel — structure, contracts, and scheduling all apply on the shared scope table (`ScopeOrigin.path`, Phase 40.1) independent of which parser built it (`resolve --parser {rust,python}`). Parity comparison (`just compare-dumps`, `scripts/compare_models.py`) runs on the SystemModel itself — `record.json` and `scripts/compare_records.py` are retired (Phase 47.B2/B5).

### Parser Architecture

The parser evaluates conditions during parsing and processes only the selected path. Key rules:
- **A container's `namespace=` does NOT reach its composable nodes** (#0021).
  It names the container PROCESS and is never pushed onto the context, so a
  `<composable_node>` with no namespace of its own takes
  `context.current_namespace()`. Reading the container's resolved namespace
  instead doubled a segment for every composable in such a container
  (`/system/system_monitor/system_monitor/cpu_monitor`), and did so in the
  RUNNING system, not just the model — `LoadNodeRecord.namespace` is what goes
  into the LoadNode request, so a graph check could not see it and only stock
  `ros2 launch` disagreed. Fixture:
  `tests/fixtures/launch/test_container_namespace_scope.launch.xml`.
- **Conditional substitutions** (IfElse, Equals, etc.): call `perform()` to evaluate → "true"/"false"
- **LaunchConfiguration substitutions**: call `__str__()` to preserve as `$(var name)` for replay-time
- **Float parameters**: always include decimal point (`0.0` not `0`) for ROS type preservation
- **`<let>` statements**: sequential parse-time resolution — values resolved immediately, stored in record.json
- **Runtime fallback**: unresolved `$(var ...)` in executable names resolved at replay time (`src/execution/node_cmdline.rs`)
- **YAML params**: substitutions in YAML files resolved and typed before passing to nodes (`src/params.rs::load_and_resolve_param_file()`)

Implementation: `src/python/api/utils.rs` (substitution handling), `src/params.rs` (YAML resolution)

### Launch Tree Scoping (Phase 30)

The parser tracks which launch file each node originates from via a **scope table**, carried in the parser's in-memory `LaunchDump` intermediate (Phase 47: no longer written to disk as `record.json`) and in the SystemModel's `structure.scopes`. Each `<include>` creates a new scope entry with `(pkg, file, ns, args, parent)`. Nodes reference their scope via `scope: Option<usize>`.

- **Scope types**: `ScopeEntry`, `ScopeTable` in `record/types.rs` (parser) and `launch_dump.rs` (executor)
- **Both parsers**: Rust (`include.rs`, `xml_include.rs`, `python_exec.rs`) and Python (`visitor/entity.py`, `visitor/include_launch_description.py`) produce matching scope tables
- **record.json**: `scopes: [...]` (top-level) + `scope: N` (per node/container/load_node)
- **Scope stamping**: captures stamped with `scope_id` during include processing; records stamped in `entity.rs`
- **Member name mapping**: `node_scope_map` keyed by `name.or(exec_name)` to match actor system member names
- **CLI**: `play_launch context <system_model.yaml> --tree|--node <FQN>|--launch <pkg> <file>` (Phase 49: reads the SystemModel, not the retired record.json)
- **Web UI**: "Launch" page (`LaunchTreeView.js`, `LaunchPanel.js`) with tree view + detail panel
- **API**: `GET /api/launch-tree` returns `{ scopes, node_scopes }` — sourced from the in-memory `LaunchDump` `replay`/`launch` build for the invocation (Phase 47: no `record.json` on disk anywhere). `launch`'s in-memory round-trip passes the real parsed dump, so this stays populated there; standalone `replay --model <path>` (no launch step) has no dump to read and degrades to empty (`{}`), not an error — a documented follow-up is to source it from `model.structure.scopes` instead (`.superpowers/sdd/p46-w5-report.md` §6)
- **Validation**: `scripts/compare_scopes.py` — self-consistency + cross-parser comparison

### Launch IR (feature-gated)

The parser includes an optional IR layer (`--features ir` on the parser crate) that preserves the full launch structure (conditions, substitution expressions, groups, includes) without evaluating. IR types: `src/play_launch_parser/.../ir.rs`. IR tests: `cargo test -p play_launch_parser --features ir`.

### Scheduling Spec

- **Scheduling spec** (the `ros-launch-manifest` repo's `sched/` crate, a git dependency — see Repository Layout): portable scheduling schema shared with nano-ros. **Phase 41 v2 model (current default)**: a `<stem>.system.<target>.yaml` platform file names a `SchedMapper` (`rate_monotonic`, `deadline_monotonic`, or `manual`) that **derives** per-node priorities from launch+contract facts (rates/deadlines/`criticality`); `resources` supplies platform facts (e.g. `rt_priority_band`), `overrides` pins specific nodes (always beats derived). Platform files ship through the same provider-sidecar + user-overlay **channels** as contracts (Phase 40): `--sched <path>` explicit > overlay (`--contracts`/`$PLAY_LAUNCH_CONTRACTS`/XDG/`/etc`) > provider sidecar next to the launch file — auto-applied at `launch`/`run`/`up` when `--sched` is absent. `play_launch check --sched --explain` prints the merged plan with per-node provenance (override/derived/default); `play_launch contract eject <pkg> <file>` copies the resolved provider contract+platform file into the overlay tree for editing (both verbs also exist on the developer-only `ros-launch-resolve` binary — same `ros_launch_resolve::verbs::*` implementation, two thin CLI wrappers). **Legacy bridge**: `system.toml` (hand-written `[tiers.X.<target>]` + `[[assign]]`) still parses via `.toml`-extension dispatch to the `manual` mapper — deprecated but supported until nano-ros migrates (Phase 41.6, not yet scheduled); `--sched-apply off|warn|strict` unchanged (Phase 38: the apply sweep covers every TID, delegated to the CAP_SYS_NICE `play_launch_rt_helper` so `play_launch` itself stays unprivileged). User guide: `docs/guide/rt-scheduling.md`; schema (v1+v2): the manifest repo's `docs/scheduling.md`; design of record: `docs/superpowers/specs/2026-07-16-rt-config-v2-design.md`.
- **Phase 60 — the full Linux policy surface.** The apply layer is
  `sched_setattr(2)`, not `sched_setscheduler(2)`: the latter cannot express
  `SCHED_DEADLINE`, uclamp, or any `sched_flags` value. `libc` has neither the
  function nor `SYS_sched_setattr` on x86_64-gnu, so both, plus the 56-byte
  `SCHED_ATTR_SIZE_VER1` layout, are declared in `src/play_launch/src/sched.rs`
  with a unit test pinning every field offset.
  - **Typed placement** (`PosixSched`/`PosixAffinity`/`PosixPlacement` in the
    manifest crate's `sched/src/posix.rs`) names only the parameters each
    policy has, so a priority on `SCHED_OTHER` or a CPU mask on
    `SCHED_DEADLINE` is unrepresentable. Additive: `sched_class`/`priority`/
    `core` remain for one release. An unknown `sched_class` is now an **error**
    — it used to become `SCHED_OTHER` silently.
  - **Override vocabulary**: `cpus`, `nice`, `uclamp_min`/`uclamp_max`,
    `budget_us`. `reservations: off|required` sits beside `mapper:` (policy,
    not a platform fact); `resources.rr_timeslice_us` is the host's global
    `SCHED_RR` slice.
  - **Cost is authorable, and a deadline is never a cost.** `budget_us` is the
    only source for a reservation's runtime. `resolve_chains` used to fill a
    boundary's `exec_ms` with the path's `max_latency_ms` — its *deadline* —
    which is why no execution time existed anywhere in the model. Absent a
    declared budget the cost is ABSENT, and the existing
    `feasible ON INCOMPLETE EVIDENCE` diagnostic finally fires for real. A
    regression test fails if a deadline is ever read as a cost again.
  - **`SCHED_DEADLINE`** is derived in `ros-launch-resolve`'s `sched_loader`
    (it needs the reservations mode, the budgets, and which nodes are
    containers — none visible to the mapper): runtime from `budget_us`, period
    from `1/rate_hz` propagated along a chain from its source, deadline
    declared-else-period. All-or-nothing within the RT band, scoped to nodes
    carrying a timing fact, **containers exempt** — `--container-mode isolated`
    is the default, so without the exemption nearly every system would
    hard-error. Reservation parameters ride the portable tier head
    (`budget_us`/`deadline_us`/`period_us`) so `up`, which reads the model and
    never the platform file, can rebuild them.
  - **Reservations need a cpuset partition, and play_launch creates nothing.**
    A deadline thread's affinity may not be narrower than its root domain, so
    `taskset` cannot confine one. A partition only validates as a **top-level**
    cgroup, and cgroup v2's common-ancestor rule means nothing can migrate
    into one — a process must be *started* inside.
    `src/play_launch/src/execution/cpuset.rs` is read-only: it detects the
    partition and refuses clearly without one (`play_launch verify` reports
    it). Provisioning is `scripts/provision_rt_cpuset.sh`, run as root.
    **The readback is load-bearing**: writing `partition = root` can succeed
    and read back `root invalid`, and a task there runs on the full root domain
    with no isolation at all.
  - **Two flags that look like hygiene and are not.**
    `SCHED_FLAG_RESET_ON_FORK` is set **only** for `SCHED_DEADLINE` (which the
    kernel refuses to `fork(2)` from without it). On `SCHED_FIFO` it breaks the
    per-TID sweep, because the kernel resets scheduling in `sched_fork()`,
    which runs for *thread* creation too — measured, and locked by a test.
    `uclamp_min` is a **no-op on RT policies** (they already default to
    1024/1024); `check` warns rather than silently doing nothing.
  - **F2 — a reservation is per-thread.** Sweeping one across a ROS node's ~11
    threads would turn an 8ms/100ms budget into 88% of a CPU at admission
    control, so the thread-group leader is reserved and siblings take
    `SCHED_FIFO`. Unsound for multi-threaded executors; refused where
    detectable, documented where not.
  - **Measured result (W8): reservations LOSE on vanilla rclcpp.** Three arms
    on one CPU — RT off 217/1013 missed, `SCHED_FIFO` 9/1030, `SCHED_DEADLINE`
    42/1038; best-effort cost −16% vs −5%. Reservations return most of the
    throughput and give up most of the determinism. The "budgets too small"
    explanation was tested and rejected (larger budgets were *worse*); the
    cause is CBS's sporadic release model versus an `rclcpp::spin()` event
    loop. Report:
    `docs/reports/rt-mixed-criticality/reservations-result.md`. Not yet
    authorable on a v2 path: `deadline_policy` (so `SCHED_FLAG_DL_OVERRUN` is
    wired but always off). Deferred: `SCHED_FLAG_RECLAIM`.

## Installation & Usage

```sh
just build                          # Full build: colcon + bundle + wheel
just build-cpp                      # C++ only (msgs + container)
just build-rust                     # Rust only (assumes C++ install/ exists)
just build-wheel                    # Bundle + wheel only (no colcon rebuild)
just build-interception             # Build interception .so (standalone, not in colcon)
just run launch <pkg> <launch_file> # Run with colcon build
play_launch launch <pkg> <launch>   # Run if installed via pip
play_launch up <model.yaml>         # Spawn from a resolved SystemModel (was `replay` before 0.9.0)
play_launch plot                    # Analysis
```

**Verb surface (0.9.0):** `play_launch` has twelve verbs — `launch`, `run`,
`up`, `resolve`, `dump`, `check`, `plot`, `contract`, `context`, `setcap`,
`verify`, `measure` (Phase 58 W2). RFC-0060 W3 briefly moved `resolve`/`dump`/`check`/`plot`/
`contract` off it onto `ros-launch-resolve`; that was reverted (Phase 56
amendment, D2/D5) because it left NO `play_launch` verb able to write the
`system_model.yaml` that `up` requires, and pointed users at a binary they
do not have.

**`ros-launch-resolve` is a developer/integration binary.** It exists so
nano-ros and other consumers can resolve launch trees without linking a ROS
runtime. It is NOT shipped in the wheel and must NEVER appear in `README.md`,
`docs/guide/`, or any error message a user can trigger — `pip install
play_launch` is the whole product as far as a user is concerned. Both CLIs are
thin wrappers over `ros_launch_resolve::verbs::*`; put shared logic there,
never in a CLI.

`replay` is the one removed verb: a hidden clap variant that errors naming
`up` — see `src/play_launch/src/commands/migrated.rs`. **Delete that module
at 1.0.0.**

## Architecture

**Execution Flow:** Load the SystemModel (`system_model.yaml`, from `resolve`/`dump`, or built in-memory by `launch`) → classify nodes → spawn async tasks → actor-based lifecycle → logs to `play_log/<timestamp>/`. `up` (renamed from `replay` in 0.9.0, see `docs/guide/cli-migration-0.9.md`) requires a SystemModel (positional `<model.yaml>` or `--model <path>`) — Phase 47 hard-removed the legacy `--input-file record.json` compat path (and `dump --format record`/`--format` entirely); `record.json` is no longer a user-facing artifact anywhere. `LaunchDump` (the parser's JSON-serializable record shape) still exists as an in-memory intermediate `resolve`/`launch` build the model from — it just never touches disk on the primary paths anymore.

**Key Concepts:**
- **Async/Tokio**: All background services run as async tasks
- **Actor Pattern**: Self-contained lifecycle management with respawn support
- **Virtual Members**: Composable nodes managed by container actors (LoadNode/Unload)

### Container Architecture

Custom C++ container (`src/play_launch_container/`) replaces stock `rclcpp_components`:

```
rclcpp_components::ComponentManager           (upstream)
  └── ObservableComponentManager              (ours: event publishing)
        └── CloneIsolatedComponentManager     (ours: fork+exec per-node isolation)
```

- **ObservableComponentManager**: publishes `ComponentEvent` on `~/_container/component_events` (reliable, transient_local, depth 100)
- **CloneIsolatedComponentManager**: fork()+exec() of `component_node` binary per composable node
  - Parameters: `request->parameters` serialized to temp YAML (`/**:` wildcard namespace), passed via `--params-file`
  - `extra_arguments`: only `use_intra_process_comms` is extracted and forwarded as `--use-intra-process-comms`
  - Ready pipe protocol: child writes `"OK name\n"` or `"ERR msg\n"`. The wait
    covers the node's CONSTRUCTOR, so it is bounded by **LIVENESS, not time**:
    poll in 1 s slices, `waitpid(WNOHANG)` each slice, give up the moment the
    child dies. There is no right fixed number — a first-run TensorRT engine
    build takes however long that board's GPU takes (~33 s cold, ~45 s even
    cached, measured on one Orin) — and a wrong one SIGKILLs a node partway
    through normal work, discards it (the `.engine` is written last) and repeats
    every launch. `PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS` (0 = default = no
    deadline) exists only to bound a node that WEDGES rather than works.
    Safe only because the wait now REPORTS: the container logs
    `constructing for Ns (pid N, alive)` every 15 s, and play_launch logs
    `N composable(s) still constructing; longest '…' at Ns` per container every
    30 s. No new message type was needed — the container waits *while alive*, so
    "still Loading" already carries the liveness claim. Issue #0019, and the
    explanation for `motion_velocity_planner` in #0017.
- **Spawn admission is governed by MEMORY, not by a worker count.** The pool
  (`PLAY_LAUNCH_SPAWN_WORKERS`, default `clamp(nproc, 4, 32)`) is sized not to
  be the limit; `await_spawn_capacity()` holds a fork while `MemAvailable` is
  under a floor (1 GiB, capped at RAM/4, `PLAY_LAUNCH_SPAWN_MIN_AVAIL_MB`).
  A fixed count cannot tell a component that is slow because it is COMPUTING
  (a TensorRT build — worth throttling) from one slow because it is WAITING
  (costs nothing, yet held a slot). Measured: the old `kWorkerThreadCount = 4`
  turned six 20 s constructors into two waves (t+20s x4, t+40s x2); now they
  are one. Matters because `perception_preset:=camera_lidar_fusion` puts SIX
  TensorRT components in one container. Admission is mutex-serialised (else
  every worker reads the same `MemAvailable` and all fork at once) and bypasses
  after 120 s with a warning. Design: `docs/design/composable-load-admission.md`.
- **The global load cap is mode-aware.**
  `composable_node_loading.max_concurrent_load_node_spawn` protects containers
  that handle LoadNode ON THEIR EXECUTOR (`stock`, `observable` — see
  `tests/fixtures/sequential_loading`: "Container serializes all LoadNode
  requests"). It is DISABLED for `isolated`, which answers immediately with a
  pre-assigned id and paces its own spawns — there a cap on in-flight calls
  bounds nothing that costs anything.
- **A composable is `Loaded` only when ListNodes confirms it.** The LoadNode
  response means "spawn requested" — the container pre-assigns a unique_id and
  returns before the child is ready — so a missing `ComponentEvent` is NOT
  evidence of success. `Unavailable` (container executor too busy to answer) is
  the normal state *while* a composable constructs and must never be read as
  loaded; that is what reported a killed node as `84/84 loaded` (#0019).
- **Two binaries**: `component_container [--use_multi_threaded_executor] [--isolated]` and `component_node` (standalone single-node loader)
- **Design docs**: `docs/design/container-isolation.md`, `docs/archive/clone-vm-container-design.md`
- **Roadmap**: `docs/roadmap/phase-19-isolated_container.md`

## Configuration

All features **enabled by default**: monitoring, diagnostics, web UI (http://127.0.0.1:8080).

Key flags: `--config <PATH>`, `--disable-monitoring`, `--disable-diagnostics`, `--disable-web-ui`, `--disable-all`, `--web-addr <IP:PORT>`, `--disable-respawn`, `--enable <FEATURE>`, `--container-mode <MODE>`.

**Container mode** (`--container-mode`, default `isolated`):
- `observable` — override all containers to use `play_launch_container` with `ComponentEvent` publishing
- `isolated` — use `play_launch_container` with `--isolated` (fork+exec per-node process isolation)
- `stock` — use the original container binary from the launch file (no override, no `ComponentEvent` subscription)

When mode is not `stock`, `prepare_container_contexts()` in `src/execution/context.rs` overrides the container's `package` and `executable`. If the original was `component_container_mt`, `--use_multi_threaded_executor` is prepended to args. Original `args` from the launch file (e.g. `--isolated`) are preserved.

See `tests/fixtures/autoware/autoware_config.yaml` for an example config (not exhaustive — full field reference: `src/play_launch/src/cli/config.rs`).

**Interception** (`interception` config section, default: disabled):
- `enabled: true` — inject `LD_PRELOAD` interception .so into all child processes
- `frontier: true` — enable per-topic timestamp frontier tracking
- `stats: true` — enable per-topic message count/rate statistics
- `ring_capacity: 65536` — SPSC ring buffer capacity per child process
- `events: true` — write `interception/events.jsonl`, the per-message record
  `play_launch measure` reads (Phase 58 W2). Like `trace` it is one line per
  publish/take (~110 bytes), but unlike `trace` it defaults ON: its value is
  being there *after* a run you then decide to measure, so an opt-in flag
  would be discovered exactly when it is too late. Turn it off for long runs
  on high-rate systems.

When enabled, play_launch creates a shared memory ring buffer per child, injects `LD_PRELOAD` + fd env vars, and spawns a consumer tokio task that writes `frontier_summary.json` and `stats_summary.json` to `play_log/<ts>/interception/` on shutdown.

### RCL Interception Architecture

`libplay_launch_interception.so` (LD_PRELOAD) hooks `rcl_publish`/`rcl_take` to extract `header.stamp` from messages. Compiled-in plugins (`FrontierPlugin`, `StatsPlugin`) write `InterceptionEvent` to a zero-copy SPSC ring buffer shared with play_launch.

Standalone crates (not in colcon workspace):
- `src/vendor/rcl_interception_sys/` — bindgen FFI types for rcl/rosidl
- `src/play_launch_interception/` — cdylib: hooks, registry, introspection, plugins
- `src/spsc_shm/` — generic SPSC ring buffer over shared memory (memfd + eventfd)

Env vars (set by play_launch, consumed by the .so):
- `PLAY_LAUNCH_INTERCEPTION_SHM_FD` — shared memory fd for ring buffer
- `PLAY_LAUNCH_INTERCEPTION_EVENT_FD` — eventfd for wakeup signaling
- `PLAY_LAUNCH_INTERCEPTION_SO` — override path to the .so (for tests)

Build: `just build-interception` (requires ROS environment). Design: `docs/roadmap/phase-29-rcl_interception.md`.

## Log Directory Structure

```
play_log/<timestamp>/
├── params_files/
├── interception/                   # when interception enabled
│   ├── frontier_summary.json       # per-topic frontier state
│   ├── stats_summary.json          # per-topic pub/take counts, rates
│   ├── events.jsonl                # per-message record read by `measure`
│   └── node_identity.tsv           # model key → PID → REAL ROS node name (#0017)
├── system_stats.csv, diagnostics.csv
└── node/<node_name>/{metadata.json, metrics.csv, out, err, pid, status, cmdline}
```

Composable nodes don't have separate directories — metadata in parent container's `metadata.json`.

## Development Practices

### Building & Testing
- **ALWAYS** use `just build` (never `colcon build` directly)
- **ALWAYS** use Bash tool's `timeout` parameter (never `timeout` command prefix)
- Temp files in `tmp/` (gitignored), never `/tmp`. Avoid writing large shell scripts in Bash tool — create files using Write/Edit tool instead of `cat` + EOF heredoc.
- **External source exploration**: download source repos to `external/` (gitignored) for deep exploration. Use `gh api` only for quick peeks or GitHub-specific features (issues, PRs, actions).
- **Standalone crates** (outside the workspace, with their own `[workspace]` in Cargo.toml) must have a `/target/` `.gitignore` — e.g. `play_launch_interception`, `rcl_interception_sys`, `spsc_shm`

### Repository Layout (phase-55 W1)

`src/ros-launch-resolve/` (RFC-0060 layer 2: the resolver + `parser/`) lives
**in this repository as plain directories** — it used to be a submodule, and
the parser a submodule of that. Both were folded in by `git subtree` with
history; nano-ros pins this repo and builds
`cargo build -p ros-launch-resolve-cli` from the nested workspace.

It is still a **separate Cargo workspace**, listed in the root manifest's
`exclude`. That line is now the only thing keeping `[patch.crates-io]`
(`rclrs`, `rosidl_runtime_rs`) out of the resolver's dependency graph — the
submodule boundary that used to make the separation visible is gone. Never
add `src/ros-launch-resolve` to `members`. `just check-layer2-isolation`
(part of `just check`, plus a CI job on a runner with no ROS) fails if the
boundary breaks: no ROS crates in the graph, no ROS shared libraries linked,
and both launch frontends resolve with no ROS installed.

**`ros-launch-manifest` is a git dependency pinned by tag** (`v0.1.5`), not a
submodule — phase-55 W2. Layer 2 now has no submodules at all. **Three**
manifests name the tag and must move together:
`src/play_launch/Cargo.toml`, `src/ros-launch-resolve/Cargo.toml` and
`tests/Cargo.toml`. Naming the same tag is what makes cargo resolve ONE
instance; different revisions would give two same-named packages from
different sources, and `SystemModel` would become two incompatible types.

`tests/` is the one that drifts unnoticed — it sat at `v0.1.0` for five tags
because it is a separate workspace that uses the crate for one call
(`fixture_dir()`) and drives `play_launch` as a subprocess, so no types cross
the boundary and nothing fails to compile. The symptom is not an error but
silent staleness. **Bump all three, and commit the lockfiles**: a clean clone
builds from the lock, and `--locked` fails against the old revision.

There is no vendored copy in-tree any more. `src/ros-launch-resolve/
third-party/ros-launch-manifest/` was deleted (phase 60) — nothing built from
it, it had drifted from the compiled tag, and it was consulted during design
work as if it were authoritative. Read the real thing at the pinned tag, or in
`~/.cargo/git/checkouts/` where cargo already keeps it.

Its `tests/fixtures` are reached through
`ros_launch_manifest_check::fixture_dir()` (the crate's `testdata` feature),
never a relative path — a git dependency is checked out under
`~/.cargo/git/checkouts/` where no consumer can name it.

Building layer 2 in-tree inherits this repo's colcon-generated
`.cargo/config.toml` (cargo's config walk ignores workspace boundaries), which
shows up as harmless `[[patch.unused]]` churn in
`src/ros-launch-resolve/Cargo.lock`. Discard it; run the gate with
`--standalone` for a clean-clone-equivalent check.

### Process Management
- **NEVER** `kill -9` individual processes — kill the process group (PGID):
  ```bash
  PGID=$(ps -o pgid= -p $PID | tr -d ' '); kill -TERM -$PGID; sleep 2; kill -9 -$PGID
  ```
- Use `just kill-orphans` to clean up stray ROS processes

### Logging
- `error!`: unrecoverable | `warn!`: recoverable | `info!`: user-facing | `debug!`: technical
- Default `RUST_LOG=play_launch=info`. Enable debug: `RUST_LOG=play_launch=debug`
- **`info!` is only for end-users** — never promote `debug!` to `info!` just to make tests pass. Use `RUST_LOG` to control log visibility in tests instead (e.g. `cmd.env("RUST_LOG", "play_launch=debug")`)

## PyO3 and ROS2 Gotchas

- **Module functions**: use `module.getattr("fn")?.call0()?` (not `call_method0`)
- **launch_ros properties**: `target_container.expanded_node_namespace` (not a method call)
- **Node names**: check for leading `/` before prepending namespace to avoid `//`
- **SomeSubstitutionsType params**: accept `PyObject`, convert via `utils::pyobject_to_string()` — handles strings, substitutions, and lists
- **YAML substitutions**: always use `load_and_resolve_param_file()` (not raw `read_to_string`) to resolve `$(var ...)` and convert types

## Testing

```bash
just test              # Parser unit (371) + scope (9) + fast integration (6), ~3s
just test-all          # Parser unit (371) + scope (9) + all integration (42), ~70s
just test-unit         # Parser unit tests only
just test-integration  # All integration tests (simple + Autoware)
cargo test -p play_launch_parser --features ir  # IR tests (42 tests, not included in default)
just compare-scopes <pkg> <launch> [args...]    # Cross-parser scope comparison
play_launch context system_model.yaml --tree    # Launch tree inspection
```

Two crates: parser unit tests (`src/play_launch_parser/`) and integration tests (`tests/`, excluded from workspace). Integration tests use `ManagedProcess` RAII guard (`tests/src/process.rs`) for guaranteed cleanup via `setsid()` + `PR_SET_PDEATHSIG` + PGID kill on Drop.

**DDS isolation**: `play_launch_cmd()` in `tests/src/fixtures.rs` assigns a unique `ROS_DOMAIN_ID` per invocation (PID + counter) so concurrent nextest processes don't cross-talk over DDS.

Test workspaces: `tests/fixtures/{autoware,simple_test,sequential_loading,concurrent_loading,container_events,parallel_loading,rt_workspace}/` — 4 of them (`autoware`, `simple_test`, `container_events`, `rt_workspace`) have a `just compare-dumps` recipe (model-based parser parity, self-contained — resolves both parsers itself; Phase 47.B5 removed the `dump-rust`/`dump-python`/`dump-both`/`compare-dumps-record` record.json-based recipes). `rt_workspace` is a real colcon workspace (`rt_demo` package) exercising RT scheduling + contract shipping; tests in `tests/tests/rt_workspace.rs` (excluded from `just test`, run by `just test-all`). **`just test-all` now builds `rt_workspace` and `io_stress` itself**, because a guarded test that skips still reports as PASSED — 27 of 108 integration tests were silently skipping on unbuilt fixtures, concealing 4 real failures. `test-all` also prints a "Silently-skipped tests" summary so a guard that starts always-skipping is visible rather than green.

## Key Recent Changes

- **2026-08-16**: Phase 61 W1 — **the edge startup storm, and the fix that
  wasn't.** A 144-process Autoware launch on a 12-core AGX Orin put **484 tasks
  in the runnable queue at load1 203** and held ~10 of 12 cores for 49 s; on the
  reporter's vehicle the OOM killer took their GNOME session. Reproduced on
  bench hardware (same 64 GiB Orin), with one honest gap: the *memory* storm
  needs sensors to reproduce, so the link from CPU storm to OOM (starved nodes,
  DDS/callback queues absorbing sensor backlog) is the one inference rather than
  a measurement.
  The obvious fix was implemented and **measured wrong**: pacing spawns to one
  per core doubled a 10.6 s startup for a ~10% cut in peak runnable tasks,
  because a wide launch costs what it costs by having those processes *exist*,
  not by starting them together — serialising just stretches the same CPU. An
  earlier variant was worse than nothing: a `2 * ncpu` runnable-task ceiling
  cannot tell "busy starting things" from "busy running the things we started",
  so already-running nodes held the gate shut until its 30 s bypass fired each
  time (startup unfinished at 190 s, peak load1 **217 vs the unpaced 183**).
  What actually moves it is the **process count**. `--container-mode isolated`
  (the default) forks a process per composable: 144 processes vs 60 under
  `observable`, **10.2 of 12 cores vs 3.9** during startup, peak load1 190 vs
  **45**, 3.5 GiB vs **1.4**, and `observable` finishes *faster* (8.4 s vs
  10.7 s) with the same 84/84 loaded. **That is not a recommendation**: the cost
  is the documented price of fault isolation
  (`docs/design/container-isolation.md`). A SIGSEGV cannot be contained inside a
  process — one segfaulting composable takes down every node sharing its
  container, 3–10 in Autoware — and `oom_score_adj`, `memory` cgroups,
  `kill(pid)` and per-node restart are all process-granular, so the cheaper mode
  is the one that makes this phase's own OOM bias per-*container* instead of
  per-*node*; `observable` also gives up zero-copy IPC (~1–5 ms per pipeline
  stage). Container mode is a **safety decision, not a performance one**:
  `observable` on a bench, `isolated` on a vehicle. The real lever for a vehicle
  is per-node isolation granularity, which play_launch does not have (W3).
  So: both throughput gates ship **off**
  (with the tables recorded at the fields they govern, so the defaults aren't
  re-derived); the `MemAvailable` floor ships **on** because it never blocks
  until memory is actually short; children get `oom_score_adj +300`
  (`PLAY_LAUNCH_OOM_SCORE_ADJ` to override) so the kernel picks a node over the
  desktop — play_launch can only volunteer its own children, since *lowering*
  another process's score needs privilege it lacks; and `isolated` reports its
  cost when it would fork more than `4 * ncpu` processes. The memory floor is
  **absolute (1 GiB, capped at RAM/4)**, not a share of RAM: what it guards
  against is one more process allocating before the next sample, and the largest
  launch-owned process measured 274 MiB regardless of machine size — an earlier
  10%-of-RAM version gave a 64 GiB box 4 GiB it did not need and a 4 GiB board
  512 MiB, less than two of the processes it was protecting against. Two defects found on the way:
  `max_concurrent_load_node_spawn` was **dead config** (`dispatch_pending_loads`
  drained its queue ungated, so all 84 composables hit LoadNode at once) — now
  real, with `delay_load_node_millis` removed rather than resurrected; and
  `Startup complete` fired **before anything had started** (issue #0016 — the
  test asked "is nothing in flight?" when it meant "is everything done?", true
  at t=0, and the progress task exits on completion so it also silenced every
  message that would have corrected it). Separately, 0.9.0 **could not parse the
  reporter's launch file at all**: `<choice>` is a child entity of `<arg>` in
  real ROS 2 (`declare_launch_argument.py:176`), but `attr_spec.rs` gave `arg`
  no children, so the YAML frontend read it as an unknown attribute — a
  regression against the 0.5.1 they run. Guide:
  `docs/guide/edge-machines.md`. Roadmap:
  `docs/roadmap/phase-61-edge-startup-storm.md`.
  **W2 — staged startup** (`execution/startup_order.rs`): `startup.order` glob
  groups (and `defer_sources` from the topic graph) hold sensor drivers until
  their consumers are up, so nothing publishes into subscribers that do not
  exist yet. A stage waits for members to appear in the **ROS graph**, not for
  `spawn()` — which returns long before subscriptions exist. Correction to W1's
  note: the SystemModel does NOT carry the chain data to derive this for a plain
  launch (`structure.topics` comes from manifests; the golf cart resolves to 0
  topics), so explicit config is the working path. Surfaced a naming gap: **19
  of 144 model FQNs never appear in the ROS graph** — a node the launch did not
  name is keyed by its EXECUTABLE while it registers its compiled-in name
  (`…/autoware_ekf_localizer_node` vs `…/ekf_localizer`), because play_launch
  emits `__ns` but no `__node` remap. `node_name.is_none()` discriminates it
  exactly; the gate falls back to process-running for those. Off by default.
- **2026-08-13**: Phase 58 W2 — **cost is measured, not asked for.** New verb
  `play_launch measure <run-dir> --model <m.yaml>` turns a recorded run into a
  platform-file `overrides:` fragment on stdout (never written back). The
  roadmap sketch for this wave had a defect worth naming: it proposed
  measuring take→publish *elapsed* time and calling it cost. That is RESPONSE
  time — it carries preemption, blocking and DDS wakeup — while `budget_us`
  becomes a CBS **runtime**, which is CPU time; and the sketch's own
  mitigation ("measure on an idle system") does not fix it, because even idle
  that figure includes wakeup latency. So W2 measures **both** and keeps them
  apart: `InterceptionEvent` grew 40 → **56 bytes** with `cpu_ns`
  (`CLOCK_THREAD_CPUTIME_ID`, read in the two hooks that already existed) and
  `tid`. The tid is load-bearing, not bookkeeping — `cpu_ns` is per-thread, so
  a take and publish on different threads would still subtract to a
  plausible-looking number; such pairs are rejected, as are any where
  `cost > response` (a thread cannot consume more CPU than the wall time it
  spans). `budget_us` takes the observed **MAXIMUM**, p50/p99 as comments:
  under CBS an overrun is throttled to the next replenishment, so a p99 budget
  converts the slowest 1% of invocations into a full-period stall. Paths that
  cannot be measured are **printed with their reason** (timer-triggered,
  unstamped, not exercised) — omitting them would read as "costs nothing",
  the same absent-versus-zero confusion Phase 60 removed from the chain
  checker. Hot-path overhead was measured rather than assumed:
  `CLOCK_THREAD_CPUTIME_ID` is not in the vDSO, but costs **85 ns/call vs
  17 ns** for `CLOCK_MONOTONIC` — ~150 ns per message, 0.15% of a core at
  10k msg/s. New artifact `play_log/<ts>/interception/events.jsonl` (streamed,
  so a killed run still leaves a usable file; `interception.events` to
  disable). Also fixed on the way: `model_builder` lowered
  `PathContract.input` from the LEGACY `input:` field, so every path written
  in Vocabulary v2 (`trigger: { input: [...] }`) reached the model looking
  periodic — it now reads `effective_trigger()`. Validated against ground
  truth: `just measure` in `examples/rt_av_demo/` (nodes busy-burn exactly
  `burn_ms`, so the fixture is an oracle) — detect 8.0 declared → **8.08 ms**
  measured, brake 3.0 → **3.05 ms**, lidar's timer path reported as
  unmeasurable. Brake's *response* max on the same invocations was **40.43 ms**
  against 3.05 ms of CPU, which is the whole argument for measuring both: a
  design that measured elapsed time would have declared a 40 ms reservation
  for a 3 ms callback. One correction the first real run forced: the
  `cost > response` sanity check needs a tolerance, because on a CPU-bound
  callback the two deltas measure the same interval and clock-read skew puts
  cost a few hundred ns over (measured 621 ns max) — rejecting those discarded
  the expensive tail the budget comes from. Design:
  `docs/superpowers/specs/2026-08-13-measured-cost-design.md`. Roadmap:
  `docs/roadmap/phase-58-scheduling-derivation.md` §W2.
- **2026-08-12**: Phase 60 complete — the Linux scheduling realizer beyond
  `SCHED_FIFO`. Details in [Scheduling Spec](#scheduling-spec); the headline is
  that the apply layer moved to `sched_setattr(2)`, cost became authorable
  (`budget_us`) and the deadline-as-cost conflation was deleted,
  `SCHED_DEADLINE` is derived and applied, and **W8 measured that reservations
  lose to fixed priority on vanilla `rclcpp`** (217 → 9 misses under
  `SCHED_FIFO`, 217 → 42 under `SCHED_DEADLINE`) — reported rather than tuned
  away. Reproduce with `just ab` (two arms, unprivileged) or `sudo -E just ab3`
  (adds the deadline arm; needs root for the partition) in
  `examples/rt_av_demo/`. Manifest crate bumped `v0.1.4` → **`v0.1.5`** across
  all three manifests. Four times the design was wrong and measurement corrected it,
  which is the pattern worth carrying forward:
  `SCHED_FLAG_RESET_ON_FORK` on `SCHED_FIFO` silently breaks the per-TID sweep
  (`sched_fork()` runs for *thread* creation); a cpuset partition validates
  only as a **top-level** cgroup and cannot be migrated into; the model carried
  `SCHED_DEADLINE` with no runtime or period, so `up` would have refused every
  reservation it derived; and a declared `budget_us` failed to reach the chain
  because cost lookup keyed by FQN while overrides key by bare name. Roadmap:
  `docs/roadmap/phase-60-linux-sched-surface.md`. Design:
  `docs/superpowers/specs/2026-08-10-linux-sched-feature-surface-design.md`.
  Result: `docs/reports/rt-mixed-criticality/reservations-result.md`.
- **2026-08-11**: `just setcap` uses a throwaway container instead of `sudo`,
  so the edit-build-test loop needs no password. Every colcon build COPIES the
  helpers into `install/` (differing inodes, `nlink 1`) and a capability lives
  on the inode, so they are dropped on every build — and nothing can preserve
  them: the kernel's `killpriv` path strips `security.capability` on write, and
  copying it forward needs `CAP_SETFCAP`. Docker's DEFAULT bounding set already
  holds `CAP_SETFCAP`, granting a file capability needs only that (not the
  capability being granted), and a bind mount writes the xattr to the host
  inode. First run builds the image (~12s), then ~0.4s. `just build` reapplies
  automatically when it can do so without prompting. **Rootless Docker is
  refused, not silently used**: in a user namespace the xattr is namespaced
  (revision 3, carrying a rootid) and honored only inside it, so `getcap` would
  read correct while host processes still got `EPERM`. This is convenience via
  privilege you already hold — `docker` group membership is root-equivalent —
  so it is a DEVELOPER shortcut; the user-facing install path is still
  `sudo setcap`. Image: `docker/setcap.Dockerfile`.
- **2026-08-06**: Phase 58 planned — deriving scheduling from contracts.
  **Superseded in part**: W1 (make cost authorable) and W4 (reservations)
  moved to Phase 60 and are DONE. Finding (1) is fixed (cost is authorable);
  finding (4) was correct and is resolved by requiring a cpuset partition. W2
  (measure cost), W3 (deadline decomposition) and W5 (synthesis) remain.
  Study: `docs/research/scheduling-derivation-prior-art.md` (options A–G:
  Audsley OPA, deadline decomposition, reservations, LET, CP synthesis,
  compositional analysis, callback granularity). Four findings from the
  current code, all blocking or shaping that work: (1) **contracts declare
  budgets, never costs** — `sched_derive.rs` passes a path's
  `max_latency_ms` in as `exec_ms`, so no true execution time exists in the
  model and response-time analysis / reservation sizing / OPA are all
  unreachable; (2) `ResolvedChain` is sequences all the way down, so a
  diamond linearises correctly for *ranking* and wrongly for *costing*
  (fork-join needs max-over-branches, feasibility sums); (3) the mapper
  design's fan-in rule ("longest-path-to-sink") has never had an input that
  would distinguish it from declaration order, because `push_segment_node`
  merges in declaration order and an authored chain never presents a fan-in;
  (4) **`sched_setattr(2)` returns EPERM when a thread's affinity mask does
  not include all CPUs** — our per-node affinity and `rt_av_demo`'s `taskset`
  are exactly that condition, so `SCHED_DEADLINE` and the current
  partitioning are mutually exclusive (deadline tasks need exclusive
  cpusets). Roadmap: `docs/roadmap/phase-58-scheduling-derivation.md`.
- **2026-08-05**: Phase 57 — mixed-criticality RT demo + Chrome timeline
  export. `examples/rt_av_demo/` is a standalone ROS 2 workspace (plain
  `rclcpp`, standard launch XML) whose lidar→detector→brake chain carries a
  60 ms deadline; `just ab` runs it twice (`--sched-apply off`, then
  `strict`) and compares. Measured: **272/1013 frames past deadline → 0/1039**,
  p99 106.6 → 13.3 ms, at a **26–37% cost to best-effort throughput**.
  Calibration changed two design assumptions: two CPUs cannot manufacture
  contention on a 32-thread host (CFS sleeper fairness serves a low-duty
  chain promptly while a spare CPU exists — zero misses at every load tried),
  so the workload is confined to ONE CPU; and the comparison must judge
  *steady state*, since discovery makes the first ~4 s miss by 150 ms
  regardless of scheduling. `just ab` refuses rather than reports when the
  baseline meets its deadline or when the helper lacks `CAP_SYS_NICE`.
  New: `interception.trace` config → `play_log/<ts>/interception/trace.json`
  in Chrome Trace Event format (rows = nodes, flow arrows = messages,
  correlated by `header.stamp`). Report:
  `docs/reports/rt-mixed-criticality/` (Typst; `just report` regenerates
  every figure and number from the run — no hand-typed values).
  Four scheduling defects fixed along the way: `band_violations` compared
  `SCHED_OTHER` tiers against the RT band and "clamped" them into it, so
  `check --explain` printed `SCHED_OTHER 10` (a state Linux cannot represent,
  and fatal in strict mode — it would have rejected valid platform files);
  demotion left a stale mapper-derived priority; `class: real_time` shipped
  on best-effort tiers in `system_model.yaml`; and **application was never
  logged**, so both halves of an A/B produced byte-identical scheduling logs
  and `chrt -p` was the only way to tell RT had applied. Note issue #0015
  (largely defanged 2026-08-11 — `just setcap` no longer prompts and `just
  build` reapplies for you, but the underlying inode fact is unchanged):
  any rebuild replaces `play_launch_rt_helper` and silently drops its
  capability — `just setcap` after every build.
- **2026-07-31**: Removed `<node machine=>` — it is ROS 1 roslaunch syntax,
  not ROS 2 (`launch_ros`'s `Node.parse()` has no such attribute;
  `launch_xml` rejects it; `ros2/design` #255 closed unmerged). The Rust
  parser accepted it while the Python parser rejected it, so the multihost
  fixture could not be loaded by `ros2 launch`. `Deploy.host` is gone from
  the model with it, and a launch-only resolve now produces no
  `execution.deploy` entries. Multi-host launches use a standard `<arg>` +
  `if=` condition and partition at resolve time
  (`play_launch resolve <launch> host:=robot1`) — see
  `docs/guide/multi-host.md`. The audit behind this fix found three more
  places the Rust parser accepted XML that stock ROS 2 rejects: `<group
  namespace=>`/`<group ns=>` (ROS 2 requires a `<push-ros-namespace>` child
  instead), `<push-ros-namespace ns=>` (ROS 2 accepts only `namespace=`),
  and a top-level `<arg value=>` (ROS 2 allows `value` only on an `<arg>`
  nested inside `<include>`). The first two are long-standing back-compat
  aliases this parser deliberately still reads, so — by human ruling, to
  keep existing launch files working — they stay WARNINGS rather than
  errors; the plain `machine=` case and the top-level `<arg value=>` case
  (a known attribute used in a context ROS 2 doesn't allow) are both hard
  errors, same as any genuinely unknown attribute. The parser now carries
  per-element attribute
  allowlists (`src/ros-launch-resolve/parser/crates/play_launch_parser/src/
  xml/attr_spec.rs`) that error on unknown attributes and warn on
  known-unsupported ones — including six `<node>` attributes ROS 2 accepts
  that this parser still doesn't implement (`exec_name`, `ros_args`,
  `launch-prefix`, `cwd`, `emulate_tty`, `shell`) — on both the XML and YAML
  frontends, guarded by a differential test against real ROS 2. Spec:
  `docs/superpowers/specs/2026-07-31-machine-attr-removal-design.md`.
- **2026-07-20**: Phase 47 (Wave B, 47.B2–B6) — hard removal of `record.json`
  as a user-facing artifact. `dump` no longer has `--format`/`DumpFormat` —
  it always emits the SystemModel; `dump run` (no SystemModel form) is
  removed (`play_launch run` covers single-node dump+replay-in-one).
  `replay` requires a SystemModel (positional `<model.yaml>` or `--model
  <path>`, mutually exclusive) — the deprecated `--input-file record.json`
  path is gone; `replay --input-file ...` now fails clap parsing
  (`error: unexpected argument`), not a silent misparse. `resolve --record
  <path>` (record-reuse mode) is removed along with it —
  `package_or_path` is a required positional now. `launch`'s internal
  round-trip is fully in-memory: parses straight into a `LaunchDump` (Rust
  parser: JSON round-trip in memory, no disk; Python parser: a private
  OS-temp scratch file, deleted before returning — not a `record.json` left
  for the user), builds the SystemModel via the same `resolve` pipeline
  (`commands::resolve::build_checked_model`), and calls the shared
  `commands::replay::play()` engine directly — no second CLI invocation, no
  file round-trip. `play()` now takes an owned `LaunchDump` + `Arc
  <SystemModel>` instead of a record.json path + optional model; standalone
  `replay` passes an empty dump (best-effort consumers — chain-colocation
  warnings, the web UI launch-tree scope map — degrade to empty, same
  documented limitation as before), `launch` passes the real one it just
  parsed (those consumers stay populated there). The container/composable
  record-path spawn builders (`prepare_container_contexts`/
  `prepare_composable_node_contexts`) and their static-equivalence gate
  (`execution::spawn_equivalence_test`) were DELETED (review fix) once the
  record.json artifact retirement left them with no reachable caller — the
  gate had become a vacuous skip (it needed a record.json it could no
  longer produce). `prepare_node_contexts` (the plain-node record-path
  builder) stays LIVE — `run` (single-node dump+replay) still uses it.
  Retired
  `scripts/compare_records.py` + `scripts/compare_parsers.sh`
  (`scripts/compare_models.py` from Phase 47.B1 is the parity tool now);
  fixture justfiles (`autoware`, `rt_workspace`, `simple_test`,
  `container_events`) dropped their `dump-rust`/`dump-python`/`dump-both`/
  `compare-dumps-record` recipes, keeping only the model-based
  `compare-dumps`. `scripts/count_processes.py` now reads a SystemModel
  YAML (`.yaml`/`.yml`) as primary input, with a legacy `.json` (record)
  fallback for old files. Migrated the 9 integration test files that
  dumped/compared via record.json (`autoware`, `simple_workspace`,
  `container_events`, `sequential_loading`, `concurrent_loading`,
  `mixed_loading`, `io_stress`, `parallel_loading`, `sched_apply`) onto
  `resolve`/model comparison; `rt_workspace.rs`'s deprecation-warning test
  was replaced with two hard-cut tests (`--input-file` rejected, no-model
  errors clearly). Residual record.json reader: `play_launch context
  record.json` — the CLI's last consumer, kept as a dev tool for old/
  hand-produced record.json files (the model's `structure.scopes` lacks
  the `ns`/`args`/file granularity `context` needs; migrating it is a
  data-model expansion out of scope for this wave). Design:
  `docs/roadmap/phase-47-cli_and_record_hard_removal.md` §B. Report:
  `.superpowers/sdd/p47-wB-report.md`.
- **2026-07-20**: Phase 46 (Unified SystemModel) complete — the SystemModel
  (`system_model.yaml`) is now the one user-facing artifact. `dump <launch>
  -o m.yaml` (default) and `resolve` emit the same complete model
  (structure+contracts+sched); `replay --model m.yaml`/`replay <model.yaml>`
  spawns from it directly, no `record.json` companion required. Both
  parsers produce the SAME complete model — contracts/sched apply on the
  shared scope table (`ScopeOrigin.path`, Phase 40.1) independent of parser
  (`resolve --parser {rust,python}`); a stale pre-40.1 Python install now
  fails loud instead of silently degrading. `record.json` is DEPRECATED
  compat/dev surface: `dump --format record` (parser-parity tooling,
  `just compare-dumps`/`scripts/compare_records.py`) and `replay
  --input-file record.json` (warns, still spawns, one-release grace) — no
  hard removal yet. `meta.record` binding + `verify_model_record_binding`
  removed. Design: `docs/design/unified-system-model.md`. Roadmap:
  `docs/roadmap/phase-46-unified_system_model.md`.
- **2026-03-19**: Phase 30 (Launch Tree Scoping) complete. Scope table in `record.json` tracks which launch file each node comes from. Both Rust and Python parsers produce matching scope tables (83 scopes, 119 entities for Autoware). New types: `ScopeEntry`, `ScopeTable` in `record/types.rs`; `scope: Option<usize>` on all record types. New CLI: `play_launch context record.json --tree|--node|--launch`. New web UI: "Launch" page with tree view, expand/collapse, per-node detail panel with logs/params/topics tabs. Cross-parser validation: `scripts/compare_scopes.py`. New API: `GET /api/launch-tree`.
- **2026-03-12**: Nodes with `name=None` in launch files use `exec_name` as the FQN map key and log-directory name (`node_cmdline.rs`, `builder.rs`). **They do NOT get a `__node` remap** — forcing one was bug `af7c524`, which renamed such nodes away from their compiled-in default and broke LifecycleNode service discovery; stock `launch_ros` emits `__node` only when the launch file declares a name (`node.py:493`). Consequence: for these nodes the model FQN is NOT the ROS graph name — issue #0017, **resolved 2026-08-18**: the rule for whether a key is a ROS name lives in ONE place (`ros_launch_resolve::ros::graph_identity`, `node_name.is_some()`), every model carries a `meta.diagnostics` line naming how many keys are executable-derived and therefore unjoinable against `ros2 node list`, and the REAL name is captured authoritatively by the interceptor into `play_log/<ts>/interception/node_identity.tsv` (`model key<TAB>pid<TAB>real FQN`) — read from the `rcl_node_t*` the publish/subscribe init hooks already hold, so no inference and no renaming. An un-named node's MODEL KEY is always numbered from `-1` (`model_builder.rs`), so `/sensing/imu/imu_corrector_node-1`; the ordinal counts collisions of that key, so an unrelated insertion elsewhere never renumbers it, and an explicitly named node keeps its bare key. The suffix marks a key as executable-derived — exactly the set whose key is NOT the node's ROS name — and never reaches the `__node` remap; the Python parser instead keys them by `launch`'s global console process label (`<exec>-<index>`) and deliberately keeps that, being ported from ROS launch — issue #0018. Web UI: graph panel close button in hint state (`GraphPanel.js`); "Last Activity" sort in Node page sorts by `stderr_last_modified` with 3s polling, container groups use max child activity in tree mode (`NodeList.js`). Build: `just build` now includes `just build-interception`; interception .so required in wheel bundle (`bundle_wheel.sh`); `install-wheel` picks newest `.whl`. CI: `release-wheel.yml` uses `just build`; all GitHub Actions bumped to v5 (Node.js 24).
- **2026-03-11**: Phase 29 (RCL Interception) integration complete — play_launch consumer wired into replay.rs, 5 integration tests passing (stats, frontier, disabled, defaults, toggles). Bug fix: `rcl_publish`/`rcl_take` hooks now fall back to `lookup_publisher_full()`/`lookup_subscription_full()` for messages without `header.stamp`, dispatching with `stamp: None` so StatsPlugin counts all messages.
- **2026-03-11**: Fixed `$(eval)` Python conditional expressions returning "false" — `needs_python_eval()` didn't recognize `" if "` / `" else "` keywords, so expressions like `'centerpoint_tiny' if ''=='' else ''` were handled by the Rust string comparison evaluator (which found `==` first), naively split on it, and returned "false". Fix: added `" if "` and `" else "` to the `needs_python_eval()` keywords list in `eval.rs` so conditional expressions delegate to Python eval.
- **2026-03-11**: Fixed composable node container matching — 5 nodes were failing to find their target containers:
  1. **`container.rs` (Python mock)**: `ComposableNodeContainer` was missing `#[getter]` for `name` and `namespace`, causing `LoadComposableNodes.extract_target_container()` to fall through to `__repr__()`, producing targets like `"ComposableNodeContainer(name='...', namespace='')"` instead of proper names. Fix: added `#[getter]` annotations.
  2. **`load_composable.rs` (Python mock)**: `extract_target_container()` wasn't combining ROS namespace (from `get_current_ros_namespace()`) with container's own namespace when building the target name. Fix: integrate ROS namespace using same logic as `capture_container()`.
  3. **`load_composable_node.rs` (XML path)**: `to_captures()` and `to_load_node_records()` were prepending `context.current_namespace()` to the resolved target name, but target is a reference by name, not a namespace-relative path. Fix: store raw resolved value. The builder handles matching via suffix fallback.
  4. **`builder.rs`**: Container matching used strict exact match only. When a relative target like `"pointcloud_container"` (no namespace prefix) normalized to `"/pointcloud_container"`, it wouldn't match a container at `"/pointcloud_container"` if the composable node was in a deep namespace. Fix: added suffix matching fallback — when exact match fails, find a container whose full name ends with `"/{target_name}"`.
- **2026-03-10**: Fixed two bugs affecting `<executable>` tags in launch XML:
  1. **`context.rs:to_exec_context()`**: Was bailing on all records where `exec_name` was `None`, but raw executables (from `<executable>` tags) legitimately have `package: None` and may lack `exec_name`. Fix: only require `exec_name` when `package` is `Some` (i.e., ROS nodes).
  2. **`record_conv.rs:into_record_json()`**: Global params backfill was injecting `-p key:=value` into `cmd` for ALL records in `self.records`, including raw executables. Raw executables don't understand ROS params, so these leaked params (e.g., `-p use_sim_time:=True`) were passed as arguments to non-ROS processes like CARLA. Fix: skip `-p` cmd injection when `node.package.is_none()`.
- **2026-03-03**: Rust 2024 edition migration — all Cargo.toml files updated to `edition = "2024"`. `std::env::set_var`/`remove_var` wrapped in `unsafe {}` (20 call sites). `clippy::collapsible_if` expanded to if-let chains; all ~120 instances fixed with `cargo clippy --fix`. Dead code cleanup: removed unused wrapper functions (`run_container`, `run_regular_node`), unused methods/fields, `MemberDetails` struct; kept semantic/test-used items with targeted `#[allow(dead_code)]`. colcon-cargo-ros2 0.4.0 migration — config now generated at `.cargo/config.toml` (auto-discovered by cargo) instead of `build/ros2_cargo_config.toml`. Removed `--config` flags from justfile (`check`, `generate-bindings`). Deleted `scripts/patch_cargo_config.sh` (colcon 0.4.0 generates `rosidl_runtime_rs = "0.6"` and patches `.cargo/config.toml` directly). Pinned `colcon-cargo-ros2>=0.4.0` in justfile `install-deps`, `docker/builder.Dockerfile`, and `.github/workflows/release-wheel.yml`.
- **2026-03-02**: Split `GraphView.js` (~2300 lines) into 6 modules in `web/assets/js/components/`: `graph-utils.js` (pure helpers), `graph-builders.js` (snapshot→Cytoscape elements), `graph-edges.js` (edge routing + port bundling), `graph-layout.js` (ELK integration + scrollbars), `graph-styles.js` (Cytoscape stylesheet), `GraphView.js` (component + event handlers). Public API unchanged (`export function GraphView`).
- **2026-03-01**: Phase 25 (Topic Introspection) — graph view animated ELK layout transitions, overlap fix, and SSE rebuild stability. `applyElkPositions` supports `animate` param (300ms slide via `ele.animation()`); initial load instant, subsequent layouts animated. `buildElkGraph` computes leaf/port dimensions from style formula (not stale `layoutDimensions()`). Branch edges use `haystack` curve-style to avoid invalid-endpoint warnings when port nodes overlap leaf nodes. SSE rebuild defers collapse via `setTimeout(0)` after `cy.json()` so expand-collapse extension can register new elements. `[GraphView]` console logs at all lifecycle points (init, update, rebuild, expand/collapse, layout, edges). New file: `GraphPanel.js`.
- **2026-02-27**: Phase 24 (Web UI Parameter Control) complete. ParameterProxy service client wrapper, ParamValue types with bidirectional ROS conversion, MemberHandle integration with FQN map, GET/POST `/api/nodes/:name/parameters` endpoints, SSE `/parameter_events` subscription, ParametersTab frontend with type-aware inputs (bool toggle, number/string inputs, range constraints), search/filter. New files: `ros/parameter_types.rs`, `ros/parameter_proxy.rs`, `web/assets/js/components/ParametersTab.js`.
- **2026-02-24**: Phase 22 (Launch Tree IR) phases 22.1–22.8 complete. IR preserves full launch structure (conditions, expressions, groups, includes). IR now feature-gated behind `--features ir` in the parser crate; WASM crates removed from workspace (kept on disk).
- **2026-02-18**: Phase 21 (build optimization) mostly complete: `scripts/bundle_wheel.sh` with artifact manifest, incremental build recipes (`build-cpp`, `build-rust`, `build-wheel`), proper wheel platform tag via `wheel tags`. Phase 20 (web UI modernization) planned: Preact + htm + SSE-driven state, vendored locally (no CDN), zero polling.
- **2026-02-17**: Phase 19 complete — fork()+exec() isolation, child death monitoring, parallel loading, event-driven container status. PR_SET_CHILD_SUBREAPER added to replay. Subprocess PID cache uses time-based 1s interval.
- **2026-02-16**: Fix container parameter passing in fork+exec isolation: `write_params_file()` now serializes `request->parameters` (not `extra_arguments`), uses `/**:` wildcard YAML namespace, enforces decimal points on double arrays. Smoke test (`tests/src/health.rs`) detects `ComponentEvent LOAD_FAILED` events. `component_node` supports `--use-intra-process-comms`. Autoware: 64/64 composable nodes load successfully.
- **2026-02-15**: Default `--container-mode isolated` — replay overrides stock containers to `play_launch_container` with fork+exec isolation. `ComponentEvent` subscription conditional on mode (skipped for `stock`). Nextest failure output deferred to end (`--failure-output final`); isolated-container tests retry once for DDS flakes.
- **2026-02-08**: Phase 18 complete — ObservableComponentManager + play_launch_msgs packages.
- **2026-02-06**: Context unification investigation, namespace fix, YAML parameter loading for composable nodes. See `docs/roadmap/phase-17-context_unification.md`.
- **2026-02-01**: Phase 14 complete — Python launch file execution. Phase 15 — PyO3 type safety.
- **2026-01-27**: Phase 13 complete — Rust parser as default (3-12x speedup).

## Documentation

- **Roadmap**: `docs/roadmap/README.md` — all phases and progress
- **Design docs**: `docs/design/` — active design documents:
  - `container-isolation.md` — why containers, Linux isolation, RMW consequences
  - `context-unification.md` — parser LaunchContext unification (Phase 17)
  - `system-model.md` — the SystemModel artifact: layout, layers, producer/consumer split
  - `criticality-from-hazards.md` — deriving criticality (and reservations)
    from declared hazards instead of a `high|medium|low` label
  - `unified-system-model.md` — Phase 46: the SystemModel as the ONE complete artifact (`record.json` retired to deprecated compat)
  - Manifest design docs live in the `ros-launch-manifest` repo's `docs/` (Phase 31; that repo is a git dependency, so read them at the pinned tag or in `~/.cargo/git/checkouts/`)
  - `composable-load-admission.md` — how many composables may come up at once:
    why the governor is memory rather than a count, and why the answer differs
    per container mode
  - `rcl-interception.md` — RCL interception architecture + graph discovery evolution
  - `record-format.md` — record.json format: current fields + Phase 30 extensions (scopes)
  - `parser-context.md` — parser LaunchContext: scope chain, namespacing, captures
  - `launch-context-tool.md` — context extraction tool design
  - `manifest-slides.md` — presentation slides for launch manifest + RCL interception
- **Archived designs**: `docs/design/archive/` — completed/abandoned designs
- **Research**: `docs/research/data-quality-semantics.md` — data loss, latency, sync semantics survey
- **Launch Tree IR**: `docs/roadmap/phase-22-launch_tree_ir.md` — IR design (feature-gated behind `--features ir`)
- **Topic Introspection**: `docs/roadmap/phase-25-topic_introspection.md` — graph view, namespace grouping, ELK layout, edge bundling
- **RCL Interception**: `docs/roadmap/phase-29-rcl_interception.md` — LD_PRELOAD interceptor, SPSC shared memory, frontier/stats plugins
- **Launch Scoping**: `docs/roadmap/phase-30-launch_scoping.md` — scope table, context extraction, cross-parser validation
- **Launch Manifest**: `docs/roadmap/phase-31-launch_manifest.md` — manifest crate, parser/executor integration, audit
- **Migration Guide**: `docs/guide/parser-migration.md` — Rust parser migration (v0.6.0+)
- **Multi-host Guide**: `docs/guide/multi-host.md` — running one launch file's nodes across multiple hosts (ROS 2 has no `<node machine=>`)
- **Edge Machines Guide**: `docs/guide/edge-machines.md` — what to change on a Jetson-class board, and why `--container-mode observable` is worth more than everything else combined

## Parser Parity Status

Entity counts match between parsers (100% functional equivalence). Remaining cosmetic differences: CMD deduplication, params_files expansion, CMD ordering, XML whitespace, array quoting, boolean case, extra_args defaults. See `docs/roadmap/phase-17-context_unification.md`.

## Distribution

PyPI: `pip install play_launch` (x86_64 + aarch64, Ubuntu 22.04+). Build: `just build`. Publish: `just publish-pypi`.
