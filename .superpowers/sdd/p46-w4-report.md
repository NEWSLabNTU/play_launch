# P46 W4 Report — Python parser → model + remove the record-binding gate (46.4, acceptance gate)

Branch `feat/p46-python-model`, created fresh from `origin/main` (`33fddd9`). Not pushed.

Commits:
- `e5fdbf2` feat(resolve): --parser python → structure-only model (46.4)
- `3bc9374` fix(replay): drop vestigial record-binding gate on the model path (46.4)
- `<amend>` docs/framing correction (this section — see "Correction" below)

## Correction (post-review — the "structure-only by construction" claim was WRONG)

My original acceptance runs reported `0 topics, 0 contracts-carrying
endpoints, 0 tier(s)` for the Python path and I framed that as "structure-
only **by construction** — the Python parser doesn't produce contract/sched
facts." **That framing is factually wrong** (review:
`.superpowers/sdd/p46-w4-review.md`).

Root cause: my manual runs' embedded PyO3 Python imported a **stale
pip-installed** `play_launch` 0.8.2 (`~/.local/lib/python3.10/site-packages/
play_launch`, dated April) whose `ScopeOrigin` predates Phase 40.1 and has
**no `path` field**. `manifest_loader`/`sched_loader` key the provider-
sidecar contract + platform-file channels off `ScopeEntry.origin.path`; with
`path` absent, those channels silently resolve nothing → empty contracts/
sched. The 0-count was a stale-install artifact, not a parser property.

**The pipeline is correct and needs no logic change** — `manifest_loader`/
`sched_loader` operate on the shared scope table independent of which parser
produced it, and BOTH parsers emit `ScopeOrigin.path` (Phase 40.1). Re-run
with the worktree's current Python source on `PYTHONPATH`,
`resolve --parser python` on rt_workspace (which ships both a contract
sidecar and a platform file) produces a model whose `structure`,
`contracts`, and `execution` layers are **byte-identical** to the Rust path
(3 topics, 5 contracts-carrying endpoints, 3 tiers). Evidence pasted in
"Acceptance run 1 (CORRECTED)" below.

So: Python gets **full** contract/sched parity whenever sidecars/`--sched`
inputs exist and the install is current. The layers are empty only when no
such input resolves — NOT a Python limitation. This wave's code (only
comments/help-text) was corrected; the model-building logic was already
right. **The spawn acceptance is unaffected** — spawning never depends on
the contracts/sched layers (it reads `structure.nodes`), so both the
original and corrected runs spawn identically.

## Goal recap

The user's explicit requirement: the Python parser must still work **after**
record.json retirement (46.5) — `resolve --parser python` must produce a
spawnable SystemModel, and `replay --model` must spawn from it cleanly, with
the vestigial `meta.record` binding gate removed. This is the acceptance
gate for 46.5.

## 1. `resolve --parser {rust,python}` wiring (`e5fdbf2`)

- Added `--parser` (`ParserBackend`, default `rust`) to `ResolveArgs`
  (`cli/options.rs`).
- `commands/resolve.rs`'s no-`--record` branch now dispatches on
  `args.parser`:
  - `Rust` (unchanged): `play_launch_parser::parse_launch_file` → JSON →
    `LaunchDump`, record companion written from that JSON string.
  - `Python` (new): runs `DumpLauncher::dump_launch` (the same PyO3 bridge
    `dump --parser python` uses) straight into the record-companion path (or
    a scratch temp file in `-o -` stdout mode, cleaned up after use), then
    `load_launch_dump()` reads it back into the same `LaunchDump` type.
- **Both branches feed the identical downstream pipeline** from that point
  on — `manifest_loader::load_manifests` → contract-error gate →
  `sched_loader` → `model_builder::build_system_model`. No python-specific
  branching exists past `LaunchDump` construction. The contract/sched layers
  apply on the shared scope table (`manifest_loader`/`sched_loader` key off
  `ScopeEntry.origin.path`, Phase 40.1 — emitted by both parsers),
  independent of parser: they come back populated whenever a contract
  sidecar / platform file (or `--contracts`/`--sched`) resolves and the
  install is current, and empty only when none does. On rt_workspace (both
  sidecars present, current source) the Python-produced model is
  byte-identical to Rust across structure/contracts/execution (see
  Correction above + Acceptance run 1 CORRECTED).
- `commands/launch.rs`'s internal `ResolveArgs` construction (its `--record`
  step 2/3) gained the new required field, set to `args.parser` for
  documentation — unused on that path since `--record` mode bypasses parser
  selection entirely.

## 2. Gate removal (`3bc9374`)

- `commands/replay.rs`: `verify_model_record_binding` (hashed
  `--input-file` against `model.meta.record`, refused on mismatch or absent
  binding) replaced by `load_system_model` — parses the model YAML, no
  record read, no hash check.
- `meta.record` is **kept** on the model (informational provenance only,
  `resolve` still writes it); dropped entirely with `record.json` at 46.5,
  per the brief.
- `play()`'s `load_launch_dump(input_file)` call remains (best-effort
  consumers: chain-colocation warnings, the web UI's launch-tree scope map),
  but now degrades to an empty `LaunchDump` instead of erroring whenever a
  `SystemModel` was given and the file is missing/unreadable. The legacy
  `replay --input-file record.json` path (no `--model`) is **unchanged** — a
  missing/bad record there is still a hard error.
- Old test `replay_model_binding_gate_refuses_stale_record` asserted the
  now-removed refusal (it would otherwise hang forever, since replay now
  proceeds to spawn and run indefinitely) — replaced with
  `replay_model_spawns_without_record_companion` (Rust-path regression) and
  a new `resolve_parser_python_produces_model_that_replays_cleanly`
  (Python-path acceptance, automated). Both pass in `just test-all`'s
  `rt_workspace` binary.

## Acceptance run 1 (CORRECTED) — Python path FULL PARITY (rt_workspace, 4 nodes)

Re-run with the worktree's current Python source prepended to `PYTHONPATH`
(`export PYTHONPATH=<worktree>/python:$PYTHONPATH`), so the embedded Python
imports the Phase-40.1 `ScopeOrigin.path`-emitting source instead of the
stale pip 0.8.2:

```
$ play_launch resolve --parser python -o system_model_py.yaml rt_demo bringup.launch.xml
Resolving launch file: .../rt_workspace/install/rt_demo/share/rt_demo/launch/bringup.launch.xml
Resolving via Python parser
[INFO] [dump_launch]: ...
INFO play_launch::python::python_bridge: Dump completed successfully: system_model_py.record.json
Record companion: system_model_py.record.json
INFO play_launch::ros::manifest_loader: Loaded 1 manifest(s) [0 overlay, 1 provider] (0 scopes without manifests, 0 errors, 1 warnings)
INFO play_launch::ros::sched_loader: Scheduling platform file [provider]: .../rt_workspace/launch/bringup.system.posix.yaml
WARN play_launch::commands::resolve: override pins chain member `/control/control_node` to priority 20 (derived 40) — below its chain-derived rank; ...
SystemModel: system_model_py.yaml (4 nodes, 3 topics, 5 contracts-carrying endpoints, 3 tier(s), 2 warning(s))
```

`3 topics, 5 contracts-carrying endpoints, 3 tier(s)` — **full contract +
sched layers**, matching Rust exactly. The scope entry now carries
`"path": ".../rt_workspace/launch/bringup.launch.xml"`, which unlocks the
provider-sidecar contract + platform-file channels. Layer-by-layer diff
against `resolve` (Rust, same fixture):

```
structure:  IDENTICAL
contracts:  IDENTICAL
execution:  IDENTICAL
```

(My original run, which reported `0 topics, 0 contracts, 0 tiers`, was the
stale-pip artifact described in "Correction" above — not a parser property.)

Then `replay --model` from a **separate, empty** working directory (no
`record.json` anywhere on the filesystem near it). (This replay used the
original structure-only variant of the model — the "carries no execution
layer" line reflects that; spawn is driven purely by `structure.nodes` and
is identical whether or not the sched/contracts layers are present, which is
the whole point of the acceptance — spawn never depends on those layers.)

```
$ cd /tmp/py_rt_replay   # empty dir, no record.json
$ play_launch replay --model .../system_model_py.yaml --disable-web-ui --disable-monitoring --disable-diagnostics
INFO play_launch::commands::replay: SystemModel: .../system_model_py.yaml
INFO play_launch::commands::replay: Contract source: SystemModel (checked at resolve time)
INFO play_launch::commands::replay: Spawn source: SystemModel (structure.nodes)
INFO play_launch::commands::replay: SystemModel carries no execution layer — scheduling disabled
INFO play_launch::commands::replay: Spawning 4 nodes (2 pure nodes, 1 containers, 1 composable nodes)
INFO play_launch::commands::signal_handler: Startup complete: all nodes ready (nodes 2/2, containers 1/1, composable 1/1)
--- node dirs under play_log/latest/node/ ---
control_node
perception_container
sensor_node
--- cmdline files ---
play_log/latest/node/perception_container/cmdline
play_log/latest/node/sensor_node/cmdline
play_log/latest/node/control_node/cmdline
```

Clean spawn, no binding error, no record companion present or required.

## Acceptance run 1b — Python path (simple_test standard launch, 2 nodes)

`tests/fixtures/simple_test/launch/pure_nodes.launch.xml` — a plain
`talker`/`listener` launch with no contracts/sched sidecars at all:

```
$ play_launch resolve --parser python -o system_model_py.yaml .../pure_nodes.launch.xml
Resolving via Python parser
Record companion: system_model_py.record.json
SystemModel: system_model_py.yaml (2 nodes, 0 topics, 0 contracts-carrying endpoints, 0 tier(s), 0 warning(s))
```

```
$ cd /tmp/py_simple_replay   # empty dir
$ play_launch replay --model .../system_model_py.yaml --disable-web-ui --disable-monitoring --disable-diagnostics
INFO play_launch::commands::replay: Spawn source: SystemModel (structure.nodes)
INFO play_launch::commands::replay: Spawning 2 nodes (2 pure nodes, 0 containers, 0 composable nodes)
INFO play_launch::commands::signal_handler: Startup complete: all nodes ready (nodes 2/2, containers 0/0, composable 0/0)
--- node dirs ---
listener
talker
--- cmdline files ---
play_log/latest/node/talker/cmdline
play_log/latest/node/listener/cmdline
```

## Acceptance run 2 — Rust path parity intact (Autoware, 119 nodes)

```
$ play_launch resolve -o system_model_rust.yaml autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning
Resolving launch file: /opt/autoware/1.5.0/share/autoware_launch/launch/planning_simulator.launch.xml
WARN play_launch_parser::substitution::types: Executing $(command xacro ...) — command substitutions run arbitrary shell commands. Use --block-commands to reject them.
Record companion: system_model_rust.record.json
SystemModel: system_model_rust.yaml (119 nodes, 0 topics, 0 contracts-carrying endpoints, 0 tier(s), 0 warning(s))

real    0m0.284s
```

119 = 34 nodes + 15 containers + 70 load_nodes, matching
`activate_autoware.sh`'s pinned Autoware 1.5.0 counts — full-model Rust
parity unchanged by this wave. `replay --model` spawn-from-model (46.3b) for
the Rust path is exercised end-to-end by the new automated
`replay_model_spawns_without_record_companion` test (rt_workspace, 3
processes: 2 nodes + 1 container) rather than a full Autoware run (heavy —
perception stack, GPU) — not required by the brief's acceptance wording,
which only asks for the 119-node parity count on the Rust side.

## Verification

- `cargo test -p play_launch`: 249 passed, 0 failed (all unit tests).
- `cd tests && cargo nextest run` filtered to
  `rt_workspace|manifest_check|sched_apply|resolve_*|container_events`: **46
  passed, 0 failed, 53 skipped** (skipped = autoware/interception-gated
  binaries not in this filter). Includes the two new/replaced
  `rt_workspace` tests.
- `clippy --all-targets -- -D warnings`: clean on all 4 touched
  `play_launch` source files (grepped clippy output for
  `commands/resolve.rs|replay.rs|launch.rs|cli/options.rs` — zero hits).
  **Pre-existing, unrelated** clippy failure in
  `runtime_enforcement/mod.rs:781` (`manual_map`) blocks a repo-wide
  `-D warnings` run — confirmed via `git diff origin/main -- .../mod.rs`
  (empty diff, untouched by this branch); out of scope for 46.4.
- `rustfmt --check` (project `rustfmt.toml`) on all 5 touched files: clean.
  `tests/tests/rt_workspace.rs` has two pre-existing formatting-drift lines
  elsewhere in the file (nightly-vs-stable rustfmt, lines 1136/1333,
  confirmed outside my diff hunk) — untouched, out of scope.

## Environment notes (for future waves in this worktree)

- Fresh worktree needed `git submodule update --init --recursive`
  (`play_launch_parser`, `ros-launch-manifest`, `play_launch_container`,
  `play_launch_msgs`, vendor crates) before `just build-rust` would even
  resolve the workspace manifest.
- Symlinked `install/` and `build/` to the main checkout's shared colcon
  workspace (`/home/aeon/repos/play_launch/{install,build}`) rather than
  rebuilding C++ from scratch — same convention already in use by sibling
  worktrees, confirmed no diff in `play_launch_container`/`play_launch_msgs`
  sources between branches. Same for `tests/fixtures/rt_workspace/{install,build}`.
  These symlinks are untracked and gitignored-by-content-type but not
  matched by the trailing-slash gitignore patterns (git doesn't match
  `dir/`-style patterns against symlinks) — left as untracked, not
  committed.
- `tests/Cargo.toml` needed a temporary `[workspace]` table to work around
  the nested-worktree cargo quirk (cargo's workspace discovery walks up and
  finds the outer repo's root `Cargo.toml` from inside
  `.claude/worktrees/...`) — added, used for all `cargo nextest run`
  invocations, reverted before both commits (not present in either commit's
  diff).
- Reverted incidental `cargo fmt`/build side effects that touched 7 files I
  never intended to edit (`capabilities.rs`, `rt_helper_client.rs`,
  `interception/mod.rs`, `ipc/sched_protocol.rs`, `regular_node_actor.rs`,
  `io_helper_client.rs`, `sched.rs`) plus environment-specific `Cargo.lock`
  drift (`diagnostic_msgs`/`std_msgs`/`geometry_msgs` version mismatch
  between the committed lockfile and this machine's installed ROS packages)
  back to their committed state before staging — none of it is part of
  either commit.

## Notes for 46.5

What still writes/reads `record.json` after this wave (all explicitly kept
per the brief, "unchanged this wave"):
- `dump --parser {rust,python}` still emits `record.json` as its only
  artifact (`commands/dump.rs`, untouched).
- `resolve` (both parsers) still writes a `<out>.record.json` companion
  next to the model and stamps `meta.record` with its hash — now purely
  informational (nothing reads it back to gate anything), but the write
  itself, and the `meta.record` field, are explicitly scoped to 46.5 for
  removal per the design doc ("Dropped from the artifact entirely:
  ... `meta.record` + `verify_model_record_binding` — one file, nothing to
  mismatch").
- `just compare-dumps` / `scripts/compare_scopes.py` and all
  parser-parity tooling still operate on `record.json` pairs — unchanged,
  dev-only, 46.5's job to converge onto the model.
- `replay --input-file record.json` (no `--model`) is the still-fully-live
  legacy path — `load_launch_dump` is a hard requirement there, unchanged.
- The `play()` function's best-effort `launch_dump`-only consumers (chain-
  colocation warnings, the web UI's launch-tree/scope map built from
  `launch_dump.scopes`) have **no model-sourced equivalent yet** — they
  simply come back empty on a model-only replay today. 46.5 (or a later
  wave) should either source these from `model.structure.scopes` directly,
  or explicitly document them as record.json-only diagnostics that
  disappear once record.json retires.

Python-path parity is **install-sensitive** (a trap for 46.5 planning and
CI): `resolve --parser python` gets full contract/sched parity with Rust
ONLY when the embedded PyO3 Python imports a current `play_launch` package
(one emitting `ScopeOrigin.path`, Phase 40.1). A stale pip-installed
package (e.g. this host's `~/.local/.../play_launch` 0.8.2) silently drops
`ScopeOrigin.path`, which disables the provider-sidecar contract +
platform-file channels — producing a structure-only model with **no error**.
The automated acceptance test (`resolve_parser_python_produces_model_that_
replays_cleanly`) deliberately asserts only parser-inherent, install-
independent facts (node count + spawnability), so it's not flaky on a stale
host; the full-parity check is the manual run above. 46.5 should ensure the
Python path picks up the bundled/current source (prepend the wheel's python
dir, or fail loud when `ScopeOrigin.path` is absent) rather than silently
degrading — otherwise `dump→model` convergence would inherit the same
silent contract/sched loss.
