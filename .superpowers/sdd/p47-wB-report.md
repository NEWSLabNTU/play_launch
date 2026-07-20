# P47 Wave B-rest Report — hard record.json removal (47.B2–B6)

Branch `feat/p47-record-removal`, created fresh from `origin/main` (`ae71463`,
which already carries A1–A3 and B1). Not pushed. NON-ADDITIVE: `record.json`
is removed as a user-facing artifact everywhere it was still produced or
consumed on a primary path. `LaunchDump` (the parser's serializable
intermediate) is unchanged and stays as an in-memory-only structure.

## What was removed

- **`dump --format record`** and the `DumpFormat`/`--format` CLI surface
  (`cli/options.rs`, `commands/dump.rs`) — `dump` always emits the
  SystemModel now.
- **`dump run`** subcommand (`DumpSubcommand::Run`) — it had no SystemModel
  form (a single executable has no launch scope tree to build one from) and
  its sole purpose was writing `record.json`. `play_launch run` already
  covers single-node dump+replay-in-one. Removed with it: `DumpLauncher::
  dump_run` and `python_bridge::run_dump_run` (dead code once the only
  caller was gone).
- **`replay --input-file <record.json>`** — hard cut, not deprecation.
  `ReplayArgs` no longer has `input_file`; `replay` now takes the model
  positionally (`<MODEL>`) or via `--model <path>` (`conflicts_with` each
  other, `ReplayArgs::model_path()` picks whichever was given). Verified:
  `replay --input-file record.json` → clap `error: unexpected argument
  '--input-file' found` (exit 2) — a real parse rejection, not a silent
  misparse. `replay` with neither → a clear `eyre` error naming both ways
  to give a model (exit 1).
- **`resolve --record <path>`** (record-reuse mode) — removed per the
  brief's stated preference ("no record.json is written anymore"; keeping a
  read-only reuse mode for a artifact nothing produces was disproportionate).
  `ResolveArgs::package_or_path` is `String` (required) now, not
  `Option<String>`.
- **`scripts/compare_records.py`** and **`scripts/compare_parsers.sh`** —
  deleted. `scripts/compare_models.py` (Phase 47.B1) is the parity tool.
  Updated the two doc-comment mentions left in
  `src/play_launch/src/ros/launch_dump.rs` that referenced the deleted
  script.
- Fixture justfiles (`autoware`, `rt_workspace`, `simple_test`,
  `container_events` — the 4 that had the recipe set) — dropped
  `dump-rust`/`dump-python`/`dump-both`/`compare-dumps-record`; kept
  `compare-dumps` (already model-based, from B1). `autoware/justfile`'s
  `count-processes` recipe now resolves a model instead of `dump --format
  record`. Top-level `justfile`'s `compare-parsers` recipe (called the now
  deleted `compare_parsers.sh`) rewritten inline: resolves both parsers on
  the 2 sample launch files `compare_parsers.sh` used to cover and runs
  `compare_models.py` — verified passing (see below).
- **`scripts/count_processes.py`** — now reads a SystemModel YAML
  (`.yaml`/`.yml`) as the primary input (plain nodes + containers from
  `structure.nodes`); a `.json` (legacy record) path is still accepted
  read-only for old files, nothing produces one anymore.

## What stays (by design)

- **`LaunchDump`** (`ros/launch_dump.rs`) — unchanged struct/serde shape,
  still the parser-agnostic in-memory intermediate `resolve`/`launch` build
  a SystemModel from. Added `LaunchDump::empty()` (used by standalone
  `replay`, see below). Not written to disk on any primary path anymore.
- **`prepare_node_contexts`/`prepare_container_contexts`/
  `prepare_composable_node_contexts`** (`execution/context.rs`, the
  LaunchDump-sourced spawn-context builders) — kept, `#[allow(dead_code)]`
  added to the container/composable variants (the plain-node one is still
  reachable from `run`). They're unreachable in a normal build now that
  `replay`/`launch` spawn exclusively from the `_from_model` variants, but
  `run` (single-node dump+replay) still uses the plain-node one, and
  `execution::spawn_equivalence_test` (`#[cfg(test)]`) is the regression
  gate proving the model-sourced and record-sourced spawn paths produce
  identical `NodeContext`/`NodeContainerContext`/`ComposableNodeContext` —
  deleting the record-sourced builders would have deleted that gate's
  reference implementation too.
- **`ContractView::from_manifest_index`** + `runtime_enforcement::qualify`
  — same story: unreachable in `replay` now (contract source is always
  `from_model`), `#[allow(dead_code)]`-kept because
  `from_model_matches_from_manifest_index` and several `runtime_enforcement`
  unit tests use it as the independent reference `from_model` is checked
  against.
- **`play_launch context record.json`** — the CLI's one remaining reader of
  an on-disk record.json-shaped file. See "Residual record.json use" below.

## The B4 in-memory `launch` round-trip

`commands/launch.rs` was rewritten from "dump to `record.json` on disk →
`resolve --record record.json` → construct a `ReplayArgs` → `handle_replay`"
into a single async function:

1. `commands::common::parse_to_launch_dump()` (new, shared with `resolve`) —
   parses the launch file into a `LaunchDump` with **zero disk I/O for the
   Rust parser** (parse → `serde_json` string → deserialize, all in memory
   — the same trick `resolve`'s Rust branch already used) and **a private
   OS-temp scratch file for the Python parser** (the PyO3 `dump_launch`
   bridge only knows how to write JSON to a path; the scratch file is
   deleted before the function returns — an interop detail, not a
   `record.json` left behind).
2. `commands::resolve::build_checked_model()` (new, extracted from
   `handle_resolve`'s body) — takes the in-memory `LaunchDump` +
   `ModelBuildInputs` (contracts/sched/system paths, arg bindings) and
   returns a checked `SystemModel`, no file write.
3. `commands::replay::play()` (signature changed: now `async fn
   play(launch_dump: LaunchDump, system_model: Arc<SystemModel>, common:
   &CommonOptions)`, was `async fn play(input_file: &Path, common:
   &CommonOptions, system_model: Option<Arc<SystemModel>>)`) — called
   directly, in-process. No second CLI invocation, no file round-trip.

`handle_resolve` and `handle_launch` both call the same two shared
functions now — `resolve` writes the model to disk afterward, `launch`
keeps it in memory and calls `play()` directly.

`handle_replay` (standalone `replay` command) calls `play()` with
`LaunchDump::empty()` — there's no companion dump to read anymore (never
was one to read after Phase 46.5; B3 just removed the flag that pretended
otherwise). The best-effort/informational consumers that read from
`launch_dump` (chain-colocation warnings, the web UI's `/api/launch-tree`
scope map) degrade to empty for standalone `replay --model` — same
documented limitation as before B2-B6, not a regression. For `launch`,
those consumers get the REAL dump (parsed in step 1), so they stay
populated on that path specifically.

## Test migrations (per file)

All migrations replace `fixtures::dump_launch()` (record.json,
`--format record`) with `fixtures::resolve_model()`/
`resolve_model_with_args()` (SystemModel) and `array_len(record, "node"|
"container"|"load_node")` with `fixtures::model_entity_counts(&model)` →
`(plain, containers, composables)`. `fixtures::count_expected_processes
(&record_path)` → `fixtures::count_expected_processes_from_model(&model)`
(`plain + containers`, composables excluded — same semantic, virtual
members aren't separate processes). New fixtures.rs helpers:
`resolve_model_with_args` (extra `KEY:=VALUE` launch args — Autoware needs
`map_path:=`), `first_container_args` (model's `args` field, for the
`--isolated` checks that used to read `record["container"][0]["cmd"]`).
Removed: `dump_launch`, `compare_records` (record.json based).

- **`tests/tests/autoware.rs`**: `dump_autoware()` → `resolve_autoware()`
  (uses `resolve_model_with_args` for `map_path:=`). All dump/parity/
  process-count tests renamed `test_autoware_dump_*` → `test_autoware_
  resolve_*` and switched to model comparison; `test_autoware_parser_
  parity` now calls `compare_models.py` on `system_model.yaml` files
  instead of `compare_records.py` on `record.json`. `test_autoware_smoke_
  test` (the model-path acceptance test) resolves via `resolve_autoware`
  then `launch`s — this is the same test the brief's "Autoware model-path
  smoke" criterion asks for; see evidence below.
- **`tests/tests/simple_workspace.rs`**: local `dump_launch()` wrapper →
  `resolve_counts()`; `test_dump_*` → `test_resolve_*`; parity tests compare
  the `(plain, containers, composables)` tuple directly instead of 3
  separate `array_len` asserts; `test_launch_pure_nodes`'s inline `dump
  --format record` + `count_expected_processes` replaced with
  `resolve_model` + `count_expected_processes_from_model`.
- **`tests/tests/container_events.rs`**: `test_dump_*` → `test_resolve_*`
  (rust/python/parity). `test_dump_isolated_has_args` → `test_resolve_
  isolated_has_args`: checks `fixtures::first_container_args(&model) ==
  ["--isolated"]` instead of `record["container"][0]["cmd"]`/`["args"]` —
  the model's `args` field IS the launch-declared source of truth
  `<node_container args="--isolated">` lowers to; the fully-assembled `cmd`
  argv record.json exposed was a replay-time derivation the model doesn't
  store precomputed (checking `args` is not weaker — it's the same
  parser-output being asserted, one layer earlier). `test_launch_container_
  events` migrated to `resolve_model` for the expected count. The 6
  crash-detection/data-delivery tests (`test_crash_detection`,
  `test_isolated_data_delivery`, `test_isolated_external_subscriber`,
  `test_observable_data_delivery`) don't touch record.json — untouched.
- **`tests/tests/sequential_loading.rs`/`concurrent_loading.rs`/
  `mixed_loading.rs`**: identical near-clone templates, same migration
  pattern (`test_dump_*` → `test_resolve_*`, parity via tuple comparison,
  launch test via `resolve_model` + `count_expected_processes_from_model`).
- **`tests/tests/io_stress.rs`**: same pattern (not in the brief's explicit
  6-file list, but uses the same retired `dump_launch`/`array_len` helpers
  — had to migrate or the crate wouldn't build once those helpers were
  removed).
- **`tests/tests/parallel_loading.rs`**: same pattern (also not in the
  brief's list, same reason). `test_dump_parallel_loading` →
  `test_resolve_parallel_loading`, its `--isolated` cmd check migrated to
  `first_container_args` like container_events.rs.
- **`tests/tests/sched_apply.rs`**: one call site (`sched_apply_warn_
  engages_and_launch_succeeds`) — inline `dump --format record` +
  `count_expected_processes` replaced with `resolve_model` +
  `count_expected_processes_from_model`. (Also not in the brief's list,
  same reason as io_stress/parallel_loading.)
- **`tests/tests/rt_workspace.rs`**: `replay_input_file_record_json_is_
  deprecated_but_still_spawns` (asserted the OLD deprecation-warning
  behavior) replaced with two tests matching the hard cut:
  `replay_input_file_flag_is_removed_and_errors_clearly` (spawns `replay
  --input-file record.json`, asserts nonzero exit) and `replay_without_
  model_errors_clearly` (spawns `replay` with no model at all, asserts
  nonzero exit). Doc comment on `replay_model_spawns_without_record_
  companion` updated to reflect the hard cut (was: "the default `--input-
  file record.json` simply doesn't exist there"; now: "no record companion
  required OR possible"). All other `rt_workspace.rs` tests (`dump_parity_
  bringup`, the `check`/`resolve`/`sched` suites) were already model-based
  from B1 and needed no changes.

**Deviation from the brief's file list**: the brief named 6 files
(`autoware`, `simple_workspace`, `container_events`, `sequential_loading`,
`concurrent_loading`, `mixed_loading`). Three more (`io_stress.rs`,
`parallel_loading.rs`, `sched_apply.rs`) also called the retired `fixtures::
dump_launch`/`array_len`/`count_expected_processes` helpers and would not
have compiled otherwise — migrated them too, same pattern, to keep the
crate buildable. Not a scope expansion in spirit, just completing the set
the helper removal actually touches.

## Autoware model-path smoke — evidence

**Resolve (Rust parser), no record.json:**
```
$ play_launch resolve --parser rust -o aw_model.yaml autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning
SystemModel: aw_model.yaml (119 nodes, 0 topics, 0 contracts-carrying endpoints, 0 tier(s), 0 warning(s))
  (0.27s)
$ find . -maxdepth 2 -iname "record*.json"
(nothing)
```
119 = 34 nodes + 15 containers + 70 composables, matching the B1 report's
Autoware entity counts.

**`play_launch launch` (the in-memory B4 round-trip), full spawn:**
```
Step 1/3: Parsing launch file...
Step 2/3: Resolving SystemModel...
Step 3/3: Replaying launch execution...
Contract source: SystemModel (checked at resolve time)
Spawn source: SystemModel (structure.nodes)
...
Startup complete: all nodes ready (nodes 34/34, containers 15/15, composable 64/64)
```
- 49 process cmdline files appeared immediately and stayed stable (34 nodes
  + 15 containers) for the full ~90s wait window.
- `grep -c "record*.json"` under the fixture dir and `/tmp`: **0 matches** —
  no record.json anywhere during the run.
- `grep ERROR` on the run's output, excluding `rviz2`: **0 matches** — the
  908 ERROR lines are all `rviz2` respawn-loop noise (expected: no X
  display in this environment; `rviz2` is in the smoke test's
  `ignored_exits` list, same as the pre-existing `test_autoware_smoke_test`
  baseline). No `CRASHED` events, no `shape_estimation` failures.
- 64/70 composable nodes reached LOADED — matches the pre-existing documented
  baseline (memory notes: "64/64 Autoware composable nodes load successfully",
  Phase 19.11), not a regression.
- Cleanly torn down via `kill -TERM -$PGID` / `kill -9 -$PGID` on the
  process group; no orphaned play_launch/component_container/
  planning_simulator processes remained afterward.

This is the same shape of evidence `just test-all`'s `autoware::
test_autoware_smoke_test` (which now resolves via the model path) checks
programmatically — see the `just test-all` run below, which includes it.

**CLI hard-cut checks (also verified manually, then codified as
`rt_workspace.rs` tests, both PASS in `just test-all`):**
```
$ play_launch replay --input-file record.json
error: unexpected argument '--input-file' found
  tip: to pass '--input-file' as a value, use '-- --input-file'
Usage: play_launch replay [OPTIONS] [MODEL]
(exit 2)

$ play_launch replay --disable-web-ui
Error: replay requires a SystemModel: `play_launch replay <system_model.yaml>` or `play_launch replay --model <path>` (produce one with `play_launch resolve` or `play_launch dump`)
(exit 1)

$ play_launch replay system_model.yaml --disable-web-ui --disable-monitoring --disable-diagnostics
... Spawn source: SystemModel (structure.nodes) ... Startup complete: all nodes ready (nodes 2/2, ...)
(positional model path works; no record.json anywhere)
```

**`just compare-parsers`** (top-level justfile, rewritten since it called
the now-deleted `compare_parsers.sh`): resolves both parsers on
`pure_nodes.launch.xml` and `composition.launch.xml`, runs
`compare_models.py` — both PASS, no record.json produced, and the Python
scratch file (`/tmp/play_launch-parse-*.record.json`) is confirmed deleted
after each run.

## `just test-all` result

Full run (fresh `just build-cpp` + `just build-rust` + `just build-
interception` in this isolated worktree; `rt_workspace` fixture built via
its own `just build`; ROS humble + Autoware 1.5.0 env):

```
Parser unit tests:  420 passed, 0 failed, 0 skipped
Integration tests:  103 passed, 0 failed, 0 skipped   (was 102 on B1 — net
                                                        +1 from splitting
                                                        the retired
                                                        deprecation-warning
                                                        test into 2 hard-cut
                                                        tests)
```

Includes `autoware::test_autoware_smoke_test`, `test_autoware_process_
count_{rust,python}`, `test_autoware_resolve_{rust,python}`, `test_autoware_
parser_parity`, and `rt_workspace::{replay_input_file_flag_is_removed_and_
errors_clearly, replay_without_model_errors_clearly, replay_model_spawns_
without_record_companion, dump_parity_bringup, resolve_no_longer_writes_
record_companion}` — the exact acceptance tests for this wave.

Run twice: once before a final `cargo fmt`/`rustfmt` pass on the touched
Rust files (both `play_launch` and `tests` crates), once after (formatting
only, no logic changes) — both green. `cargo clippy --all-targets
--all-features -- -D warnings` clean on `play_launch` (found and fixed one
PRE-EXISTING unrelated `clippy::manual_map` lint in `runtime_enforcement/
mod.rs:781` that blocked the whole-crate `-D warnings` run — a trivial,
behavior-preserving one-liner, noted here since it's incidental to this
wave, not part of the record.json removal itself). `cargo clippy --tests
-- -D warnings` clean on the `tests` crate.

## Residual `record.json` use + why

**`play_launch context record.json --tree|--node|--launch`**
(`commands/context.rs`) is the only place left that reads an on-disk
record.json-shaped file. Left unchanged. Reasons this wasn't migrated:

1. Not in the brief's B2–B6 deliverables list (only in the "files that
   touch record.json" read-first list) — `context`'s CLI help was updated
   to say so explicitly (`play_launch --help`: "the last CLI consumer of
   the retired record.json format — there is no CLI path left that
   produces one").
2. The model's scope representation (`ros_launch_manifest_model::
   ScopeInfo`: `parent: Option<String>`, `manifest: Option<String>`) lacks
   the granularity `context` needs (`ns`, per-scope `args`, launch file
   `pkg`/`file`/`path`) that `LaunchDump`'s `ScopeEntry` carries. Migrating
   `context` onto the model would be a data-model expansion project, not a
   record.json-removal task — out of proportion for this wave, matching
   the brief's explicit escape valve ("if removal cascades unexpectedly,
   scope it... document what still reads record.json + why").
3. It's a read-only dev tool for files that predate this wave or are
   hand-produced (`play_launch_parser`'s `Record` type, unaffected by this
   wave, serialized directly) — not a live production path, and doesn't
   cascade into anything else.

No other production code path reads or writes record.json. `LaunchDump`
(the struct) is unchanged and stays as the parser's in-memory intermediate,
exactly as instructed.

## Other incidental notes

- `scripts/count_processes.py` gained PyYAML as a dependency (already used
  by `scripts/compare_models.py`, same defensive import-with-message
  pattern).
- Fixed a `just`-parser gotcha while writing the `autoware/justfile`
  `count-processes` model-based rewrite: a multi-line `python3 -c "..."`
  heredoc-style string in a recipe body breaks `just`'s recipe-line-
  indentation parsing (each unindented line reads as a new top-level
  directive) — collapsed to a single-line `python3 -c "...; ...; ..."`
  instead.
- The nested-worktree `cargo metadata` quirk under `tests/` (this worktree
  living inside `.claude/worktrees/...` inside the main repo's own
  workspace) needed the documented temporary `[workspace]` shim in
  `tests/Cargo.toml` to run `cargo build`/`clippy`/`nextest` directly (not
  through `just`, which shells out per-recipe and doesn't hit this). Added
  before verification, reverted before every commit below.
