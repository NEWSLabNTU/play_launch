# P46 W5 Report — retirement cleanup (46.5)

Branch `feat/p46-retirement`, created fresh from `origin/main` (`d83c939`).
Not pushed.

Commits:
- `5d8f278` feat(resolve): drop the record companion (46.5)
- `525e6f2` feat(dump): emit the SystemModel — one artifact (46.5)
- `dfd8391` feat(replay): deprecate record.json input (46.5)
- `2a0b3c5` docs(46.5): W5 retirement-cleanup report
- `b1e91f8` fix(dump): gate stale-python check at the parse entry point
  (46.5 review) — addresses the review's 1 Important finding (below)

## Goal recap

Make the SystemModel the one user-facing artifact — the user's "one kind of
dump" — and remove the runtime two-artifact coupling, WITHOUT breaking the
parser-parity dev infrastructure (the 100% Autoware Rust-vs-Python
`record.json` comparison) and without a hard removal of `record.json`
reading this wave.

## 1. `dump` → model convergence

`dump <launch> -o m.yaml` (no `--format`) now emits the SystemModel — the
exact same artifact `resolve` produces. Implementation:
`dump_launch_model()` (`commands/dump.rs`) builds a `ResolveArgs` from the
`dump launch` CLI args (`--parser`, plus `CommonOptions`'
`contracts`/`no_provider_contracts`/`sched`/`target`, already present on
`LaunchArgs` since it's shared with `play_launch launch`) and calls
`resolve::handle_resolve()` directly. `dump` and `resolve` share one code
path top to bottom — contract/sched channel resolution, the stale-Python
check (below), provenance hashing — nothing is duplicated.

`dump run` (single-executable dump, no launch scope tree) is explicitly
out of scope for the model conversion — it always writes record.json
regardless of `--format`, with a one-time warning if `--format model` is
requested (the default) since there's no launch tree to build a model from.

Evidence (rt_workspace, both parsers; Autoware, Rust):

```
$ play_launch dump -o m.yaml launch rt_demo bringup.launch.xml
Resolving launch file: .../bringup.launch.xml
SystemModel: m.yaml (4 nodes, 3 topics, 5 contracts-carrying endpoints, 3 tier(s), 2 warning(s))
To replay: play_launch replay --model m.yaml
$ ls m.record.json
ls: cannot access 'm.record.json': No such file or directory   # no companion
```

```
$ play_launch dump -o system_model.yaml launch --parser rust autoware_launch \
    planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning
SystemModel: system_model.yaml (119 nodes, 0 topics, 0 contracts-carrying endpoints, 0 tier(s), 0 warning(s))
real    0m0.270s
$ ls system_model.record.json
ls: cannot access 'system_model.record.json': No such file or directory
```

`replay --model system_model.yaml` (Autoware, 119-node model) spawned 49
processes and shut down cleanly on SIGTERM — the "model dump + replay
--model smoke" acceptance at Autoware scale (a full 60s+ GPU-dependent
health assessment was out of scope; this proves spawn-from-a-dump-produced-
model works, matching the rt_workspace automated test's stronger,
process-count-checked version of the same claim). rviz2's repeated "Exited
without code" during that run is the pre-existing, environment-specific
(no X display) exit already documented as an ignored case in
`test_autoware_smoke_test`.

## 2. Parity tooling preserved — choice: `--format record` escape hatch (option a)

Per the brief's options, chose (a): `dump --format {model,record}` (default
`model`). `--format record` reproduces byte-for-byte the pre-46.5 `dump`
behavior (writes record.json via the same `dump_launch_rust_wrapper`/
`dump_launch_python_wrapper` functions, unchanged). This is the dev/parser-
parity escape hatch `scripts/compare_records.py`, `just compare-dumps`, and
`dump_parity_bringup` all now use explicitly.

Why (a) over (b) (switching parity comparison to models): `compare_records.py`
is deeply record.json-shaped — cmd-string dedup, params_files expansion,
XML-whitespace normalization, array-quoting, boolean-case, param/remap
ordering — none of which the model (which carries typed `ParamValue`, not
`cmd` strings) has an equivalent representation for. Rebuilding that
normalization logic against the model's shape would be a much larger
change than this wave's scope, and would leave the actual pre-model
resolution step (parser → LaunchDump) less directly compared. (a) is a
2-line CLI addition that changes zero parity-tooling logic.

Updated call sites (all fixture justfiles' `dump-rust`/`dump-python`/
`compare-dumps`/`count-processes` recipes, the shared
`tests/src/fixtures.rs::dump_launch()` helper used by ~20 parity tests, and
6 direct `"dump"` call sites in `autoware.rs`/`simple_workspace.rs`/
`sched_apply.rs`/`rt_workspace.rs`) now pass `--format record` explicitly.

Evidence — Autoware parity (34 nodes, 15 containers, 70 load_nodes, real
Autoware 1.5.0 install):

```
$ play_launch dump --output record_rust.json --format record launch --parser rust \
    autoware_launch planning_simulator.launch.xml map_path:=...
real    0m0.253s
$ play_launch dump --output record_python.json --format record launch --parser python \
    autoware_launch planning_simulator.launch.xml map_path:=...
real    0m24.990s
$ python3 scripts/compare_records.py record_rust.json record_python.json
  Entity counts:
    node        : Rust= 34  Python= 34  [ok]
    container   : Rust= 15  Python= 15  [ok]
    load_node   : Rust= 70  Python= 70  [ok]
  Nodes (32 matched): Exact match: 0/32  Functionally equiv: 32/32
  Containers: 15/15 equivalent   Load nodes: 70/70 equivalent
  Result: PASS (all records functionally equivalent)
```

Evidence — `just compare-dumps` in rt_workspace (via the updated fixture
justfile, real `play_launch` binary, real `just dump-both`):

```
$ cd tests/fixtures/rt_workspace && just dump-both && just compare-dumps
...
  Entity counts:
    node        : Rust=  2  Python=  2  [ok]
    container   : Rust=  1  Python=  1  [ok]
    load_node   : Rust=  1  Python=  1  [ok]
  Nodes (2 matched): Exact match: 2/2  Functionally equiv: 2/2
  Result: PASS (all records functionally equivalent)
```

`dump_parity_bringup` (the automated nextest equivalent of the above) is
green — see verification section.

## 3. Companion removal

`resolve` no longer writes `<out>.record.json` next to the model in the
no-`--record` (parse-from-launch-file) path, for either parser. The Rust
path serializes straight from parser output to `LaunchDump` in memory (no
disk round-trip). The Python path writes its intermediate record.json to a
process-scoped scratch temp file (`std::env::temp_dir()`), never a
companion, and deletes it after loading.

`meta.record` is no longer populated (the `if let Some(rp) = &record_path {
model.meta.record = Some(...) }` hash-binding block is removed along with
the `verify_model_record_binding` gate it fed — that gate was already
removed in 46.4). **Design call**: I did NOT delete the `record: Option
<InputHash>` field from the shared `ros-launch-manifest-model` crate itself
— that crate lives in a separate git submodule (`src/ros-launch-manifest`,
NEWSLabNTU/ros-launch-manifest) shared cross-track with nano-ros (RFC-0050),
and this wave is explicitly play_launch-only / never-push. Since nothing
populates it anymore and it's `#[serde(skip_serializing_if =
"Option::is_none")]`, it silently disappears from every emitted model
(verified: `meta.record` is absent from real `resolve`/`dump` output below)
without a schema-touching submodule change. Fully deleting the field is a
reasonable 46.6-docs-wave or later follow-up if wanted — noted here rather
than done, to keep this wave's blast radius to the play_launch crate.

`--record <path>` (reuse mode: re-resolve from an existing record.json
someone else produced) is unaffected — that's a read path, not the
companion write this wave drops — and its path still lands in
`meta.inputs`.

Evidence:

```
$ play_launch resolve rt_demo bringup.launch.xml -o system_model.yaml
SystemModel: system_model.yaml (4 nodes, 3 topics, 5 contracts-carrying endpoints, 3 tier(s), 2 warning(s))
$ ls system_model.record.json
ls: cannot access 'system_model.record.json': No such file or directory
$ grep -c "^  record:" system_model.yaml   # meta.record key
0
```

Two new/updated automated tests assert this: `resolve_no_longer_writes_
record_companion` (new, rt_workspace) and `resolve_merges_launch_and_
contracts_into_full_model` (`resolve_merge.rs`, updated — was asserting
`model["meta"]["record"]["sha256"].is_string()`, "record companion bound";
now asserts `model["meta"]["record"].is_null()` and companion-file absence).

### Stale-Python-install fail-loud (the 46.4 report's CAVEAT)

The Python parser path detects and refuses a stale pre-Phase-40.1
`play_launch` Python install (one that predates `ScopeOrigin.path`, which
silently disables the provider-sidecar contract/sched channels with no
error otherwise). Detection: after loading a Python-produced `LaunchDump`,
if any file scope has `origin.is_some()` but `origin.path().is_none()`,
that's a reliable stale-install signal — the current source
(`python/play_launch/dump/visitor/include_launch_description.py`) always
passes `path=` to `push_scope()` for every file scope. On detection the
command `eyre::bail!`s with a message identifying the exact scope/file and
the two remediations (prepend `PYTHONPATH`, or reinstall the wheel) — a
hard error, not a warning, since a silently degraded artifact is exactly
the failure mode this exists to prevent.

**Review fix (`b1e91f8`) — the guard now gates the parse ENTRY POINT, not
just model-emit.** My first cut placed the check only on the model-emitting
path (`resolve`, and `dump` via delegation). The reviewer correctly caught
that `dump --format record` — the exact path the parser-parity tooling
(`dump-python`/`compare-dumps` in every fixture justfile,
`scripts/compare_records.py`) uses — bypassed it, so a stale parser could
silently produce a record.json that `compare_records.py` scored PASS while
reflecting the stale April parser rather than current source (reproduced
on-host). Fix: the check is now a shared `launch_dump::
ensure_python_scope_paths()` called from BOTH Python-parse sites — the
model path (`resolve.rs`) AND the record path
(`dump.rs::dump_launch_python_wrapper`, which `dump --format record` uses).
The record path loads the just-written record.json READ-ONLY for the check,
never round-tripping it through Rust's serializer, so the file the parity
comparison reads stays byte-for-byte what the Python parser wrote. Any
stale-python usage now fails loud everywhere, including the parity tooling.

Confirmation (rt_workspace, this host's live stale 0.8.2 pip install):

```
$ cd tests/fixtures/rt_workspace && just dump-python   # no current-source PYTHONPATH
...
Error: Python parser produced 1 file scope(s) without `ScopeOrigin.path`
  (["bringup.launch.xml"]) — this means a STALE `play_launch` Python install
  (pre-Phase-40.1) is shadowing the current source on PYTHONPATH. ... and a
  record.json dump would reflect the stale parser while `compare_records.py`/
  `just compare-dumps` scored it PASS. Fix: ...
error: recipe `dump-python` failed with exit code 1
```

(Previously: `dump-python` silently produced a stale record.json and
`compare-dumps` reported PASS.) With current source on `PYTHONPATH`,
`dump-both` + `compare-dumps` PASS (2/1/1 entities, functionally
equivalent). Also dropped the now-stale `#[allow(dead_code)]` +
"not yet consumed by the executor" doc on `ScopeEntry::path()` — the guard
consumes it for real (review's cosmetic finding #2).

This actually fired for real during verification: this host has `play_launch
0.8.2` (April, pre-Phase-40) pip-installed at `~/.local/lib/python3.10/
site-packages`, which the embedded PyO3 interpreter picks up ahead of
anything else unless `PYTHONPATH` is steered around it. Fixed the test
harness (`tests/src/fixtures.rs::play_launch_cmd()`) to prepend the current
worktree's `python/` source to `PYTHONPATH` — mirroring exactly what the
46.4 report's manual acceptance run did by hand — so `--parser python`
tests exercise current source rather than tripping the new check against
whatever happens to be pip-installed on the runner. This is a **production
alignment gap worth flagging**: `python_bridge.rs::setup_python_path()`
only reads the external `PYTHONPATH` env var — play_launch has no
self-locating logic to find its own bundled python source. A real user
running `dump --parser python` without a fresh `PYTHONPATH` (or a stale
pip install) will now hit this error where they previously got a silent
structure-only model — correct per the brief, but the actionable fix
(steer `PYTHONPATH`, or `pip install --force-reinstall`) is now a hard
requirement for that path rather than a nice-to-have. A follow-up could
have play_launch locate its own installed python share dir (ament-style)
and prepend it automatically instead of relying on the caller's
environment; out of scope for this wave (not requested, and the two-line
`PYTHONPATH` fix is proven to work in both the report's manual run and this
wave's test harness).

## 4. `replay`: model primary, record.json deprecated-compat

`replay --model <system_model.yaml>` (with or without `--input-file`) is
unchanged — the primary, self-sufficient spawn source since 46.3b/46.4.
`replay --input-file <record.json>` **without** `--model` — the legacy
record-only path — now prints a one-time `tracing::warn!` deprecation
message (`"record.json replay is deprecated; use the SystemModel (...)"`)
before proceeding, and still spawns exactly as before: no hard removal
this wave, per the brief.

Evidence (rt_workspace, `--format record` dump → legacy replay):

```
$ play_launch dump -o record.json --format record launch rt_demo bringup.launch.xml
$ play_launch replay --input-file record.json --disable-web-ui --disable-monitoring --disable-diagnostics
WARN play_launch::commands::replay: record.json replay is deprecated; use the SystemModel
     (`play_launch resolve`/`dump` then `replay --model <system_model.yaml>`). Continuing
     with the legacy --input-file record.json path.
INFO play_launch::commands::replay: Spawning 4 nodes (2 pure nodes, 1 containers, 1 composable nodes)
INFO play_launch::commands::signal_handler: Startup complete: all nodes ready (nodes 2/2, containers 1/1, composable 1/1)
```

(Note: play_launch's tracing output — including this warning — writes to
**stdout**, not stderr; the new automated test checks both to be robust to
that detail.) New test `replay_input_file_record_json_is_deprecated_but_
still_spawns` (rt_workspace) automates this: dumps a legacy record.json via
`--format record`, replays with `--input-file` and no `--model` from a
separate empty dir, asserts 3 processes spawn (same outcome as the
model-driven path) and that combined stdout+stderr contains "deprecated".

## 5. `file_data`/`variables` — confirmed nothing to drop

Per the design doc's own note, confirmed by reading `ros-launch-manifest-
model`'s `SystemModel`/`Structure` types (`src/ros-launch-manifest/model/
src/lib.rs`): the model schema has never had `file_data` or `variables`
fields — those are `LaunchDump`-only (the record.json / parser-cache
representation), and the model was never built by embedding a `LaunchDump`.
`model_builder::build_system_model()` reads a `LaunchDump` as input but
only lowers specific fields (`node`/`container`/`load_node`/`scopes`) into
the model's own typed structures — `file_data`/`variables` were never
copied across. Nothing to remove; noted per the brief's instruction to
confirm-and-document when there's nothing to drop.

## 6. Web UI / launch-tree scope map — accepted degrade, documented

The web UI's launch-tree/scope-map (`GET /api/launch-tree`) and the
replay-time chain-colocation warnings are still sourced from
`launch_dump.scopes`/`node_scope_map` (`commands/replay.rs`), built from a
record.json — which, since this wave, `resolve`/`dump` never produce. On
`replay --model` (the primary path now), `load_launch_dump(input_file)`
degrades to an empty `LaunchDump` (46.4's existing fallback, unchanged
logic) whenever no record companion exists at the default/explicit
`--input-file` location — which, post-46.5, is effectively **always** for
a model-only replay, not just the common case it was before. The web UI's
handler (`web::handlers::get_launch_tree`) returns `{"scopes": [],
"node_scopes": {}}` — empty, not an error; the "Launch" page's tree view
just shows nothing to expand.

**Decision (per effort)**: documented as a follow-up rather than
model-sourced this wave. The model schema *does* carry an equivalent scope
tree (`structure.scopes: BTreeMap<String, ScopeInfo>` with `parent`/
`manifest` links, and each `NodeInstance` carries its owning `scope: String`)
— cheap in principle to adapt the web UI's scope-map builder to read
`model.structure.scopes` + `model.structure.nodes[*].scope` instead of
`launch_dump.scopes` + `node_scope_map`. But the two representations differ
in shape (numeric ids + `(pkg, file, ns, args, parent)` vs string namespace-
path ids + `(parent, manifest)`, and the member-name-keyed `node_scope_map`
has no direct model equivalent — `NodeInstance` is keyed by FQN, not by the
actor system's member name convention `name.or(exec_name)`), so
"cheap" turned out to require a small but real adapter, not a one-line
change, once inspected. Given this wave's scope (already three commits
touching dump/resolve/replay + ~15 test/tooling files), I chose not to add
a fourth surface change here. **The UI does not error** — verified by
inspection of the handler (`Json(json!({"scopes": state.scopes,
"node_scopes": state.node_scope_map}))` — both fields are plain
collections, empty is a valid value, no `unwrap`/panic path) — this is
purely cosmetic degradation on the primary path, consistent with the
brief's "UI degrades, doesn't break" acceptance criterion. Left inline
code comments at both consumer sites (`commands/replay.rs`) documenting the
decision and pointing at `model.structure.scopes` as the follow-up's
starting point.

## Verification

- User-facing acceptance:
  - `dump <launch> -o m.yaml` → a model (`structure.*` present, no
    top-level `node`); `replay --model m.yaml` spawns (rt_workspace, 3
    processes: 2 nodes + 1 container) — automated in
    `dump_default_emits_model_and_replays_cleanly`. Manually verified at
    Autoware scale too (119-node model, 49-process spawn, clean SIGTERM
    shutdown) — pasted above.
  - `resolve` → model, NO `.record.json` companion — automated in
    `resolve_no_longer_writes_record_companion` and the updated
    `resolve_merges_launch_and_contracts_into_full_model`. Pasted above.
- Parity tooling STILL WORKS: `just compare-dumps` in rt_workspace passes
  (pasted above); `dump_parity_bringup` green (see suite run below);
  Autoware `dump --format record --parser {rust,python}` +
  `compare_records.py`: PASS, 34/15/70 entity counts match (pasted above).
- Deprecated compat: `replay --input-file <old record.json>` still spawns
  (3/3 processes) + prints the deprecation warning — automated in
  `replay_input_file_record_json_is_deprecated_but_still_spawns`.
- `cargo test -p play_launch`: **249 passed, 0 failed** (unit) + **21
  passed** (lib) — re-run after the `b1e91f8` review fix, still green.
- `cargo test -p play_launch_parser` (untouched crate, sanity check): **53
  passed, 0 failed**.
- `cd tests && cargo nextest run` on the full `rt_workspace`,
  `manifest_check`, `sched_apply`, `resolve_launch_fields`,
  `resolve_merge`, `resolve_multihost`, `container_events` binaries
  (build-rust first; note: the brief's literal substring filter
  `test(/rt_workspace|manifest_check|sched_apply|resolve_|
  container_events/)` matches by bare TEST NAME, not binary — it silently
  misses tests like `replay_model_spawns_without_record_companion` and all
  three of this wave's new tests whose names don't contain those
  substrings; ran full `binary_id(...)` selection instead for real
  coverage): **49 passed, 0 failed, 0 skipped** — includes all 3 new tests
  plus the updated `resolve_merge` test and the two pre-existing 46.4
  regression tests (`replay_model_spawns_without_record_companion`,
  `resolve_parser_python_produces_model_that_replays_cleanly` — the latter
  only green with the `PYTHONPATH` fix in place, since it exercises the new
  stale-install check for real on this host). Re-run after `b1e91f8`
  (which touches the record-path Python parse `dump_parity_bringup`
  exercises) — still **49 passed, 0 failed**.
- `just dump-both && just compare-dumps` in rt_workspace (review's explicit
  re-run ask): with current source on `PYTHONPATH`, PASS (2 nodes / 1
  container / 1 load_node, functionally equivalent). Without it (this
  host's stale 0.8.2 pip install), `dump-python` now FAILS LOUD instead of
  silently passing — pasted in the Stale-Python section above.
- `clippy --all-targets -- -D warnings`: clean on all touched
  `play_launch` files (`commands/{dump,resolve,replay}.rs`, `cli/
  options.rs`, `ros/launch_dump.rs`) and the whole `tests` crate — zero
  hits when grepped from a full clippy run. **Pre-existing, unrelated** clippy failure in
  `runtime_enforcement/mod.rs:781` (`manual_map`) still blocks a repo-wide
  `-D warnings` run — confirmed via `git diff origin/main -- .../mod.rs`
  (empty diff) — same pre-existing issue the 46.4 report documented, out
  of scope.
- `rustfmt --check` on all touched files: clean, except two pre-existing
  drift lines in `tests/tests/rt_workspace.rs` (lines 1142/1349 in the
  final file — the exact `1136/1333` lines the 46.4 report already
  documented as nightly-vs-stable rustfmt drift, untouched by this wave;
  confirmed by running `rustfmt --check` against `origin/main`'s copy of
  the same file, which shows identical drift at the same logical spots).
  My own new code in `rt_workspace.rs` (initially 2 more drift spots) was
  manually reformatted to rustfmt's expected output.

## Environment notes (for future waves in this worktree)

Same as 46.4's documented list — submodule init, `install`/`build`
symlinks to the shared colcon workspace, `tests/Cargo.toml`'s temporary
`[workspace]` shim (added for every nextest/clippy invocation, reverted
before every commit — confirmed absent in all three commits' diffs), and
environment-specific `Cargo.lock` drift (this host's installed ROS msgs
package versions vs. the committed lockfile) reverted before staging each
time. Additionally this wave: a stale `play_launch` 0.8.2 pip install on
this host's user site-packages is real and live (not just a hypothetical
from the 46.4 report) — the new `PYTHONPATH` fix in
`tests/src/fixtures.rs` is load-bearing for `--parser python` tests on
this specific host, not just future-proofing.

## What record.json usage REMAINS (deprecated compat surface)

- `dump --format record` — the parser-parity dev escape hatch (explicit
  opt-in, not the default). Its Python path is now covered by the
  stale-install guard (`b1e91f8`), so it can't silently emit a stale-parser
  record.json.
- `resolve --record <path>` — reuse-mode input (read an existing
  record.json instead of re-parsing a launch file); unaffected by this
  wave, still hashed into `meta.inputs`.
- `replay --input-file <record.json>` without `--model` — the deprecated
  legacy replay path; still fully functional, now with a one-time warning.
- `play_launch launch`'s internal flow (`commands/launch.rs`) still dumps
  its own record.json (via local, undelegated `dump_launch_rust`/
  `dump_launch_python` functions — separate from `commands/dump.rs`'s
  wrappers, pre-existing duplication not touched this wave) and feeds it
  through `resolve --record` (reuse mode) before replaying with both
  `--model` and `--input-file` set. This is unaffected by the companion
  removal (it never asked `resolve` to write one — reuse mode was already
  companion-free) and is out of this wave's scope; `launch` continues to
  work exactly as before.
- `scripts/compare_records.py` / `scripts/count_processes.py` and all
  parser-parity fixture justfile recipes (`dump-rust`/`dump-python`/
  `compare-dumps`/`count-processes`) — dev-only, now spelled with
  `--format record`.
- The web UI's launch-tree scope map and chain-colocation warnings are
  still `launch_dump`-only (documented follow-up in §6) — they degrade to
  empty on a model-only replay rather than reading a record.json, so this
  is not really "record.json usage" so much as "a feature not yet ported
  off record.json."

No hard removal of record.json READING anywhere this wave, per the brief's
constraint.
