# P47 Wave B1 Report — parser-parity moved onto model comparison (47.B1, the gate)

Branch `feat/p47-parity-model`, created fresh from `origin/main` (`c9f76b6`). Not
pushed. ADDITIVE: `record.json`, `dump --format record`, and
`scripts/compare_records.py` are untouched and still work — this wave adds the
model-comparison path alongside them; it doesn't remove anything (that's
47.B2+).

## Deliverables

### 1. `scripts/compare_models.py` (new)

Compares two `system_model.yaml` SystemModel artifacts for cross-parser
functional equivalence — the model-shaped sibling of `compare_records.py`.
Full "what's compared" contract lives in the module docstring; summary:

- **`structure.nodes`** matched by FQN key (the model's own dict key — no
  name/namespace/package tuple-matching heuristic needed, which is itself a
  structural improvement: `compare_records.py`'s tuple-key matching loses
  entities to collisions — Autoware's 34 regular nodes match down to only 32
  distinct keys there; the model matches all 34).
  - **Every** node/container/composable gets a STRUCTURAL identity check:
    `pkg`, `exec`, `plugin`, `container`, `is_container`, `lifecycle`,
    `lifecycle_autostart`, `criticality`, `node_name`. This is the same rigor
    `compare_records.py` already gives containers (`containers_equivalent`:
    name/namespace) and composables (`load_nodes_equivalent`:
    node_name/target_container/plugin) — just FQN-keyed instead of
    tuple-keyed.
  - **Plain nodes only** (not a container, no `plugin`) additionally get the
    DEEP spawn-input check `compare_records.py`'s `nodes_equivalent` already
    applies to regular `<node>` entries: remaps (sorted set), env (effective
    "last wins" mapping), ros_args (sorted set), respawn/respawn_delay, args/
    raw_cmd (whitespace-normalized), extra_args, and params (`params_files`
    expanded + merged with the same asymmetric tolerance
    `nodes_equivalent` uses — a rust-only key is only a problem if it was
    inline, not file-expanded; a python-only key is only a problem if rust
    has no `params_files` to plausibly cover it — plus a new
    `canonical_param_repr` normalizer that bridges natively-typed model
    values against already-stringified ones and folds boolean case, the
    same class of cosmetic gap CLAUDE.md's "Parser Parity Status" already
    lists).
  - **Deliberate scope decision**: containers and composables do **NOT** get
    the deep spawn-input check in this wave. `compare_records.py` never
    compared container/load_node params/remaps/env either — extending
    scrutiny there is a rigor *increase*, not a faithful migration, and doing
    so surfaces real pre-existing Rust/Python composable-param divergences
    (see "Findings" below) that predate this tool and are out of scope for a
    "move the parity mechanism" wave. Matching the *existing* gate's rigor
    was the deciding principle (never break parity mid-transition).
  - `scope` (which launch scope a node originates from) is reported as a
    non-blocking **advisory** note, not a pass/fail diff — `compare_records.py`
    never compared it either (record.json's own scope cross-check is the
    separate `scripts/compare_scopes.py`, Phase 30).
- **`structure.topics`/`services`/`actions`** — compared only when either side
  is non-empty (empty on both is the common case without contract
  manifests — topic wiring comes from `ManifestIndex`, not raw launch
  remaps).
- **`contracts`** / **`execution.sched`** — compared only when either side is
  non-empty (both parsers apply these on the shared scope table, Phase
  40.1, so a plain resolve with nothing to resolve leaves both empty).
  Checked via endpoint/path/topic-contract key-set equality and
  chain/binding/tier counts when present.
- `meta` (args/inputs/resolver/diagnostics — provenance, not structure) is
  intentionally **not** compared.

Exit code 0 = equivalent, 1 = mismatch. Verbose report mirrors
`compare_records.py`'s shape (entity counts, matched/non-equivalent nodes,
per-field diffs).

### 2. Fixture justfiles (4 touched: `autoware`, `rt_workspace`, `simple_test`,
`container_events` — these are the only 4 with a pre-existing
`dump-rust`/`dump-python`/`compare-dumps` recipe; no 5th fixture has one)

Each fixture's `compare-dumps` recipe now: resolves both parsers itself
(`play_launch resolve --parser {rust,python} -o model_{rust,python}.yaml
...`, self-contained — no `dump-rust`/`dump-python` prerequisite) and runs
`scripts/compare_models.py`. The OLD record.json-based recipe is kept,
renamed `compare-dumps-record` (still calls `compare_records.py` against
`dump --format record` output) — nothing removed, `dump-rust`/`dump-python`/
`dump-both` recipes are untouched. `clean` recipes now also remove the new
`model_rust.yaml`/`model_python.yaml`.

Every new `compare-dumps` recipe explicitly puts the colcon-built binary
(`install/play_launch/lib/play_launch`) ahead of `PATH` and this worktree's
`python/` package ahead of `PYTHONPATH`, so it can never silently exercise a
stale pip install — verified this matters: the pip-installed `play_launch`
on this host predates `resolve` entirely, and the pip-installed
`play_launch` Python package (`~/.local/lib/python3.10/site-packages/`) is
dated **April 9**, well before Phase 40.1 — it would trip the fail-loud
stale-scope-path guard (`ensure_python_scope_paths`) had these recipes not
pinned PATH/PYTHONPATH to the built worktree first.

### 3. `dump_parity_bringup` (`tests/tests/rt_workspace.rs`) — switched fully
to model comparison

- New `fixtures::resolve_model()` (mirrors `dump_launch()`): runs
  `play_launch resolve --parser <p> -o system_model.yaml <pkg> [<file>]`,
  parses the YAML into `serde_json::Value` (`serde_yaml_ng` deserializes into
  any target), returns it + the backing tempdir.
- New `fixtures::compare_models()` (mirrors `compare_records()`): runs
  `scripts/compare_models.py`, returns `(success, combined_output)`.
- New `fixtures::model_entity_counts()`: classifies a model's
  `structure.nodes` into `(plain, containers, composables)` — the
  model-shaped sibling of `array_len(record, "node"|"container"|
  "load_node")`.
- `dump_parity_bringup` now: resolves both parsers, asserts 2 nodes / 1
  container / 1 composable (matching the OLD assertion's 2/1/1), and asserts
  `compare_models.py` passes. The old record-only `dump_bringup()` helper
  (only used by this one test) was removed as dead code — the brief's
  explicit "GOAL is model-based" made this a clean full switch rather than a
  parallel record-based test.

### 4. Record.json path — untouched (kept for B2 to remove)

`dump --format record`, `scripts/compare_records.py`,
`scripts/compare_parsers.sh`, every fixture's `dump-rust`/`dump-python`/
`dump-both`/(now `compare-dumps-record`) recipe, and `replay --input-file
record.json` are all unmodified and still work.

## Verification

All run from a fresh submodule checkout + from-scratch build in this
isolated worktree (`git submodule update --init --recursive`, `just
build-cpp`, `just build-rust`, `just build-interception`, `cd
tests/fixtures/rt_workspace && just build`) — confirmed the colcon-built
binary (not a stale pip install; see PATH/PYTHONPATH note above) with
`--parser python` exercising the CURRENT worktree's `python/` package.

### Autoware model-parity (the fidelity payoff)

```
$ cd tests/fixtures/autoware && just compare-dumps
...
  Entity counts (structure.nodes):
    node        : Rust= 34  Python= 34  [ok]
    container   : Rust= 15  Python= 15  [ok]
    composable  : Rust= 70  Python= 70  [ok]
    total       : Rust=119  Python=119  [ok]

  Nodes (119 matched by FQN):
    Functionally equiv:   119/119
  Topics/services/actions: 0 topic(s), equivalent (or both empty)
  Contracts:              equivalent (or both empty)
  Execution/sched:        equivalent (or both empty)

  Scope advisories (8, non-blocking):
    /control/autoware_operation_mode_transition_manager: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /control/autoware_shift_decider: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /control/control_check_container: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /control/control_container: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /control/glog_control_check_container_component: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /control/glog_control_container_component: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /control/vehicle_cmd_gate: scope: '/control' vs '/' (advisory — see scripts/compare_scopes.py)
    /simulation/dummy_perception_publisher: scope: '/simulation' vs '/' (advisory — see scripts/compare_scopes.py)

  Result: PASS (all models functionally equivalent)
```

119 nodes (34 + 15 + 70), functionally equivalent, exit 0 — the fidelity
check that must survive `record.json` removal, confirmed on models.

### Other 3 fixtures' `compare-dumps` (model-based)

- `rt_workspace`: 4 nodes (2/1/1), 3 topics, contracts + execution.sched all
  populated (this fixture ships a provider contract sidecar + posix platform
  file, auto-discovered) — PASS.
- `simple_test`: 3 nodes (0/1/2) — PASS.
- `container_events`: 3 nodes (0/1/2) — PASS.

### Rust test suite

- `cargo test -p play_launch`: 249 unit tests passed, 0 failed.
- `cd tests && cargo nextest run -E 'binary(rt_workspace) | binary(manifest_check)'`:
  34 tests passed, 0 failed, 0 skipped — includes `dump_parity_bringup`
  (model-based) and all the pre-existing resolve/sched tests.
- `just test`: 61 passed, 11 skipped, 0 failed (needed `just
  build-interception` once — unbuilt on a fresh worktree, unrelated to this
  wave).
- `just test-all`: parser unit 420 passed; integration 102 passed, 0 failed
  (includes `dump_parity_bringup`, the 3 Autoware smoke/parity/process-count
  tests, everything else).
- `cargo clippy --all-targets` (tests crate): clean after one fix
  (`map_or(true, ...)` → `is_none_or(...)`, clippy's suggestion, in the new
  `model_entity_counts` helper).
- `scripts/compare_models.py` isn't in `just check`'s lint scope (that only
  runs `ruff check python/`, not `scripts/`) — matches `compare_records.py`,
  its un-linted sibling; left in the same style (single-quoted strings,
  `typing.Dict`/`List`/`Tuple`) for consistency rather than modernizing just
  this one file.

## Findings (why containers/composables stay shallow this wave)

Attempting FULL deep comparison (params/remaps/env) on containers and
composable nodes too — before I scoped it back to match `compare_records.py`'s
existing rigor — surfaced 84 non-equivalent entities on Autoware, all
genuine PRE-EXISTING Rust/Python divergences invisible to the old tool
because `containers_equivalent`/`load_nodes_equivalent` never checked params
at all:

- Python's composable-node params sometimes carry raw duplicates (e.g. a
  param declared 4-6× across includes, same value each time — the same
  "cmd param deduplication" class of gap already documented for regular
  nodes, just never checked for composables before).
- Rust's composable params sometimes include MORE keys than Python's for the
  same node (e.g. `/adapi/node/autoware_state`: Rust has
  `check_autoware_control`, `mode`, `mrm_descriptions.*` that Python's
  inline dump doesn't carry at all) — looks like a `params_files`-merge gap
  specific to the composable-node path.
- At least one real VALUE mismatch was found (same node, `update_rate`:
  Rust `0.1` vs Python `10.0`) — not a formatting artifact, a genuine
  divergence in what value each parser resolves for that composable's
  parameter.

These are real bugs worth someone's attention (candidate 47.B1-follow-up:
"extend deep param comparison to containers/composables, then go fix what
it finds"), but fixing parser behavior is explicitly out of scope for "move
the parity mechanism" — and shipping a stricter check that turns
Autoware's long-green gate red mid-transition would violate the wave's
core constraint. Scoping the deep check back to plain nodes only (matching
`compare_records.py`'s existing tolerance) restored 119/119 PASS.

## Remaining record.json parity surface (for B2 to remove)

Per Phase 47.B §"The remaining record.json surface" — still present,
untouched by this wave:

- `dump --format record` (`commands/dump.rs`) and its `DumpFormat::Record`
  CLI variant.
- `scripts/compare_records.py` and `scripts/compare_parsers.sh`.
- Every fixture's `dump-rust`/`dump-python`/`dump-both` recipes (still
  record.json-shaped) and the newly-renamed `compare-dumps-record` recipe
  (kept as an explicit legacy path, not wired to anything by default now).
- `replay --input-file record.json` (deprecated, still spawns with a
  warning) and `resolve --record <path>` (record-reuse mode).
- `launch`'s internal record round-trip.
- `tests/src/fixtures.rs::{dump_launch, compare_records, count_expected_processes}`
  and every OTHER integration test file still calling them (this wave only
  touched `rt_workspace.rs::dump_parity_bringup` — `autoware.rs`,
  `simple_workspace.rs`, `container_events.rs`, `sequential_loading.rs`,
  `concurrent_loading.rs`, `mixed_loading.rs` etc. all still dump/compare
  via `record.json` for their own parity assertions; B2+ needs to either
  migrate those too or accept record.json's read/compat path stays a bit
  longer for them specifically).

## Deviations from the brief

- Brief said "5 fixture justfiles" — only 4 actually have a
  `dump-rust`/`dump-python`/`compare-dumps` recipe (`autoware`,
  `rt_workspace`, `simple_test`, `container_events`); no 5th fixture exists
  with this recipe set (checked all of `tests/fixtures/*/justfile`). Treated
  as a minor inaccuracy in the brief, not a missing deliverable.
- Scoped deep node-field comparison (params/remaps/env/etc.) to PLAIN nodes
  only, not all three entity kinds uniformly — see "Findings" above. This
  keeps the migration additive/non-regressive per the brief's own
  constraint; documented in the script's docstring and this report rather
  than silently narrowing scope.
- `dump_parity_bringup` was fully switched to model comparison (brief's
  "OR" alternative — keep it record-based — was not taken; the brief itself
  says "the GOAL is model-based," and keeping a parallel record-based copy
  of the exact same test would just be duplication since `record.json`
  comparison is still validated end-to-end by `just compare-dumps-record`
  and the other integration tests' record-based parity checks).
