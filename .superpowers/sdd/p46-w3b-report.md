# P46 W3b Report — spawn from the model (46.3b, load-bearing)

Commit: `90ee3b0` on `worktree-agent-aba387e0918adac9f` (branch name assigned
by the worktree harness; the brief's suggested `feat/p46-spawn-from-model`
name wasn't available to request). Built on top of 46.3a (`7c7038a`,
`a87cbc8`, submodule `ros-launch-manifest` @ `0fefe17`, unchanged by this
wave — play_launch-only, as scoped). Not pushed.

## Goal recap

Make `play_launch replay --model <m.yaml>` build the spawn context (exec
path, argv, injected env, materialized param files, LoadNode requests) from
`model::NodeInstance` instead of the LaunchDump `NodeRecord`, additively —
the record path stays the default and fully intact.

## Retarget design

**Adapter over the existing pipeline, not a reimplementation.** The W3
analysis's core finding held: `node_cmdline.rs`'s ~380-line argv assembly
(`from_node_record`/`to_cmdline`/`to_command`, ament resolution, params-file
materialization, overrides.yaml) is already well-tested and source-agnostic
in spirit — it just needed a `&model::NodeInstance` entry point.

- **`node_record_from_instance(fqn, inst) -> NodeRecord`**
  (`execution/node_cmdline.rs`) builds a synthetic `NodeRecord` from a
  model FQN + `NodeInstance`:
  - `split_model_fqn(fqn)` inverts `model_builder::fqn`'s forward join
    (`/ns/name` -> `("ns", "name")`, `/name` -> `("", "name")`).
  - `param_value_to_record_string` renders typed `ParamValue` back to the
    resolved-string form `from_node_record`'s `str_to_yaml` re-infers types
    from — the one subtle correctness trap: `f64::to_string()` drops the
    decimal point for whole numbers (`50.0.to_string() == "50"` in Rust),
    which would silently re-infer as an Integer. Forces the decimal point
    back on. Covered by
    `node_cmdline::tests::test_param_value_to_record_string_preserves_float_type`.
  - GAP-4 (global+node-specific param merge) was already resolved at
    model-build time (46.3a); `global_params: None` here, nothing left to
    merge. GAP-3 (`params_files`) and GAP-5 (`raw_cmd`) carry through
    verbatim. GAP-2 (`args`)/ros_args pass through as-is (empty today per
    the parser gap, see below).
- **`NodeCommandLine::from_node_instance(fqn, inst, params_files_dir,
  variables)`** = `node_record_from_instance` + `Self::from_node_record`,
  the brief's requested parallel entry point. `variables` is passed through
  empty in practice (the model is early-bound, no `$(var …)` should
  survive), kept for signature symmetry. Not called from `context.rs`
  (which needs the intermediate `NodeRecord` too, and containers mutate it
  before building the cmdline) — `#[allow(dead_code)]` with a comment
  explaining why; exercised by the equivalence gate.
- **`execution/context.rs`**: three model-sourced siblings —
  `prepare_node_contexts_from_model`, `prepare_container_contexts_from_model`,
  `prepare_composable_node_contexts_from_model` — mirroring the record-path
  functions' dedup/output-dir/metadata.json conventions exactly, and
  applying the identical `--container-mode` package/executable override.
  `NodeContext`/`ComposableNodeContext` gained a `model_fqn: Option<String>`
  field (`None` on the record path) so `replay.rs`'s sched-tier lookup uses
  the model's own already-known FQN directly (`p.for_fqn(&fqn)`) instead of
  recomputing via `sched_loader::fqn_for(&launch_dump, …)`, which needs a
  `LaunchDump` numeric scope id the model path doesn't carry. This is the
  "route through the single source of truth, not a fourth FQN builder"
  instruction from the brief — since `structure.nodes` keys ARE already the
  reconciled FQN (computed once, at model-build time, in
  `model_builder.rs`), the consume side needs no FQN computation at all,
  just to carry the already-known string through.
- **Composable-node container matching**: reused
  `model_builder::resolve_node_ref` (made `pub(crate)`) — the exact
  exact-match + unique-bare-name-suffix-fallback algorithm — instead of
  reimplementing it, closing the exact drift risk the 46.3a review (Finding
  1) flagged for a different call site.
- **`replay.rs`**: `pure_node_contexts`/`container_contexts`/
  `load_node_contexts` branch on `system_model.is_some()`; everything else
  (interception LD_PRELOAD/fd env injection, actor spawn, PGID, cmdline
  file writing) is unchanged — it all operates on the already-built
  `NodeContext`/`NodeCommandLine`, agnostic to origin. `load_launch_dump`
  stays unconditional (still feeds manifest loading, the web UI's
  `node_scope_map`, `chain_colocation_warnings_for_plan`, and the legacy
  sched path — none of those are in this wave's scope).

## A genuinely new gap the W3 analysis didn't anticipate: container vs. node classification

`model::NodeInstance` has no `is_container` flag — `model_builder.rs`'s
`dump.container` loop produces the *exact same shape* as its `dump.node`
loop (`plugin: None, container: None` either way). The W3 analysis's field
matrix only asked "can NodeInstance reproduce argv," not "can the consumer
tell a container apart from a regular node once both are just entries in
`structure.nodes`" — it can't, directly.

**Fix, without touching the submodule schema**: `model_container_fqns()`
resolves every composable node's `container` reference (exact match, else
`resolve_node_ref`'s suffix fallback) against `structure.nodes`' own keys —
a node IS a container iff some composable references it. This correctly
classifies every container in both fixtures (rt_workspace: 1/1; Autoware:
15/15, confirmed by the real-spawn smoke). The only case it misclassifies:
a `<node_container>` declared with **zero** composable nodes attached (would
fall through to "regular node," losing the `--container-mode` override) —
a degenerate pattern (why declare an empty container in a static launch?),
zero occurrences in either fixture. Documented in the code, not silently
assumed away.

## Static-equivalence evidence (the gate)

**rt_workspace** (`execution::spawn_equivalence_test`, gitignored fixture —
run `cd tests/fixtures/rt_workspace && just build` then dump+resolve, exact
commands in the file's module doc comment):

```
test execution::spawn_equivalence_test::rt_workspace_composable_node_request_fields_match ... ok
test execution::spawn_equivalence_test::rt_workspace_container_argv_matches_model_stock_mode ... ok
test execution::spawn_equivalence_test::rt_workspace_nodes_write_no_params_files_either_path ... ok
test execution::spawn_equivalence_test::rt_workspace_node_and_container_argv_env_match ... ok
```

Compares the **structured `NodeCommandLine`** (derived `PartialEq`), not
the flattened `to_cmdline()` `Vec<String>` — see "HashMap ordering" below
for why that distinction matters and is itself evidence, not a shortcut.
Tested at both `--container-mode isolated` (the default, exercises the
package/executable override) and `stock`. Composable nodes compared field-
by-field on the synthesized `ComposableNodeRecord` (package/plugin/
target_container_name/remaps/params/extra_args) — no `NodeCommandLine`
involved there by design (composables have no argv of their own).

Fixture-independent unit coverage (`node_cmdline::tests`, always runs, no
built fixture required): `test_node_record_from_instance_field_mapping`
(GAP-1..6 field mapping incl. remaps/ros_args/args/env/respawn/
params_files verbatim), `test_node_record_from_instance_root_namespace_omits_namespace`,
`test_node_record_from_instance_raw_executable` (GAP-5), and the float-
formatting risk test cited above.

**Autoware** (119 nodes, real install, `--model` off the fresh
`resolve --record record.json -o system_model.yaml`): not run through the
in-crate equivalence-gate test (that's rt_workspace-scoped per the brief),
but through the heavier real-spawn smoke below, which is a strictly
stronger check (it exercises the actual `ament_index` resolution, real
process spawn, and container LoadNode flow end to end).

## Real-spawn cmdline diff

### rt_workspace

Ran `replay --input-file record.json` and
`replay --input-file record.json --model system_model.yaml` back to back
(`--sched-apply off --enforce-rules off --disable-all`), diffed
`play_log/<ts>/node/*/cmdline`:

```
--- control_node ---
< ... --ros-args -r __node:=control_node -r __ns:=/control
> ... --ros-args -r __ns:=/control -r __node:=control_node
--- perception_container ---
IDENTICAL
--- sensor_node ---
IDENTICAL
```

**Root cause, confirmed empirically, not assumed**: `NodeCommandLine.remaps`
is a `HashMap<String,String>`; `to_cmdline()` iterates it in whatever order
its `RandomState`-seeded buckets land the two keys. Re-ran the record path
**twice** (no model at all) and diffed run1 vs run2: all three files came
back byte-identical — so it isn't "randomness" in the sense of differing
run to run for the *same* code path; it's that the model path constructs
its `HashMap`s in a different *sequence* than the record path (model
iterates `structure.nodes`, a `BTreeMap` sorted by FQN; record iterates
`launch_dump.node`/`.container`, declaration order), and each `.collect()`
call draws a fresh random seed from the position it lands at in that
sequence — so the 2-element map for `control_node`'s remaps happens to
hash into a different bucket order between the two paths, while
`sensor_node`'s and `perception_container`'s happened to match by chance.
This is a documented, pre-existing property of `NodeCommandLine.remaps`
(present before 46.3b), functionally inert (`-r __node:=X -r __ns:=Y` vs
the reverse parse identically in rcl), and exactly the "diff is
explainable/benign, e.g. ordering" case the brief pre-authorized. It's why
the equivalence-gate test compares the **struct** (`assert_eq!(cmdline,
cmdline)`, `HashMap`/`HashSet` fields compare by content) rather than the
flattened argv `Vec` — the struct comparison is strictly correct; the raw
argv diff is a real but content-empty divergence.

### Autoware (119 nodes, real spawn, both paths)

Both paths: **49/49** processes spawned (34 nodes + 15 containers), **70/70**
composables loaded (`grep -c "loaded as"` across all container stderr),
**0** `LOAD_FAILED`. Clean PGID shutdown both times, `rm -f
/dev/shm/fastrtps_*` after. (rviz2 crash-loops identically on both paths —
`qt.qpa.xcb: could not connect to display`, no `$DISPLAY` in this headless
box; confirmed present at the same rate on the record-only path too, purely
environmental, not a regression.)

Diffed all 49 `cmdline` files (normalizing each run's own temp `output_dir`
prefix out of `--params-file` paths first, content not path). Categorized
programmatically (same multiset of argv tokens = benign/order; different
multiset = real):

- **12/49 byte-identical.**
- **23/49 pure ordering/path-only** (same `HashMap`-ordering artifact
  above, now also touching `-p`/`--params-file` token order for nodes that
  actually have params).
- **14/49 real content differences** — all fully explained, two distinct,
  both **found via this exact diff, not anticipated in advance**:

**(a) 10 nodes** (`aggregator_node`, `autoware_map_projection_loader_node`,
`autoware_perception_analytics_publisher_node`,
`autoware_pose_initializer_node`, `autoware_simple_planning_simulator_node`,
`autoware_vehicle_door_simulator_node`, `converter_node`,
`initial_pose_adaptor_node`, `routing_adaptor_node`,
`service_log_checker_node`) — model path has an **extra** `__node:=<name>`
remap the record path omits. This is exactly the documented `name=None`
divergence (see below) — **10/119**, matching the 46.3a review's
independent prediction ("Autoware: ~10/119 name=None nodes") exactly.

**(b) `container`/`container_2`/`container_3`** (3 Autoware containers all
literally named `"container"`, in 3 different namespaces:
`/adapi`, `/autoware_api/external/rtc_controller`,
`/system/component_state_monitor`) and **(c) `pointcloud_container`** — see
"Node classes that diverged" below; (c) was a **real bug, found and fixed**
during this verification (not a pre-accepted divergence).

## Node classes that diverged (do not silently paper over — per instructions)

### 1. `name=None` nodes: `__node` remap always set (accepted, documented)

`from_node_record` deliberately **omits** `__node` when the launch declared
no `name=` (`af7c524`, "Use None for the node name if it's not set instead
of exec_name fallback") so LifecycleNodes keep their internally-declared
default name instead of being forced onto `exec_name`. The model's
`structure.nodes` key is a *reconciled* FQN (46.3a's `name.or(exec_name)`
fallback) — it carries no signal for *which* of the two supplied the value.
`node_record_from_instance` cannot reconstruct "was `name=` declared" from
the FQN alone, so it always sets `name: Some(..)`, which always emits
`__node`. **Functional risk**: narrow — only matters for a `name=None` node
whose internal default name differs from `exec_name` (LifecycleNodes with a
custom internally-declared name); for anything where `exec_name` already
matches the node's real default, forcing `__node` is a no-op. Verified
empirically at exactly the predicted Autoware count (10/119); zero
occurrences in rt_workspace (no `name=None` nodes there, so the mandatory
gate never exercises this). **Not closed in this wave** — per the brief's
"play_launch-only" scope, closing it properly needs a submodule schema
field (e.g. `explicit_name: bool` or a raw `name: Option<String>` alongside
the FQN key) — out of scope here, flagged for 46.4/46.5 or a small
dedicated follow-up.

### 2. Same-literal-name containers in different namespaces: dedup-suffix label instability (accepted, documented, found empirically)

Three Autoware containers are all declared with the exact same `name`
attribute (`"container"`) in three different namespaces. Both paths
disambiguate identically-named siblings with a `_2`/`_3` suffix, but the
**order** they assign that suffix in differs: the record path iterates
`launch_dump.container: Vec<_>` (launch declaration order); the model path
iterates `structure.nodes: BTreeMap<String, _>` (FQN-sorted order) — there
is no preserved "declaration order" in the model to match against.
Re-verified this is **deterministic, not random**: re-ran the model path
twice, got the identical `container`->`/adapi`,
`container_2`->`/autoware_api/external/rtc_controller`,
`container_3`->`/system/component_state_monitor` mapping both times — it's
a stable but *different* convention from the record path's, not noise.
**Functional impact**: each container still spawns with its own correct
namespace/package/executable/args regardless of which numbered label it
lands under (verified — the composable-container matching, dedup, and
argv-per-container are all independently correct in both diffs above);
`sched_plan.for_fqn` lookups use `model_fqn` (the real FQN) directly, not
the ambiguous member-name label, so RT-scheduling correctness is
unaffected. The only user-visible effect is cosmetic/informational: the
`play_log/.../node/container_N/` directory name and the parameter-service
member-name (Phase 24, name-keyed lookup) for one of these three containers
can point at a different namespace under `--model` than under the record
path. **Not closed** — would need the model to preserve original
declaration order (another schema gap, out of this wave's scope); zero
occurrences in rt_workspace (only 1 container, no name collision).

### 3. Container root-namespace `__ns` omission — real bug, found and fixed

`node_record_from_instance`'s general "collapse root FQN's namespace to
`None`" rule (correct for regular `<node>`s, where `NodeRecord.namespace`
is genuinely `Option`) is **wrong for containers**:
`NodeContainerRecord.namespace: String` is not optional, and
`prepare_container_contexts` (the record path) always wraps it
`Some(container_record.namespace.clone())` — so the record path emits
`__ns` for *every* container, including root-namespaced ones (`-r
__ns:=/`). Found via the Autoware real-spawn diff (`pointcloud_container`,
root-namespaced): record path kept `-r __ns:=/`, model path dropped it
entirely. **Fixed** in `prepare_container_contexts_from_model` — forces
`record.namespace = Some(record.namespace.unwrap_or("/".to_string()))`
after calling `node_record_from_instance`, unconditionally, matching the
record path's always-`Some` behavior. Re-verified after the fix:
`pointcloud_container`'s cmdline now carries `__ns:=/` on both paths.
rt_workspace has no root-namespaced container, so this bug (and its fix)
only shows up against Autoware — a concrete example of why the brief asked
for the Autoware smoke, not just the rt_workspace gate.

## `ros_args` disposition

Consumed correctly end to end (`NodeInstance.ros_args: Vec<String>` ->
`node_record_from_instance` -> `Some(vec)` when non-empty -> `from_node_record`'s
`--ros-args <args> --` block) — plumbing is right, but **empty for every
real record** in both fixtures (rt_workspace and Autoware): the Rust XML/
Python-bridge parser hardcodes `None` at every construction site
(`record/generator.rs`, `actions/container.rs`, `python/bridge.rs` — the
gap `.superpowers/sdd/p46-w2-review.md` already flagged, unrelated to this
wave, "fix Rust not Python" per CLAUDE.md). No regression either way:
record path also gets `None` today. Did not attempt the parser fix (cross-
cutting, separate wave); the synthetic unit test
(`test_node_record_from_instance_field_mapping`) hand-constructs a
populated `NodeInstance.ros_args` to prove the plumbing works despite no
real fixture exercising it, per the W3 analysis's own recommendation.

## Verification run

- **Static equivalence gate** (the mandatory first gate): GREEN, see above.
- `cargo test -p play_launch --bin play_launch` (ROS humble + repo
  `install/setup.bash` + `tests/fixtures/rt_workspace/install/setup.bash`
  sourced, the last one needed only so the equivalence gate's ament lookup
  for `rt_demo` succeeds): **245 passed, 0 failed**. This wave adds 9 new
  tests: 4 in `execution::spawn_equivalence_test` (the fixture-driven
  equivalence gate) + 5 fixture-independent unit tests in
  `node_cmdline::tests` (`test_split_model_fqn`,
  `test_param_value_to_record_string_preserves_float_type`,
  `test_node_record_from_instance_field_mapping`,
  `test_node_record_from_instance_root_namespace_omits_namespace`,
  `test_node_record_from_instance_raw_executable`).
- `cd tests && cargo nextest run -E 'binary(rt_workspace) | binary(manifest_check) | binary(sched_apply) | binary(resolve_multihost) | binary(resolve_launch_fields) | binary(resolve_merge) | binary(container_events)'`
  (nested-worktree `[workspace]` shim added, ran, reverted —
  `git diff --stat -- tests/Cargo.toml` clean afterward, confirmed twice,
  once before and once after the container-namespace bugfix): **45/45
  passed** both times.
- **Real-spawn cmdline diff**: rt_workspace (record vs `--model`, both
  container modes implicitly covered by the equivalence gate; real spawn
  used isolated mode) and Autoware (119 nodes, 49 processes) — see above,
  fully accounted for (benign HashMap-order noise + the two documented
  divergences + the one bug, now fixed).
- **Autoware smoke**: ran (environment was available — Autoware 1.5.0 at
  `/opt/autoware`, sample map present). 49/49 processes, 70/70 composables
  loaded, 0 `LOAD_FAILED`, clean PGID shutdown, `/dev/shm/fastrtps_*`
  cleaned, both paths.
- `cargo clippy -p play_launch --all-targets -- -D warnings`: **one
  pre-existing failure** at `runtime_enforcement/mod.rs:781` (`manual_map`)
  — confirmed via `git diff --stat`/`git log` untouched by this wave's
  commit, last touched at `4afecb7` (Phase 43), independently reproduces
  the exact same finding the 46.3a review already logged for this same
  file. No clippy hits anywhere in my touched files (`node_cmdline.rs`,
  `context.rs`, `replay.rs`, `spawn_equivalence_test.rs`, `mod.rs`,
  `model_builder.rs`). Left `#[allow(dead_code)]` on `from_node_instance`
  (genuinely unused by production code today — see design section).
- `rustup run nightly rustfmt --check` on all touched files: clean.
  (Non-nightly `cargo fmt --check` reports drift because the project's
  `rustfmt.toml` uses nightly-only options — matches the established
  `+nightly` convention other waves used.)
- During formatting cleanup, `src/execution/rt_helper_client.rs` turned up
  modified in `git status` despite never being touched by any command I
  ran (no file list I passed to `rustfmt` included it) — reverted via
  `git checkout --` before committing; not part of this wave's diff.

## Notes for 46.4/46.5 retirement

- `load_launch_dump` is still unconditional in `replay.rs` — still feeds:
  manifest loading (`load_manifests`) for the legacy sched path and
  `ContractView::from_manifest_index` fallback when no `--model` is given;
  the web UI's `node_scope_map` (built directly from `launch_dump.node/
  container/load_node` + `launch_dump.scopes`, always record-sourced, never
  migrated this wave); `chain_colocation_warnings_for_plan(&launch_dump,
  …)` (takes `&LaunchDump` directly, unmigrated). All three could plausibly
  read from the model's `structure.nodes` FQN keys + `execution.sched`
  instead — `model.structure.nodes` already carries `scope: String` per
  node, so the scope-map migration in particular looks straightforward for
  a future wave, just out of this one's stated deliverable list.
- `--container-mode` override, interception env injection, PGID/process-
  group mechanics, `to_command`/`to_shell` — all fully source-agnostic
  already (operate on `NodeCommandLine`/`NodeContext` regardless of origin)
  and need zero changes when record.json is eventually retired.
- Two real, unclosed gaps flagged above (`name=None` `__node` ambiguity;
  container-dedup-order label instability) both trace to the same root
  cause: `model::NodeInstance`'s `structure.nodes` key is a *derived*
  identity (computed once, forward-only, at model-build time) with no
  stored provenance for "which optional field(s) contributed, and in what
  original order." Closing both cleanly would need one new, small,
  additive submodule field each (an `explicit_name: bool` and/or a
  declaration-ordinal) — natural candidates for whoever picks up the
  `is_container` gap too, since all three are the same shape of problem
  (the consume side needs a bit more provenance than the model currently
  stores).
- `NodeCommandLine.remaps`/`.params`: still `HashMap`-backed (pre-existing,
  untouched this wave) — the source of every "ordering-only" diff line
  above. A future cleanup to `BTreeMap` would make `cmdline` files
  byte-reproducible across repeated runs and paths; flagged as a
  worthwhile, low-risk, out-of-scope-for-this-wave improvement, not a
  defect this wave introduced.
