# P45 W3 Report — resolve embeds sched structure into the model (45.4)

**Branch:** `feat/p45-resolve-embed`, created from freshly-fetched `origin/main`
(`5aba11c`, which already carries W2's merged schema — submodule
`src/ros-launch-manifest` at `5016b4d`). Isolated worktree
(`.claude/worktrees/agent-adb05dec8a839bb95`).

**Submodule:** untouched — `src/ros-launch-manifest` stays at `5016b4d` (the
commit the brief said should already be on main). No gitlink bump needed;
this wave only *consumes* the W2 schema from the parent (`play_launch`) side.

**Files changed** (parent repo only):
- `src/play_launch/src/ros/sched_loader.rs` — `DerivedSchedPlan` gained 3
  fields (`chains`, `nodes`, `ranks`); `derive_sched_plan` populates them by
  moving out of the one `MapperInput`/`MapDiagnostics` it already built.
- `src/play_launch/src/ros/model_builder.rs` — `build_system_model` embeds
  `execution.sched` from those 3 new fields + `s.derived.mapper`.
- `tests/tests/rt_workspace.rs` — new golden test
  `resolve_embeds_sched_structure_matching_check_explain`.

## What `resolve` now writes

`execution.sched` (the W2 `ExecutionSched`) is populated whenever a platform
file resolves (same `if let Some(s) = sched` branch that already fills
`tiers`/`bindings`); it stays `None` when no `--sched`/overlay/provider
platform file is found — no schema/behavior change on that path.

- **`chains: Vec<ResolvedChain>`** — `s.derived.chains`, carried straight
  through from `MapperInput::chains` (`sched_derive::resolve_chains`'s
  output). Embedded verbatim, not re-derived.
- **`requirements: Vec<NodeSchedRequirement>`** — one entry per
  `MapperNode` in `s.derived.nodes` (`MapperInput::nodes`) that has at least
  one declared path or a criticality (`n.criticality.is_some() ||
  !n.paths.is_empty()`). Nodes with neither (e.g. `perception_container`,
  which owns no contract-declared path) are skipped — a `NodeSchedRequirement`
  row with nothing in it would just be `node_fqn` noise, and
  `ExecutionSched`/its fields are all `skip_serializing_if`-guarded for
  exactly this reason. `node_fqn` = `MapperNode::name`; `criticality` and
  `paths` are moved/cloned straight from the `MapperNode` (already exactly
  the "trigger / deadline / budget" per-path facts + node-level criticality
  the design calls for — no reshaping).
- **`mapper: Option<String>`** — `Some(s.derived.mapper.clone())`, always set
  whenever a platform file resolved (including `manual`/`rate_monotonic`/
  `deadline_monotonic`, not just `chain_aware` — provenance is useful
  regardless of which mapper ran).
- **`ranks: Vec<ChainAwareDetail>`** — `s.derived.ranks`, straight from
  `MapDiagnostics::details` (the `chain_aware` mapper's own PiCAS ranks,
  pre-override). Empty for other mappers, since only `chain_aware` populates
  `details`.

`execution.sched` is wrapped in `!exec_sched.is_empty()` before being
assigned — in practice this is always `true` whenever a platform file
resolved (since `mapper` is always `Some`), but it's the same defensive
pattern the design doc describes ("all four fields independently
optional/empty").

## Single-derive wiring (deliverable 4)

No second derivation was introduced. `sched_loader::derive_sched_plan`
already built exactly one `MapperInput` (`input`) and ran the mapper once
(`mapper.map_with_diagnostics(&input, ...)` → `(derived, diagnostics)`) to
produce `tiers`/`bindings`' source data. The only change there is that,
**after** `input`/`diagnostics` are done being borrowed (contradiction
scans, `build_provenance`), their `chains`/`nodes`/`details` fields are
*moved* into the returned `DerivedSchedPlan` instead of being dropped:

```rust
Ok(DerivedSchedPlan {
    plan, target: file.target, mapper: file.mapper, warnings, provenance,
    chain_member_nodes, suppressed_contradictions,
    chains: input.chains,      // was dropped before; now carried out
    nodes: input.nodes,        // was dropped before; now carried out
    ranks: diagnostics.details, // was dropped before; now carried out
})
```

`model_builder::build_system_model` then reads `s.derived.{chains,nodes,mapper,ranks}`
— the exact same `DerivedSchedPlan` value `resolve.rs` already passes in as
`SchedInputs::derived` for the `tiers`/`bindings` block just above it, in the
same `if let Some(s) = sched { ... }` scope. There is no second call to
`derive_sched_plan`, `resolve_chains`, or the mapper anywhere in this diff.

## FQN identity handling (deliverable 2)

Every FQN written into `execution.sched` already comes from the same two
identity sources `bindings`/`tiers` use — no fourth FQN builder was
introduced:

- `requirements[].node_fqn` and `ranks[].node` originate in
  `MapperNode::name`, built by `scheduled_records_from_dump`
  (`sched_loader.rs`, the same function that produces the plan's node set —
  `t.members` — which `bindings` is keyed by).
- `chains[].elements[].node` (`Boundary::node` / `SegmentNode::node`)
  originates in `resolve_chains` → `resolve_segment`'s contract-side
  resolution (`manifest_loader::qualify_name`), same as before this wave —
  this is the identity `chain_member_nodes` (already used for the
  cross-scope-mismatch warning at derive time) has always used.

**`model_builder::fqn` is NOT in this write path.** It is only used to build
`structure.nodes` keys from `dump.node`/`dump.container`/`dump.load_node`
records directly (own `namespace=` or `"/"`, no scope-ns fallback) — a
separate concern from the sched pipeline's own FQN builder
(`sched_loader::effective_ns`, which *does* have the scope-ns fallback and is
what `scheduled_records_from_dump` already uses for `bindings`/`tiers`). Since
`execution.sched`'s new fields reuse the sched pipeline's existing identity
(same as `bindings`), they inherit whatever correctness `bindings` already
had — this wave does not make FQN identity better *or* worse than the
pre-45.4 `bindings`/`tiers` state.

The `rt_workspace` fixture (all 3 rt_demo nodes use explicit `namespace=`
attributes) doesn't exercise the `model_builder::fqn` scope-ns-fallback bug,
so `structure.nodes`, `execution.bindings`, and the new `execution.sched`
all agree on FQNs in the pasted YAML below and in the golden test's
assertions (`structure_nodes.contains_key(fqn)` /
`bindings.contains_key(fqn)` for every `requirements`/`ranks` entry passes).
**I did not touch `model_builder::fqn`** — per the brief, fixing it is
in-scope only if "in the write path" for this wave; it isn't (the sched
pipeline never calls it), so this is a documented deferral, not a fix,
per the brief's own fallback clause. See "Notes for W4" below — the known
contract-side-vs-launch-dump-side chain FQN divergence (documented in
`sched_loader.rs`'s "Defensive FQN-consistency check", pre-existing, already
warns loudly) is the more consequential of the two identity gaps and is
already tracked as a follow-up in that file's own doc comment.

## Golden test (deliverable 3)

`tests/tests/rt_workspace.rs::resolve_embeds_sched_structure_matching_check_explain`
runs `play_launch resolve rt_demo bringup.launch.xml --sched
launch/bringup.system.posix.yaml` (the `chain_aware` / `points_to_cmd`
showcase fixture) and asserts:

1. `execution.sched.mapper == "chain_aware"`.
2. `execution.sched.chains` contains `points_to_cmd`, decomposed into
   1 `Boundary` (`sensor_node.tick`) + 1 `Segment`
   (`filter_component.filter` → `control_node.control`, in
   drain-toward-sink order).
3. `execution.sched.requirements` has exactly 3 entries (the 3 rt_demo
   nodes with contract facts — `perception_container` is correctly
   excluded, no facts), each `node_fqn` present in both `structure.nodes`
   and `execution.bindings`; `control_node`'s requirement carries
   `criticality: high` and `max_latency_ms: 10.0`.
4. `execution.sched.ranks` has exactly 3 entries, each `node` present in
   `structure.nodes`, each `provenance` citing `chain_aware`.
5. **Model-vs-derive consistency**: runs `check --sched --explain` on the
   *same* inputs and asserts every chain-member FQN embedded in the model
   appears in `--explain`'s output labeled either `chain_aware`-derived or
   `override` (the two legitimate final-provenance labels a chain member can
   carry — `control_node` is overridden to priority 20 in
   `bringup.system.posix.yaml`, so its `--explain` row shows
   `override(control_node)` while its embedded `ranks` entry still shows the
   pre-override `chain_aware` rank of 40 — both are asserted, at the layer
   where each is the correct label).

## Real `execution.sched` YAML (deliverable, pasted)

Produced by `play_launch resolve tests/fixtures/rt_workspace/launch/bringup.launch.xml
--sched tests/fixtures/rt_workspace/launch/bringup.system.posix.yaml
--contracts tests/fixtures/rt_workspace/contracts` against the built
`rt_workspace` fixture (`cargo build -p play_launch`'s `target/debug/play_launch`,
direct launch-file-path mode — no colcon install/ needed for this manual
check; the golden test above uses the colcon-installed binary via
`package/launch_file` mode):

```yaml
execution:
  tiers:
    /control/control_node:
      class: real_time
      posix:
        priority: 20
        core: 0
        sched_class: SCHED_FIFO
    /perception/filter_component:
      class: real_time
      posix:
        priority: 39
        sched_class: SCHED_FIFO
    /perception/sensor_node:
      class: real_time
      posix:
        priority: 38
        sched_class: SCHED_FIFO
  bindings:
    /control/control_node: /control/control_node
    /perception/filter_component: /perception/filter_component
    /perception/sensor_node: /perception/sensor_node
  sched:
    chains:
    - name: points_to_cmd
      criticality: high
      max_latency_ms: 30.0
      semantics: reaction
      elements:
      - kind: boundary
        value:
          node: /perception/sensor_node
          path: tick
          period_ms: 10.0
      - kind: segment
        value:
          nodes_in_topo_order:
          - node: /perception/filter_component
            path: filter
          - node: /control/control_node
            path: control
    requirements:
    - node_fqn: /perception/sensor_node
      paths:
      - name: tick
        effective_trigger:
          kind: timer
          value:
            rate_hz: 100.0
        outputs:
        - points_raw
    - node_fqn: /control/control_node
      criticality: high
      paths:
      - name: control
        effective_trigger:
          kind: input
          value:
          - points_filtered
        max_latency_ms: 10.0
        inputs:
        - points_filtered
        outputs:
        - cmd
    - node_fqn: /perception/filter_component
      paths:
      - name: filter
        effective_trigger:
          kind: input
          value:
          - points_raw
        max_latency_ms: 5.0
        inputs:
        - points_raw
        outputs:
        - points_filtered
    mapper: chain_aware
    ranks:
    - node: /control/control_node
      path: control
      priority: 40
      provenance: 'derived(chain_aware: points_to_cmd segment drain 1/2) -> prio 40'
    - node: /perception/filter_component
      path: filter
      priority: 39
      provenance: 'derived(chain_aware: points_to_cmd segment drain 2/2) -> prio 39'
    - node: /perception/sensor_node
      path: tick
      priority: 38
      provenance: 'derived(chain_aware: points_to_cmd boundary RM period=10ms) -> prio 38'
```

`ranks[].priority` (40/39/38, the mapper's own pre-override PiCAS rank) is
deliberately different from `tiers./control/control_node.posix.priority`
(20, the override-applied final value) — that's the documented "structure
vs. Linux realization" split working as designed: `bindings`/`tiers` are the
*applied* plan, `ranks` are the `chain_aware` mapper's own ranking fact.

Cross-check via `check --sched --explain` on the identical inputs (same
`derive_sched_plan` call, independently invoked):

```
FQN                               CLASS        PRIO  CORE  PROVENANCE
/perception/filter_component      SCHED_FIFO     39     -  derived(chain_aware: points_to_cmd segment drain 2/2) -> prio 39
/perception/sensor_node           SCHED_FIFO     38     -  derived(chain_aware: points_to_cmd boundary RM period=10ms) -> prio 38
/control/control_node             SCHED_FIFO     20     0  override(control_node)
/perception/perception_container  SCHED_OTHER     0     -  default (no timing facts)
```

Chain membership (sensor_node/filter_component/control_node), priorities
(39/38, 20-post-override), and provenance text all agree exactly with the
embedded model.

## Verification (real output)

```
$ cargo build -p play_launch --manifest-path src/play_launch/Cargo.toml
Finished `dev` profile [unoptimized + debuginfo] target(s) in ~30s   (exit 0)

$ cargo clippy -p play_launch --manifest-path src/play_launch/Cargo.toml --all-targets -- -D warnings
1 pre-existing error (clippy::manual_map in src/play_launch/src/runtime_enforcement/mod.rs:781,
  unrelated to this change — confirmed present identically on a `git stash`'d
  (pre-45.4) tree). No new clippy findings in the touched files.

$ cargo test -p play_launch --manifest-path src/play_launch/Cargo.toml
220 passed; 0 failed   (includes existing sched_derive/sched_loader unit tests, unmodified)

$ cd tests/fixtures/rt_workspace && just build
Finished <<< rt_demo [8.70s]   (colcon build of the rt_demo package)

$ just build-rust
Finished <<< play_launch [41.9s]   (refreshes install/play_launch/lib/play_launch/play_launch)

$ cd tests && cargo nextest run -E 'binary(rt_workspace) | binary(manifest_check) | binary(sched_apply)'
Summary: 32 tests run: 32 passed, 0 skipped
  (includes the new resolve_embeds_sched_structure_matching_check_explain)

$ cargo +nightly fmt --check / cargo fmt --check on the 3 touched files
No diff attributable to my changes (sched_loader.rs: clean; model_builder.rs:
the one pre-existing long-line diff at the untouched `lower_params` fn,
confirmed via `git diff` that I never touched that region; rt_workspace.rs:
clean except one pre-existing diff at a line I didn't add, in
check_export_graph_json_matches_contract).
```

### Environment note: nested-worktree cargo workspace quirk (not committed)

This worktree lives at `.claude/worktrees/agent-adb05dec8a839bb95/` *inside*
the main checkout's directory tree. `cargo`'s ancestor-based workspace-root
search, when invoked from `tests/` (which is `exclude`d from the inner
workspace root's `[workspace]` table), walks *past* the inner worktree root
and finds the **outer** main checkout's `Cargo.toml` — which doesn't
recognize the nested `tests/` package either (different relative path) —
producing `error: current package believes it's in a workspace when it's
not`. This only happens because of the nested-worktree layout; it does not
happen in a normal (non-nested) checkout. Workaround used **only locally,
for running verification**, never committed: temporarily added an empty
`[workspace]` table to the top of `tests/Cargo.toml` (cargo's own suggested
fix), ran the nextest suites, then reverted the file (`git checkout --
tests/Cargo.toml`) before finishing. Final `git status`/`git diff --stat`
confirm only the 3 intended files are modified — no `tests/Cargo.toml`,
`Cargo.lock`, or submodule Cargo.lock changes are part of this diff (those
also churned from building against this host's locally-installed ROS
message-package versions, which differ in patch version from what the
repo's lockfiles pin — reverted, not committed). **Flag for W4/whoever runs
the same command next**: if you hit this exact error in this or a similarly
nested worktree, this is the fix; it's a harness artifact, not a repo bug.

## Notes for W4 (`from_model` reader + `--explain`)

- `execution.sched` is now populated end-to-end by `resolve`; W4's job is to
  make `SchedPlan::from_model`/`--explain` **read** it instead of
  re-deriving. Concretely: `chain_member_nodes` (today recomputed at replay
  time via `sched_derive::chain_member_fqns(index)`, re-parsing the
  manifest index) can instead walk `model.execution.sched.chains` directly —
  no manifest index needed off-host. Same for the colocation warning.
- `execution.sched.ranks`/`requirements` are exactly what a model-reading
  `--explain` needs to render provenance without re-deriving: `ranks` gives
  the chain_aware pre-override line per (node,path); `tiers`/`bindings` (already
  read today) give the post-override applied value; `requirements` gives
  the trigger/deadline/budget/criticality facts `describe_derived_fact`
  needs for non-chain_aware mappers. The existing `Provenance` enum
  (`sched_loader.rs`) and `render_explain` renderer are candidates for the
  "same renderer, one code path" the design doc asks for — they'd need a
  `From<&ExecutionSched>`-shaped input instead of a live `DerivedSchedPlan`,
  but the *data* is all there now.
- **Known pre-existing FQN gap, not closed this wave** (documented above):
  `resolve_chains`' contract-side FQN resolution (`manifest_loader::qualify_name`)
  can diverge from the launch-dump-side FQN (`scheduled_records_from_dump`)
  for a node using a bare `namespace=` attribute outside a `<group>`/
  `<include>` boundary — `sched_loader.rs` already emits a loud warning when
  this happens (`"chain member ... does not match any schedulable node"`),
  and that warning is preserved unchanged by this wave (it fires before
  `execution.sched` is even embedded, since it's inside `derive_sched_plan`
  itself). If W4 (or later) unifies FQN builders per the design doc's "FQN
  identity — one builder" section, `execution.sched.chains`' node identities
  will automatically benefit — no separate fix needed in the sched-embedding
  code added this wave.
- `model_builder::fqn`'s missing scope-ns fallback (the `structure.nodes`-only
  bug, same class as the 44.6 `vehicle_cmd_gate` fix) is **not** in the sched
  write path and was deliberately left untouched — see "FQN identity
  handling" above. It only matters for `execution.sched` if/when a future
  consumer cross-references `requirements[].node_fqn`/`ranks[].node` against
  `structure.nodes` keys for a node that hits that bug (none do in
  `rt_workspace`/Autoware fixtures tested here). Worth a dedicated
  regression test + fix in whichever wave unifies the FQN builders, since
  fixing it here would have been an untested, out-of-scope drive-by change.
- Considered but not attempted this wave (per brief, "optional if time"):
  running `resolve` against Autoware's `planning_simulator` + a chain_aware
  platform file. Skipped for time; the `rt_workspace` golden test already
  exercises the full `chains`/`requirements`/`mapper`/`ranks` shape plus the
  model-vs-`--explain` consistency check, which is the deliverable's actual
  correctness bar.
