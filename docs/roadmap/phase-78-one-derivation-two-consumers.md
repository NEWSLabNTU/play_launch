# Phase 78 - one derivation of the mapper input, two consumers

Status: **planned** (2026-09-21). Follows phase 70's census, which found the
seams, and phase 45 section 45.10, which chose to leave this one open. Design
of record: ros-launch-manifest `docs/design-issues.md` #52; consumer side:
nano-ros `docs/roadmap/phase-457-consume-the-shared-derivation.md`.

## Parallel plan

Four waves, each a branch and a PR, so that separate sessions can take one
each. This repository has no claim tool (nano-ros has `just claim
phase-NNN-Wk`): the claim is the branch named below, pushed with an early
draft PR, so a second session sees it before it starts. The `Status:` line
under a wave is edited in the PR that lands it. Upstream: rlm
`docs/design-issues.md` #52, whose units R1 (model fields), R2 (the `derive`
crate), R3 (parity tests and snapshot) and R4 (tag v0.1.37) precede W1.
Downstream: nano-ros phase-457 W1 depends on W4 here, the 0.12.0 tag it
bumps its gitlink to.

| wave | depends on | owns | gate | starts now? | branch |
|---|---|---|---|---|---|
| W1 pin and lower | rlm #52 R1 and R2, through the v0.1.37 tag R4 cuts | the four `tag = "v0.1.36"` pins in `src/ros-launch-resolve/Cargo.toml` and `src/play_launch/Cargo.toml`, plus the new `ros-launch-manifest-derive` line; `src/ros-launch-resolve/resolve/src/ros/model_builder.rs` (`path_contract` :299, the `input: []` lowering :954-960); `src/ros-launch-resolve/resolve/src/ros/sched_derive.rs` only to move `convert_trigger` (:229) out; golden models `tests/fixtures/rt_workspace/launch/bringup.system.posix.yaml`, `.../bringup.system.zephyr.yaml`, `tests/fixtures/rt_workspace/contracts/rt_demo/launch/bringup.system.posix.yaml`, `tests/fixtures/contract_derived_chain/launch/bringup.system.posix.yaml` | `cargo test -p ros-launch-resolve`; `tests/tests/resolve_launch_fields.rs`; Gates 3 (`trigger` on every timer path); an 0.11.0 model still loads | no: the first act is the pin bump, which needs the tag | `phase-78-W1` |
| W2 the transition gate | W1 | the `tests` module of `src/ros-launch-resolve/resolve/src/ros/sched_derive.rs` (the parity test); `src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs` only if walking every `tests/fixtures/contract_*` needs a helper | `cargo test -p ros-launch-resolve sched_derive` on every `tests/fixtures/contract_*`, `contract_derived_chain` included (Gates 1) | no | `phase-78-W2` |
| W3 delete the copy | W2 | `src/ros-launch-resolve/resolve/src/ros/sched_derive.rs` (the listed functions go), `src/ros-launch-resolve/resolve/src/ros/manifest_graph.rs` (the route copy, as used by chains), `src/ros-launch-resolve/resolve/src/ros/sched_loader.rs` (`derive_sched_plan` :704 builds the model and calls the shared function), and its two callers `src/ros-launch-resolve/resolve/src/model.rs` (:138) and `src/play_launch/src/execution/sched_plan.rs` (:149) if the signature moves | W2's test stays green with the shim gone; Gates 2 (`--explain` diff on `rt_workspace` and `contract_w1d`); Gates 4 (`git grep min_rate_hz` on `sched_*`) | no | `phase-78-W3` |
| W4 release | W3 | `CHANGELOG.md`, `version` in `src/play_launch/Cargo.toml`, the status line of this doc and its entry in `docs/roadmap/README.md`, the `v0.12.0` tag | workspace tests and `just check` on the fixtures green on the tagged commit; nano-ros phase-457 W1 bumps its gitlink to the tag | no | `phase-78-W4` |

Nothing here starts before rlm cuts v0.1.37: W1's first act is the pin
bump, and the field lowering has nothing to lower into until then. A session
may open W1's draft PR early against a `rev =` pin to R1's branch head to
get the golden re-emission ready, but it merges only on the tag. W2, W3 and
W4 are strictly sequential: each edits `sched_derive.rs` after the previous
one, which is why it is the one file three waves own.

**W1 - pin and lower.** The Cargo pins move from `v0.1.36` to `v0.1.37`
and `ros-launch-manifest-derive` joins them; `model_builder::path_contract`
lowers the six fields listed under "What this repository emits";
`convert_trigger` moves out of `sched_derive.rs`; the four golden models
are re-emitted, and a test loads one written before the fields.

Claim: `phase-78-W1`. Depends on: rlm #52 R1, R2 (tag v0.1.37, R4). Owns:
the rlm pins in `src/ros-launch-resolve/Cargo.toml` and
`src/play_launch/Cargo.toml`; `model_builder.rs`; `convert_trigger` in
`sched_derive.rs`; the four golden `*.system.*.yaml` fixtures. Gate: `cargo
test -p ros-launch-resolve`; `tests/tests/resolve_launch_fields.rs`; Gates
3. Status: not started.

**W2 - the transition gate.** The parity test over every
`tests/fixtures/contract_*` workspace, `from_dump == from_model` and
`resolve_chains_derived == resolve_chains`, as specified under Waves. It
lands green with the private copy still in place; a red row is a fact the
model failed to carry (fix in W1's files, or in rlm R1) or a rule the port
got wrong (fix in rlm R2), never a tolerance.

Claim: `phase-78-W2`. Depends on: W1. Owns: the `tests` module of
`sched_derive.rs`; `manifest_loader.rs` only for a fixture-walk helper.
Gate: `cargo test -p ros-launch-resolve sched_derive` on every contract
fixture (Gates 1). Status: not started.

**W3 - delete the copy.** `derive_sched_plan` builds the checked model and
calls `mapper_input_from_model` and `resolve_chains`; `mapper_input_from_dump`
and the functions listed under "What this repository stops doing" are
deleted, together with `manifest_graph`'s chain route. The check-side
derivations in `manifest_graph.rs` stay. Last, because W2's gate is what
says the deletion changed nothing.

Claim: `phase-78-W3`. Depends on: W2. Owns: `sched_derive.rs`,
`manifest_graph.rs` (chain route only), `sched_loader.rs`, and the
`derive_sched_plan` call sites in `model.rs` and
`src/play_launch/src/execution/sched_plan.rs`. Gate: W2's test green with
the shim gone; Gates 2 and 4. Status: not started.

**W4 - release.** 0.12.0: the CHANGELOG entry, the version bump, this doc's
status line and the README entry, the tag. nano-ros phase-457 W1 bumps its
gitlink from `07f0461e` (v0.9.0-158) to the tag.

Claim: `phase-78-W4`. Depends on: W3. Owns: `CHANGELOG.md`,
`src/play_launch/Cargo.toml` (`version`), this doc's status line,
`docs/roadmap/README.md` (the phase 78 entry), the `v0.12.0` tag. Gate:
workspace tests and `just check` green on the tagged commit; nano-ros
phase-457 W1 resolves the tag. Status: not started.

## Why

Phase 45.10 reverted the embedding of the resolved schedule into the model
(rlm `f090400`) and settled that the ALGORITHM is shared while the DERIVATION
of its input "stays per-consumer, sharing the `MapperInput` type". Measured on
2026-09-21 against this release (0.11.0, `5eaa3191`), rlm `ea5cbea` and
nano-ros `783cdfa14`, the two derivations disagree on six of the eight facts
`chain_aware_rank` ranks by. The one this repository owns is the first:

`model_builder.rs:954-960` takes a path's inputs from the EFFECTIVE trigger,
which phase 68 fixed for input-triggered paths, and lowers every other kind -
Timer, Once, Spontaneous, Unclassified - to `input: []` with no rate. The
model then says "empty = periodic" (rlm lib.rs:1130) about a `once` map loader
and a `spontaneous` service alike, and nano-ros's `mapper_input.rs:72-81`
believes it, rebuilding the timer's rate from the FIRST output's
`pub.min_rate_hz`. On the safety island that number happens to equal the
timer's `rate_hz` on all four paths, which is exactly what our own
`derivable-min-rate` info is reporting fourteen times: the schedule is right
because a redundant declaration is present.

The rest of the disagreement is ours too, in the sense that the model we emit
cannot carry the answer:

| fact | `sched_derive.rs` here | `mapper_input.rs` there |
|---|---|---|
| chains | scope paths through `manifest_graph::critical_path` (:251) | none (`chains: Vec::new()`) |
| criticality | hazard-derived first (phase 72, :496) | the label; the model has only `NodeInstance.criticality` as the raw string (:669 here) |
| `claims_concurrency` | merged groups, "none covers every path" (:195) | "some path outside every set" |
| `deadline_us` | min over `max_latency` and srv `max_response` (:462) | unset; its realizer re-derives without `max_response` |
| `exec_ms` | platform `budget_us`, one-path nodes only (:131) | `[wcet]` profile per boundary |
| `rate_hz` | max over topic `rate_hz`, derived rate, `min_rate_hz` (:424) | unset |

Two toolchains scheduling one system from different facts is the seam
`contract-axes.md` section 5 names; phases 67/68 closed it for `miss` and
`concurrency` by carrying the fact on the model. This phase closes it for the
trigger and, since carrying one more field would leave the other five rows to
drift again, moves the derivation itself into ros-launch-manifest so that both
consumers hand their realizers the same object.

## What it does

### The rule

The model carries every fact the CHECKER resolves per entity; it carries
nothing the MAPPER resolves. The derivation from the first to the second is one
function in rlm's new `derive/` crate:

```text
ros_launch_manifest_derive::mapper_input_from_model(&SystemModel, &DeriveFacts)
    -> (MapperInput, DeriveReport)
ros_launch_manifest_derive::resolve_chains(&SystemModel, &DeriveFacts)
    -> Vec<ResolvedChain>
```

This is not 45.2 again. 45.2 embedded `ResolvedChain`s and ranks - the mapper's
output, realizer-specific and stale on replay. An effective trigger is fixed by
the contract alone, like `miss` and `max_jitter_ms` which already cross.

Why the crate lives in rlm and not in `ros-launch-resolve`: nano-ros's
`nros-orchestration-ir` is the crate its `nros::main!` proc-macro depends on,
and `ros-launch-resolve` links tokio, the Python loader and the parser. It
cannot go in rlm's `sched` either (rlm `model` already depends on `sched`), so
it is a fifth workspace member above both.

### What this repository emits (producer side)

`model_builder::path_contract` lowers, additively:

- `trigger`: `PathDecl::effective_trigger()` converted to
  `sched::EffectiveTrigger` (the `convert_trigger` at `sched_derive.rs:229`
  moves here). `input` keeps the Input trigger's endpoints.
- `sync` (policy, `max_interval_ms`, `timeout_ms`), `min_latency_ms`.
- `SubContract.buffer` for `state: true` subscriptions.
- `Contracts.severity_levels`, and `Contracts.node_criticality` = the
  phase-72 EFFECTIVE criticality (`index.derived_criticality` first, the label
  where none reaches) as `sched::Criticality`. `NodeInstance.criticality`
  stays the advisory string it is.

### What this repository stops doing

`sched_derive.rs` keeps `mapper_input_from_dump` only as a shim for one
release: it builds the checked model and calls the shared function. Then the
shim, `extract_rate_hz`, `extract_path_facts`, `extract_criticality`,
`extract_paths`, `claims_concurrency`, `resolve_chains_derived` and
`manifest_graph`'s route copy (`build_global_graph`, `subgraph_for_scope_path`,
`critical_path`, as used by chains) are deleted. `manifest_graph` keeps the
check-side derivations (`derive_topic_rates`, sampling cost, the fault-reaction
route) until the checker moves too, which is not this phase.

`derive_sched_plan` (`sched_loader.rs:704`) is otherwise unchanged: platform
file, `map_with_diagnostics`, overrides, band violations, and `execution.tiers`
plus `bindings` as the applied outcome. `SchedPlan::from_model` is unchanged.

### Rates

`MapperNode.rate_hz` becomes the fastest timer trigger among the node's paths
and nothing else. Topic `rate_hz` and pub `min_rate_hz` stay in the contract
and the model as PROMISES: `measure` and nano-ros's runtime monitors read them;
`rate-mismatch` and `min-rate-mismatch` keep warning when they disagree with
the timers; `derivable-rate` and `derivable-min-rate` stay infos. The one
visible change: `rate_monotonic` no longer ranks a node whose only rate fact is
a promise. A source that publishes at a hardware rate is `trigger:
spontaneous`, and a spontaneous path has no period to be monotonic about; the
default tier is where `chain_aware` already puts it.

## Waves

- **W1 - pin and lower.** Bump `ros-launch-manifest` to the tag that carries
  the model fields and the `derive` crate (v0.1.37 in the plan). Lower the six
  fields above. Golden model fixtures re-emitted; a model written before the
  fields parses with them absent.
- **W2 - the transition gate.** A test over every `tests/fixtures/contract_*`
  workspace asserts `mapper_input_from_dump(dump, index, None, budgets) ==
  mapper_input_from_model(&build_checked_model(..), &facts)` and
  `resolve_chains_derived(index, graph, budgets) == resolve_chains(&model,
  &facts)`. Every difference is a fact the model failed to carry or a rule the
  port got wrong; the gate is green before W3 deletes anything.
- **W3 - delete the copy.** `derive_sched_plan` builds the model and calls the
  shared function; the functions listed above go. `check --explain`,
  `resolve`, `launch`, `run` are behaviour-identical: the W2 gate says so.
- **W4 - release.** 0.12.0. nano-ros pins this repository by gitlink (at
  `07f0461e`, v0.9.0-158, today) and bumps to the 0.12.0 tag in its phase-457
  W1.

## Gates

- `cargo test -p ros-launch-resolve sched_derive` - the W2 parity test, on
  every contract fixture, including `contract_derived_chain` (the one rlm's
  own `RankedPlan` snapshot is taken from).
- `just check` on `rt_workspace` and `contract_w1d` prints the same
  `--explain` provenance before and after W3 (diff of the two outputs in CI).
- The resolved safety-island model carries `trigger: { kind: timer, value: {
  rate_hz: 10.0 } }` on every timer path, and `derivable-min-rate` still
  prints as info, not as a change in the schedule.
- No `min_rate_hz` read remains in any scheduling path of this repository
  (`git grep min_rate_hz src/ros-launch-resolve/resolve/src/ros/sched_*`
  returns nothing).

## Limits

- **The checker still derives from `ManifestIndex`.** `derive_topic_rates`,
  sampling cost and the fault-reaction route keep their index-coupled
  implementations; only the CHAIN route moves. A second port, over the
  model, is what would let nano-ros run the checker's cross-scope rules on a
  model alone; not this phase.
- **`exec_ms` stays a consumer fact.** A platform-file budget and a WCET
  profile are different measurements with different keys; `DeriveFacts`
  carries both and the crate applies the one-path attribution rule. Nothing
  here makes a Linux budget reach an RTOS image.
- **A model from an older resolver ranks nothing.** `trigger: None` is
  Unclassified by design, never a timer. `DeriveReport.paths_without_trigger`
  is what a consumer prints; the migration order (rlm, then here, then
  nano-ros) is what keeps the window short.
- **`rate_monotonic` users lose promise-only ranking.** Named above; the
  alternative - a promise as a fallback rate - is the reading this phase
  exists to remove.
