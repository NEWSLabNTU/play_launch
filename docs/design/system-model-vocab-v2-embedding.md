# SystemModel: embedding Vocabulary v2 (Phase 44.7 coordination note)

Status: coordination note, not a design change. Written by the Phase 44
(vocabulary v2 + chain-aware mapper) track for the Phase 43/layer-2 track,
which owns the `model` crate. **No code in the `model` crate is touched by
this note or by Phase 44** — see [system-model.md](system-model.md) for the
artifact this describes.

## What layer 2 gains

`SystemModel`'s `contracts:` layer (layer 2) today embeds the *pre-vocab-v2*
manifest facts: `endpoints` (rates/ages/state/required), `node_paths`
(latency/correlation/tolerance), `scope_paths` (E2E budgets), `topics`. None
of that needs to change — vocabulary v2 is additive on the manifest side
(`docs/superpowers/specs/2026-07-17-contract-vocabulary-v2-design.md` §6).
What layer 2 is currently missing, and would gain by embedding the v2
structs verbatim (per that same §6: "the model crate's contracts layer
should gain the same structs rather than a parallel copy"):

1. **Resolved chains with FQN `via:` topics.** Today a chain (`chains:`,
   Phase 44.1 §4) lives only in the manifest's own scope-relative form
   (`{ scope: "/perception", path: "preprocess" }`, `{ via:
   "/perception/objects" }`). A model consumer (nano-ros build system, or
   play_launch's own `--model` runtime path) has no resolved, FQN-qualified
   view of "which concrete nodes, in which order, compose this budget" —
   it would have to re-run manifest resolution itself, which is exactly
   the duplication `SystemModel` exists to prevent (`system-model.md` §Why,
   point 1).
2. **Per-path triggers.** `trigger: timer|input|once|spontaneous`
   (Phase 44.1 §1) is the fact that makes a node's own contribution to a
   chain classifiable as a clock boundary vs. an event-segment link. Without
   it embedded, a model consumer can see *that* a node is on a chain but not
   *what kind* of hop it is — the distinction the chain-aware mapper's
   entire feasibility/ranking algorithm is built on
   (`docs/superpowers/specs/2026-07-17-chain-aware-mapper-design.md` §Model).
3. **Per-path ranks from `MapDiagnostics`.** The chain-aware mapper (Phase
   44.3/44.4) computes a rank *per path*, not just per node — the POSIX
   apply layer projects to a per-node max (a lossy, documented compression,
   `chain-aware-mapper-design.md` §Algorithm step 6), but non-POSIX
   consumers (nano-ros executors on an RTOS, which *can* discriminate at
   callback granularity) shouldn't inherit that lossy projection. Embedding
   the per-path ranks (`ChainAwareDetail`, one per (node, path)) in layer 2
   or layer 3 gives an embedded consumer the finer-grained fact play_launch
   itself already computes and then throws away at the POSIX boundary.

## Exact types to share (no parallel copies)

Two crates, two representations — both already exist, deliberately
decoupled (`sched/src/chain.rs`'s own module doc: "the sched crate has no
dependency on `ros-launch-manifest-types` today... kept
FQN-string-based/dependency-light"). The `model` crate should reuse the
**declaration-side** (`types`) structs for what a chain *says*, and either
reuse or mirror the **resolved-side** (`sched`) structs for what a chain
*resolves to* — matching the same translation boundary play_launch's own
`sched_derive.rs` already crosses.

**From `ros-launch-manifest-types` (`types/src/types.rs`) — declaration
shapes, already `Serialize`, safe to embed as-is:**

| Type | Signature | Role |
|---|---|---|
| `Trigger` | `enum { Timer { rate_hz: f64 }, Input(Vec<String>), Once, Spontaneous }` | the authored `trigger:` field |
| `EffectiveTrigger` | `enum { Timer { rate_hz: f64 }, Input(Vec<String>), Once, Spontaneous, Unclassified }` | `PathDecl::effective_trigger()`'s output — the single source of truth after legacy-`input:` derivation |
| `PathDecl::effective_trigger(&self) -> EffectiveTrigger` | method | how to derive it; the model's resolve step should call this once and embed the *result*, not re-derive it downstream |
| `Sync` | `struct { policy: SyncPolicy, max_interval_ms: Option<f64>, timeout_ms: Option<f64> }` | fan-in policy |
| `SyncPolicy` | `enum { Exact, Approximate, TimeoutAny }` | — |
| `ChainDecl` | `struct { semantics: ChainSemantics, max_latency_ms: f64, segments: Vec<ChainSegment> }` | the authored `chains:` entry |
| `ChainSemantics` | `enum { Reaction, Age }` | reaction vs. age composition math |
| `ChainSegment` | `enum { Path { scope: String, path: String }, Via { via: String } }` | one alternating hop |
| `Buffer` | `enum { Latest, Queue }` | `state:` sub buffering discriminator |

**From `ros-launch-manifest-sched` (`sched/src/chain.rs`, `sched/src/mapper.rs`)
— resolved shapes, FQN-qualified, what a model consumer actually wants:**

| Type | Signature | Role |
|---|---|---|
| `ResolvedChain` | `struct { name: String, criticality: Criticality, max_latency_ms: f64, semantics: ChainSemantics, elements: Vec<ChainElement> }` | one chain, fully resolved against the launch DAG |
| `ChainElement` | `enum { Segment { nodes_in_topo_order: Vec<(String, String)> }, Boundary { node: String, path: String, period_ms: f64, exec_ms: Option<f64> } }` | the S·B·S decomposition — this is item 1 above, already computed, ready to embed |
| `MapperPath` | `struct { name: String, effective_trigger: EffectiveTrigger, max_latency_ms: Option<f64>, inputs: Vec<String>, outputs: Vec<String> }` | per-path facts (item 2 above) — mirrors `types::EffectiveTrigger` per the sched crate's own doc comment |
| `ChainAwareDetail` | `struct { node: String, path: Option<String>, priority: i64, provenance: String }` | per-(node, path) rank provenance (item 3 above) — one instance per ranked path, from `MapDiagnostics.details` |
| `MapperInput.chains: Vec<ResolvedChain>` | field | the resolved-chain list the mapper consumes; `resolve()` already builds the equivalent (`sched_derive::resolve_chains` in play_launch) and this is what the model's `contracts:` layer would carry forward |

**Translation boundary** (do this once, in the resolver, not per-consumer):
`sched_derive::resolve_chains` (`src/play_launch/src/ros/sched_derive.rs`,
Phase 44.4) is the existing, tested reference implementation of
`types::ChainDecl` + launch DAG → `sched::ResolvedChain`. The `model` crate
should not reimplement this translation — either the resolver
(`play_launch resolve`, `commands/resolve.rs`) calls the same function and
embeds its output, or (if the `model` crate wants to be resolver-independent)
the translation function itself moves to a location both `play_launch` and
`model` can depend on. Either way: **one** translation, embedded verbatim,
matching this design's own stated policy ("no parallel copies").

## The residual gap this closes

Phase 43's `SystemModel`/`--model` runtime path has a **documented, known
gap** today (W4 report, `.superpowers/sdd/p44-w4-report.md` deliverable 4):
`SchedPlan::from_model` — the code path a `replay --model system_model.yaml`
invocation uses — hands back `chain_member_nodes: BTreeSet::new()`
unconditionally, because the model's `execution:` layer (layer 3) is
bindings-only (FQN → tier name) and carries no chain structure at all. This
means:

- The container co-location warning (`chain_container_colocation_warnings`,
  §1.7/design step 8 in the chain-aware-mapper spec) never fires on the
  `--model` path — a chain member co-located in a stock/observable container
  silently loses the "these nodes can't get distinct priorities" warning
  that the *launch-file* path already gives.
- More generally, a bare `replay --model <file>` run has **zero** chain
  awareness: it can apply the tier numbers the model was resolved with, but
  it cannot re-derive, re-explain, or re-warn about chain structure, because
  the structure was never carried in the artifact it's reading.

Embedding `ResolvedChain`/`ChainElement`/per-path `MapperPath` facts in
layer 2 (or the resolved plan in layer 3, alongside `tiers:`/`bindings:`)
closes this gap directly: a `--model` consumer would have the same chain
facts the launch-file path derives fresh every time, without re-parsing
contracts or re-running `resolve_chains`. This is the same "one canonical
input" argument `system-model.md` already makes for every other layer-2
fact (`system-model.md` §Why, point 1) — chains are not a special case, they
were simply not yet vocabulary when that document was written.

## Non-goals (this note)

- No change to the `model` crate — that crate is owned by the Phase 43
  track; this note is an input to its next iteration, not a patch.
- No opinion on *where* in the model the resolved chain data lands (layer 2
  `contracts:` vs. a new field on layer 3 `execution:` alongside
  `tiers`/`bindings`) — that's a layout decision for whoever implements the
  embedding, informed by how `bindings:` already keys on FQN.
- Does not resolve the separate, already-tracked FQN-qualification carry-
  forward (contract-side `qualify_name` vs. launch-dump node identity,
  `.superpowers/sdd/p44-w6-fixes-report.md` Fix B) — that fix already
  landed on the `play_launch`-native path; a model-embedding implementation
  should simply reuse the same reconciled identity, not reintroduce the bug.

## Cross-link

See [system-model.md](system-model.md) — this note has a one-line pointer
added there under "Work items", since that file is shared with the
Phase 43/nano-ros track.
