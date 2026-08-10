# Contract Verification — Implementation

How manifest contracts are verified, as implemented by the `types/` and
`check/` crates in this workspace. For the manifest format see
[launch-manifest.md](launch-manifest.md); for the formal foundations see
[contract-theory.md](contract-theory.md).

> **Scope note.** This document describes the *per-manifest* checker that
> lives in this repository. Cross-file checks (chain link resolution,
> cross-scope QoS reconciliation, the topology-aware critical-path budget
> check) run in the consumer's merge layer (`ros-launch-resolve`, invoked
> by `play_launch check`) because they need the merged launch tree. See
> [Division of Labor](#division-of-labor-with-the-consumer).

## Pipeline

```
YAML file ──→ parse with spans ──→ Manifest AST ──→ build DataflowGraph ──→ run rules ──→ Diagnostics ──→ emit
              (types/, yaml-rust2)  (types::Manifest)  (check/src/graph.rs)   (check/src/rules/)            (terminal | codespan)
```

1. **Parse** (`types/src/parse.rs`) — a hand-rolled deserializer over
   plain `yaml-rust2` values (Serde is used for *serialization* only).
   Spans come from a second pass: `SpanIndex::build` re-scans the source
   with the event parser's `Marker`s and produces a YAML-path → byte-range
   index (`types/src/span.rs`) that diagnostics resolve against.
   Entry points: `parse_manifest`, `parse_manifest_str`, and the
   `*_with_spans` variants returning
   `ParseResult { manifest, source, spans }`.
2. **Filter & substitute** (`types/src/`) — `evaluate_condition` /
   `filter_manifest` apply `if:`/`unless:` conditions for a given arg
   assignment; `resolve_args` / `substitute_manifest` perform `$(var)`
   substitution.
3. **Graph** (`check/src/graph.rs`) — a `petgraph::DiGraph<GraphNode,
   GraphEdge>` over manifest nodes and include scopes, with edges from
   topic publisher → subscriber. Subscribers tagged `state: true` are
   skipped — that is how declared feedback loops break the causal cycle
   check.
4. **Rules** (`check/src/rules/`) — each rule is an independent module
   implementing:

   ```rust
   pub trait ValidationRule: Send + Sync {
       fn id(&self) -> &str;
       fn check(&self, manifest: &Manifest, graph: &DataflowGraph, ctx: &mut CheckContext);
   }
   ```

   `run_checks` / `run_checks_with_spans` (`check/src/check.rs`) build the
   graph, then run `rules::default_rules()` in registration order.
5. **Diagnostics** — rules emit into `CheckContext` (`emit` / `error` /
   `warning`), producing:

   ```rust
   pub struct Diagnostic {
       pub rule_id: String,
       pub severity: Severity,        // Info | Warning | Error
       pub message: String,
       pub path: String,              // YAML path, e.g. "topics.pointcloud.qos"
       pub span: Option<Range<usize>>, // resolved from SpanIndex when available
   }
   ```

## Why manual parsing (not Serde deserialize)

Serde's data model has no concept of source locations — by the time a
struct is populated, span information is gone. The manual `yaml-rust2`
layer keeps a `SpanIndex` from YAML path to byte range, so any rule can
point a diagnostic at the exact offending line, including in multi-file
output.

## Rule Registry

20 rules, in registration order (`check/src/rules/mod.rs`). Severity is
what the rule emits; several rules emit at more than one severity.

| # | Rule | Severity | What it catches |
|---|------|----------|-----------------|
| 1 | `endpoint-unique` | Error | Duplicate endpoint name across a node's pub/sub/srv/cli |
| 2 | `wiring` | Warning | Path input/output endpoint not wired by any topic |
| 3 | `qos-compat` | Error | Invalid QoS value token (`reliability`, `durability`, `history`, `liveliness`) at topic or endpoint level |
| 4 | `qos-match` | Error / Warning | Structural: `depth: 0` (E), `keep_all` with depth (W), `best_effort` + `transient_local` (W). DDS pub/sub compatibility on `reliability` and `durability` (E) — offered ≥ requested, checked only when both sides specify (no implicit ROS defaults) |
| 5 | `rate-hierarchy` | Error | `pub.min_rate_hz < topic.rate_hz`; `topic.rate_hz < sub.min_rate_hz` |
| 6 | `scope-budget` | Warning | Flat conservative sum: scope `max_latency_ms` < Σ node latencies + declared topic transport. Per-manifest fallback — the topology-aware critical path is play_launch's cross-scope diagnostic |
| 7 | `causal-dag` | Error | Cycle in the causal dataflow graph (`state: true` on feedback endpoints breaks it) |
| 8 | `drop-sanity` | Error | Effective delivery rate < subscriber demand; `max_drop_rate` outside [0,1]; `n > w` in `"N / W"`; `max_consecutive == 0` |
| 9 | `service-wiring` | Warning | Service client with no matching server |
| 10 | `service-type` | Error / Warning | Service without `type` (E); server/client ref not declared on its node (W) |
| 11 | `dangling-entity` | Warning / Error | Topic with 0 pubs or 0 subs (W); service/action with 0 servers (E) |
| 12 | `satisfiability` | Warning / Error | **Z3-backed.** Node unreachable under all valid arg assignments (W); some valid arg assignment produces a dangling entity (E). Skips topics whose subscribers are all state-only |
| 13 | `consistency` | — | Placeholder, currently a no-op (cross-scope agreement runs in play_launch) |
| 14 | `state-consistency` | Warning | Likely-missing `state: true` on a subscriber that is neither state-tagged nor referenced by any path trigger (two noise-gated heuristics) |
| 15 | `explicit-trigger` | Info | Path has no explicit `trigger:` — migration lint toward the Vocabulary v2 taxonomy |
| 16 | `inherited-rate` | Warning | Non-`input` explicit trigger combined with a stale legacy `input:` list |
| 17 | `once-durability` | Warning | `once`-triggered path publishes to a topic whose effective durability is not `transient_local` |
| 18 | `sync-feasibility` | Warning | `sync.max_interval_ms` / `sync.timeout_ms` shorter than the slowest declared input period |
| 19 | `queue-drain-rate` | Warning | Timer path `rate_hz` lower than the summed input rates of its `buffer: queue` subscriptions |
| 20 | `chain-shape` | Error | Cyclic chain (same `{scope, path}` twice); adjacent path segments with no `via:` between them |

Shared helper: `rules/endpoint_topic.rs` resolves `node/endpoint`
references to their declaring topic (used by `once-durability`,
`sync-feasibility`, `queue-drain-rate`).

### Z3 and satisfiability

Args declared with `type: bool` or `choices:` define a finite
configuration space. The `satisfiability` rule encodes `if:`/`unless:`
conditions as SMT formulas (crate `z3`) and asks, per entity: *is there a
valid arg assignment under which this topic/service ends up with zero
publishers/servers?* Errors carry the witness assignment:
`"topic 'pose' has 0 publishers when pose_source=gnss"`.

Z3 is used only inside this rule — there is no SMT-LIB file output.

## Emitters

`check/src/emit/` has exactly two backends:

- **`terminal`** — plain stderr lines:
  `"{severity}[{rule_id}]: {message} (at {path})"` plus an
  error/warning count summary.
- **`diagnostic`** — `codespan-reporting` rendering with `rule_id` as the
  diagnostic code and the span as a primary label; falls back to an
  `at {path}` note when the manifest was parsed without spans.

Example codespan output:

```
error[qos-match]: incompatible reliability on topic 'pointcloud': pub best_effort < sub reliable
  ┌─ manifests/sensing/sensing.launch.contract.yaml:12:5
  │
12│     reliability: best_effort
  │     ^^^^^^^^^^^^^^^^^^^^^^^^ topics.pointcloud.qos.reliability
```

## Division of Labor with the Consumer

The checker in this repo is deliberately **single-manifest**. Checks that
need the merged launch tree run in the consumer's cross-scope layer —
the `ros-launch-resolve` resolve crate, which `play_launch check`
invokes. Cross-scope rule ids, emitted from
`resolve/src/ros/manifest_loader.rs` and `chain_checks.rs`:

| Rule | What it checks |
|------|----------------|
| `consistency` | Topic/QoS/rate declarations agree across the scopes that declare the same topic (the local `consistency` rule is a placeholder for exactly this reason) |
| `budget-overflow` | Cross-scope path budgets: a child scope's path budget must not exceed a matched ancestor path's budget (theory doc "Check 1") |
| `scope-budget` | Topology-aware critical path over the merged dataflow DAG, including per-sink `max_transport_ms` overrides (the local flat-sum rule is the standalone fallback) |
| `rate-hierarchy`, `qos-match`, `dangling-entity` | Cross-scope variants of the local rules, run after merge |
| `chain-link` | Every chain `{scope, path}` segment resolves; `via:` topics exist, are produced by the preceding segment and consumed by the following one |
| `chain-budget` | Chain `max_latency_ms` vs the sum of its event-segment budgets plus boundary sampling costs (each timer boundary contributes period + exec, not its declared budget) |
| `chain-sampling-feasibility` | Chain budget minus boundary sampling cost must leave positive controllable time (mirrors the `chain_aware` mapper's feasibility rule) |

Runtime monitors (rate, age, drop, burstiness) live in play_launch's
interception layer (Phase 29), fed by `rcl_publish`/`rcl_take` events.

Invocation:

```bash
# Check the manifests of a launch tree (merged, cross-file checks included)
play_launch check <pkg> <launch_file>

# Filter to one rule, JSON output
play_launch check --rule qos-match --format json <pkg> <launch_file>

# Scheduling plan check with per-node provenance
play_launch check --sched <platform.yaml> --explain <pkg> <launch_file>
```

## Crate Choices (as shipped)

| Concern | Crate | Notes |
|---------|-------|-------|
| YAML parsing with spans | `yaml-rust2` | `MarkedYaml` gives line/col per node; converted to byte offsets |
| Diagnostic rendering | `codespan-reporting` | Multi-file, FileId-based |
| Graph analysis | `petgraph` | Dataflow DAG, cycle detection |
| Satisfiability | `z3` | Finite arg-space checking in the `satisfiability` rule |

Ideas from earlier drafts of this document that were **not** built: a
`Constraint`/`ConstraintKind` intermediate representation, a formal-notation
emitter, SMT-LIB output, `good_lp` budget optimization, and RTLola
monitors. The rule → `Diagnostic` path proved sufficient; revisit only
with a concrete need.
