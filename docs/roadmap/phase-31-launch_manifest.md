# Phase 31: Launch Manifest

**Status**: In progress (31.1–31.5 complete: types, checker, fixtures, executor loading, integration tests, source spans, static check CLI — 110 tests)
**Priority**: High
**Dependencies**: Phase 30 (scope table), Phase 29 (RCL interception)
**Repo**: `src/ros-launch-manifest/` (workspace with multiple crates)

Design docs (in `src/ros-launch-manifest/docs/`):
- `launch-manifest.md` — manifest format specification
- `contract-theory.md` — formal contract theory
- `contract-verification.md` — implementation tooling
- `design-issues.md` — open design issues (conditions, service contracts)

Research docs (in `docs/research/`):
- `caret-analysis.md` — CARET causal model analysis
- `io-contract-prior-art.md` — prior art survey
- `manifest-prior-art.md` — tool survey

---

## 31.1: Manifest types crate

**Crate**: `ros-launch-manifest-types` in `src/ros-launch-manifest/`

Parse manifest YAML into typed AST with source spans.

- [ ] Cargo.toml: `yaml-rust2`, `thiserror`, `serde` (for output)
- [x] `types.rs`: core types (14 types)
  - `Manifest`, `NodeDecl`, `EndpointProps`, `SrvEndpointProps`
  - `TopicDecl`, `ServiceDecl`, `ActionDecl`
  - `IncludeDecl` (External/Inline), `QosDecl`
  - `PathDecl`, `DropSpec`, `DropCount`, `GlobalTopicDecl`
- [x] `span.rs`: `Spanned<T>` wrapper, `line_starts()`, `line_col_to_offset()`
- [x] `parse.rs`: yaml-rust2 → `Manifest`
  - Plain list vs map form for endpoints
  - `drop: N / W` string parsing + full form
  - Shorthand topic form (type only)
  - External/inline include parsing
- [x] Unit tests: 18 tests (round-trip, shorthand, edge cases, drops, includes)
- [x] Validate: endpoint name uniqueness per node (at parse time)

## 31.2: Static checker crate

**Crate**: `ros-launch-manifest-check` in `src/ros-launch-manifest/`

Graph algorithms + constraint checking with diagnostic output.

- [x] `graph.rs`: build dataflow graph from manifest (petgraph DiGraph)
  - Nodes + child scopes as vertices
  - Topic wiring as edges (endpoint reference resolution)
- [x] `rules/` directory with `ValidationRule` trait + 9 rules:
  - `endpoint_unique.rs`: names unique per node
  - `wiring.rs`: path endpoints wired by topics, imports resolved
  - `qos_compat.rs`: QoS values valid (reliability, durability)
  - `rate_hierarchy.rs`: pub.min_rate_hz >= topic.rate_hz >= max(sub.min_rate_hz)
  - `rate_chain.rs`: export rates achievable from upstream
  - `scope_budget.rs`: scope latency >= node sum, age >= latency
  - `causal_dag.rs`: dataflow graph acyclic (petgraph)
  - `drop_rate.rs`: delivery rate composition (product/log-sum), scope check
  - `drop_consecutive.rs`: Erdos-Renyi Poisson check at epsilon=0.01 + necessary condition
- [x] `check.rs`: CheckContext, Diagnostic, Severity, run_checks()
- [x] `emit/terminal.rs`: simple stderr diagnostic output
- [x] `emit/diagnostic.rs`: codespan-reporting with source spans (done in 31.4c)
- [ ] `emit/formal.rs`: Unicode mathematical notation (deferred)
- [x] Integration tests: 18 tests (clean, violations, pipeline)
- [x] Full fixture set (31.3)

## 31.3: Test fixtures

- [x] `tests/fixtures/manifest_simple/` — talker/listener, clean (2 tests)
- [x] `tests/fixtures/manifest_pipeline/` — perception pipeline: lidar + camera + fusion + tracker (3 tests)
- [x] `tests/fixtures/manifest_ndt/` — NDT with state, required, feedback cycle, services (4 tests)
- [x] `tests/fixtures/manifest_periodic/` — timer-driven nodes with empty-input paths (3 tests)
- [x] `tests/fixtures/manifest_violations/` — intentional violations for all 9 rules (8 tests)
- [x] `tests/fixtures/manifest_multi_scope/` — nested inline includes with imports/exports wiring (3 tests)
- [x] Cross-fixture round-trip test (1 test)
- [x] Bug fix: graph builder now skips `state: true` subscriber edges (feedback loops ≠ causal cycles)

## 31.4: Manifest loading in executor

**In**: `src/play_launch/`

Load and resolve manifests at executor startup using the scope table from record.json.
The parser stays decoupled — it produces the scope table, the executor applies manifests.

**Rationale**: The parser (`play_launch_parser`) is a standalone workspace with its own
`[workspace]` in Cargo.toml. Adding a path dependency to `ros-launch-manifest-types` would
require `..` relative paths across workspace boundaries. Since the executor already has the
full scope table (pkg, file, ns, args, parent) from record.json, it can resolve manifests
without the parser knowing about them.

- [x] Add `--manifest-dir <path>` to executor CLI options (`CommonOptions`)
- [x] `ManifestLoader` in `src/play_launch/src/ros/manifest_loader.rs`:
  - Lookup `<manifest_dir>/{pkg}/{stem}.yaml` for each file scope (strips `.launch.xml`/`.launch.py`)
  - Cache loaded manifests by scope ID (`ManifestIndex`)
  - Apply scope `ns` prefix to manifest's relative topic/node names (`qualify_name`, `qualify_endpoint_ref`)
  - Scopes without manifests silently skipped (incremental adoption)
- [x] Run static checks (31.2) on each loaded manifest at startup
- [x] Build resolved manifest index (`ManifestIndex`):
  - `topics`: resolved topic FQN → {type, qos, pub, sub, rate_hz, drop}
  - `node_paths`: node FQN + path name + PathDecl
  - `scope_paths`: scope ID → named paths
- [x] Log check diagnostics (errors as `warn!`, warnings as `debug!`)
- [x] 14 tests (5 unit + 9 integration using fixture YAML files)
- [x] Fix: manifest crate `edition.workspace = true` → `edition = "2024"` for cross-workspace compatibility
- [ ] Integration test: load manifests for Autoware scopes, verify resolution (deferred — requires real Autoware manifests)

## 31.4a: Manifest integration tests

Integration tests exercising the full manifest loading pipeline (scope table → manifest
resolution → static checks → resolved index). Tests use synthetic scope tables paired with
the 31.3 fixture manifests. Covers general cases and Autoware-observed edge cases.

**General cases (6 tests):**
- [x] Single scope, single manifest — load, resolve names, verify topic FQNs
- [x] Multiple scopes — each scope loads its own manifest, topics don't collide
- [x] Scope with deep namespace — `/planning/scenario_planning/lane_driving` prefixing
- [x] Manifest with violations — errors counted but loading proceeds (warn, not fail)
- [x] Manifest not found — scope silently skipped, no error
- [x] Mixed scopes — some with manifests, some without, some group scopes

**Autoware-observed edge cases (6 tests, from 169-scope, 48-package launch tree):**
- [x] Same (pkg, file) included multiple times with different namespaces — like
      `load_topic_state_monitor.launch.xml` ×10 under `/system`. Each gets its own
      resolved topic set with distinct namespace prefixes. Manifests are loaded once per
      scope ID, not deduplicated.
- [x] Group scopes (origin: None) interleaved with file scopes — groups are skipped,
      their child file scopes still resolve correctly
- [x] Deep nesting (depth 16) — scope at depth N resolves manifest from its own
      `(pkg, file)`, not from ancestors. Namespace comes from scope's `ns` field, not
      accumulated from parent chain.
- [x] Scopes without entities (108/169 in Autoware) — manifest loaded and checked even
      if no nodes reference this scope (useful for pre-validation)
- [x] Large argument sets — scopes with 100+ args don't affect manifest resolution
      (args are scope metadata, not used by manifest loader)
- [x] Root scope at namespace "/" — topic `chatter` becomes `/chatter`, not `//chatter`

**Additional coverage (9 tests):**
- [x] All 6 fixtures × 4 namespace variants — smoke test, no panics, no double-slashes
- [x] Pipeline full resolution — all 4 nodes, 5 topics, scope paths verified
- [x] NDT state/required — 2 EKF paths, max_latency_ms values, topic FQNs
- [x] Periodic empty-input paths — all 3 nodes verified as timer-driven

## 31.4b: Autoware manifest files

**Repo**: `~/repos/autoware-contract/` (separate repository, self-contained docs)
**Status**: Complete — 36 manifests, all refined with source traceability.
See `~/repos/autoware-contract/docs/roadmap.md` for per-file status.

## 31.4c: Source span tracing and diagnostic reporting

**In**: `src/ros-launch-manifest/`

Wire source byte spans from YAML parsing through to diagnostic output so errors
point at the exact line/column in the manifest file. Currently the parser discards
yaml-rust2's `Marker` positions and diagnostics carry string paths (e.g.,
`"nodes.ndt.paths.main"`) instead of byte ranges.

**Parser: SpanIndex via MarkedEventReceiver (`types/src/span.rs`)**

Instead of migrating all AST nodes to `Spanned<T>`, a `SpanIndex` replays the
yaml-rust2 event parser (`MarkedEventReceiver`) to build a `HashMap<dot_path, Range<usize>>`
mapping YAML key paths to byte offsets. This keeps the existing parser untouched
while enabling source-level diagnostics.

- [x] `SpanIndex::build(source)` — event-driven span collection via `SpanReceiver`
- [x] `SpanReceiver` implements `MarkedEventReceiver` — tracks mapping keys, nesting,
      sequences; records byte offset for each key using `Marker::index()`
- [x] `parse_manifest_with_spans()` / `parse_manifest_str_with_spans()` — new entry
      points returning `ParseResult { manifest, source, spans }`
- [x] 7 unit tests: simple keys, nested topics, byte offset correctness, QoS nesting,
      paths, empty input

**Diagnostic: byte spans (`check/src/check.rs`)**
- [x] `Diagnostic.span: Option<Range<usize>>` — byte range in source text
- [x] `CheckContext.spans: Option<SpanIndex>` — auto-resolves path→span on `emit()`
- [x] `run_checks_with_spans(manifest, spans)` — new entry point
- [x] All 9 rules automatically get spans via `CheckContext::emit()` path lookup
      (no per-rule changes needed — path strings already match SpanIndex keys)
- [x] 4 integration tests: QoS span, rate-hierarchy span, scope-budget span,
      no-spans-without-index

**Emitter: codespan-reporting (`check/src/emit/diagnostic.rs`)**
- [x] Added `codespan-reporting = "0.11"` dependency
- [x] `emit_diagnostics(result, filename, source)` — renders to stderr with
      `ColorChoice::Auto` (isatty detection)
- [x] `emit_diagnostics_to(writer, result, filename, source)` — generic writer
- [x] `emit_diagnostics_to_string()` — for testing
- [x] Maps `Diagnostic` → codespan `Diagnostic<()>` with primary label at span
- [x] Falls back to notes with YAML path when span is absent
- [x] 4 unit tests: QoS violation with line numbers, rate hierarchy with span,
      no-span fallback, clean manifest empty output

**Integration**
- [x] `manifest_loader.rs` uses `parse_manifest_with_spans()` and
      `run_checks_with_spans()` — diagnostics carry byte spans
- [x] `ResolvedManifest.source: String` stored for downstream codespan output
- [ ] Wire codespan emitter into `manifest_loader.rs` log output (currently logs
      via `tracing::warn!` — codespan rendering deferred to CLI tool in 31.8)

## 31.5: Static check CLI

**In**: `src/play_launch/`

`play_launch manifest check` — post-hoc static checking without running the system.
Uses scope table from record.json + manifest YAML files. Renders diagnostics with
codespan-reporting source excerpts.

- [x] Add `Manifest` subcommand to CLI: `play_launch manifest <subcommand>`
  - `ManifestArgs` with `ManifestSubcommand::Check(ManifestCheckArgs)`
  - Dispatch in `main.rs`, handler in `commands/manifest.rs`
- [x] `play_launch manifest check record.json --manifest-dir dir/`
  - Load record.json → scope table
  - Load manifests per scope (reuse `manifest_loader::load_manifests()`)
  - Run static checks with spans
  - Render diagnostics with codespan-reporting (`emit::diagnostic::emit_diagnostics()`)
  - Per-scope summary (N manifests checked, N clean, N with errors)
  - Exit code 1 if any errors, 0 if clean
- [x] `play_launch manifest check --manifest path/to/single.yaml`
  - Check a single manifest file without record.json (no namespace resolution)
  - Useful for authoring and CI validation
- [x] `--format` flag: `terminal` (default, codespan with source excerpts) or `json`
  (machine-readable with byte spans)
- [x] Integration tests (`tests/tests/manifest_check.rs`, 11 tests):
  - Clean manifests exit 0 (simple, pipeline, NDT, periodic)
  - Violations manifest exits nonzero
  - Stderr contains rule IDs (qos-compat, rate-hierarchy, causal-dag)
  - Stderr contains filename with line numbers (codespan output)
  - JSON format: valid array with rule/severity/message/path/span fields
  - Clean JSON: empty array
  - Missing file: nonzero exit
  - No args: nonzero exit

## 31.6: Runtime monitors

**In**: `src/play_launch/`

Check contracts against RCL interception data at runtime.

- [ ] `LatencyMonitor`: per-path, check t_pub - t_take <= max_latency_ms
- [ ] `LatencyAnomalyMonitor`: per-path, flag t_pub - t_take < min_latency_ms
- [ ] `RateMonitor`: per-endpoint, check actual rate in [min_rate_hz, max_rate_hz]
- [ ] `JitterMonitor`: per-endpoint, check |interval - period| <= jitter_ms
- [ ] `DropMonitor`: per-path and per-topic, sliding window N/W count
- [ ] `AgeMonitor`: per-path, static bound check + header.stamp where available
- [ ] `BurstinessMonitor`: always-on per-topic
  - Lag-1 autocorrelation
  - Dispersion index (window=100)
  - Observed max run vs Bernoulli expected
  - Estimated mean burst length (when DI > 1.5)
- [ ] 4-way diagnosis: (assumption ok/violated) × (guarantee ok/violated)
- [ ] Wire monitors to interception consumer task (Phase 29 SPSC events)
- [ ] Write `manifest_audit.json` to play_log on shutdown
- [ ] Integration test: launch with manifest, verify audit output

## 31.7: Executor audit (graph diff)

**In**: `src/play_launch/`

Periodically diff expected graph vs runtime GraphSnapshot.

- [ ] `audit.rs`: `diff(expected, actual) → AuditReport`
- [ ] Deviation types: new topic, missing topic, new endpoint, missing endpoint, QoS mismatch, type mismatch
- [ ] Stabilization period: escalate "missing topic" from info → warn after N seconds
- [ ] Integrate into coordinator: periodic audit (every 5s after stabilization)
- [ ] Log deviations: warn!/info!
- [ ] API endpoint: `GET /api/manifest/diff`
- [ ] Integration test: launch with manifest, inject deviation, verify detection

## 31.8: Capture

**In**: `src/play_launch/`

Snapshot runtime graph into per-launch-file manifests.

- [ ] Add `--save-manifest-dir <path>` CLI flag
- [ ] Use scope table to partition runtime graph by launch file
- [ ] For each scope: collect nodes, their pub/sub endpoints, topics
- [ ] Strip namespace prefix (scope provides the prefix)
- [ ] Infer imports/exports from unmatched endpoints
- [ ] Derive rate_hz from observed rates, drop from observed drops
- [ ] Write per-launch-file YAML to manifest dir
- [ ] Integration test: capture → audit round-trip (zero deviations)

## 31.9: Runtime CLI tools

Depends on 31.6–31.8 (runtime features must exist before CLI exposes them).

- [ ] `play_launch manifest capture --manifest-dir dir/`
  - Save current runtime graph as manifests (requires running system)
- [ ] `play_launch manifest diff record.json --manifest-dir dir/`
  - Show expected vs actual differences
- [ ] `play_launch context --manifest-dir dir/`
  - Extend Phase 30 context tool with manifest-derived info

---

## Crate Structure

```
src/ros-launch-manifest/
├── Cargo.toml                    # workspace
├── types/
│   ├── Cargo.toml                # ros-launch-manifest-types
│   └── src/
│       ├── lib.rs
│       ├── types.rs              # Manifest, NodeDecl, TopicDecl, PathDecl, ...
│       ├── span.rs               # Spanned<T>, SpanIndex, MarkedEventReceiver
│       └── parse.rs              # yaml-rust2 → Manifest (with_spans variants)
├── check/
│   ├── Cargo.toml                # ros-launch-manifest-check
│   └── src/
│       ├── lib.rs
│       ├── graph.rs              # petgraph dataflow graph
│       ├── check.rs              # CheckContext, run_rules()
│       ├── rules/
│       │   ├── mod.rs            # ValidationRule trait, default_rules()
│       │   ├── wiring.rs
│       │   ├── scope_budget.rs
│       │   ├── age_budget.rs
│       │   ├── qos_compat.rs
│       │   ├── rate_hierarchy.rs
│       │   ├── rate_chain.rs
│       │   ├── causal_dag.rs
│       │   ├── endpoint_unique.rs
│       │   ├── drop_rate.rs
│       │   └── drop_consecutive.rs
│       └── emit/
│           ├── mod.rs
│           ├── diagnostic.rs     # codespan-reporting
│           └── formal.rs         # Unicode math notation
└── tests/
    └── fixtures/
```

Dependencies:

| Crate | Dependencies |
|-------|-------------|
| types | yaml-rust2, thiserror, serde |
| check | types, petgraph, codespan-reporting |
| play_launch_parser | *(none — parser stays decoupled)* |
| play_launch | types, check (manifest loading, runtime monitoring, audit) |

The manifest crates are a standalone workspace (`src/ros-launch-manifest/`).
`play_launch` adds path dependencies to them. The parser never imports manifest types.

---

## Phase Order

```
31.1 (types) ──→ 31.2 (checker) ──→ 31.3 (fixtures)
                                         │
         31.4  (executor loading) ───────┤
         31.4a (integration tests) ──────┤
         31.4b (autoware-contract repo) ─┤  ← separate repo, parallel work
         31.4c (source spans + report) ──┤
                                         │
         31.5  (static check CLI) ───────┤  ← next: usable without runtime
                                         │
         31.6  (runtime monitors) ───────┤
         31.7  (executor audit) ─────────┤
         31.8  (capture) ────────────────┤
         31.9  (runtime CLI tools) ──────┘
```

31.1–31.4c are complete (manifest crates + executor loading + integration tests + source spans).
31.4b is parallel work in a separate repo.
31.5 is next — static check CLI, usable without running the system.
31.6–31.9 integrate runtime monitoring into play_launch.

---

## Verification

```bash
# Manifest crate tests (standalone, no ROS env needed)
cd src/ros-launch-manifest
cargo test -p ros-launch-manifest-types    # 24 tests
cargo test -p ros-launch-manifest-check    # 54 tests (7 unit + 22 checker + 25 fixture)

# Executor manifest loader tests
cd src/play_launch
cargo test manifest_loader                 # 21 tests

# Static check — single manifest (no record.json needed)
play_launch manifest check --manifest path/to/manifest.yaml

# Static check — per-scope via record.json
play_launch manifest check record.json --manifest-dir manifests/

# Executor integration (runtime)
play_launch launch demo_nodes_cpp talker_listener.launch.xml \
    --manifest-dir tests/fixtures/manifest_simple/

# Capture round-trip (runtime)
play_launch launch autoware_launch planning_simulator.launch.xml \
    --save-manifest-dir /tmp/manifests
play_launch manifest check record.json --manifest-dir /tmp/manifests
```
