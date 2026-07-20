# play_launch Roadmap

Started October 2025. 34 phases total; 27 complete, 3 in progress, 4 planned.

Completed phase docs are in `archive/`.

---

## All Phases

| # | Phase | Status | Completed |
|---|-------|--------|-----------|
| 1–7 | Core CLI, dump_launch, docs, testing, I/O, logging | ✅ | 2025 |
| 8 | Web UI | ✅ | 2025-12-18 |
| 9 | Web UI Status Refactoring | ✅ | 2026-01-17 |
| 10 | Async Actor Pattern | ✅ | 2026-01-01 |
| 11 | Web UI Actor Integration | ❌ Dropped | — |
| 12 | Container-Managed Composable Nodes | ✅ | 2026-01-13 |
| 13 | Rust Parser Migration | ✅ | 2026-01-27 |
| 14 | Python Launch File Execution | ✅ | 2026-01-31 |
| 14.5 | Namespace Accumulation Fixes | ✅ | 2026-02-02 |
| 15 | Python API Type Safety | ✅ | 2026-01-31 |
| 16 | YAML Parameter Loading & Global Params | ✅ | 2026-02-06 |
| 17 | Context Unification & Parser Parity | 🔄 17.1–17.6 done | — |
| 18 | Code Quality | ✅ | 2026-02-08 |
| 19 | Isolated Component Manager | ✅ | 2026-02-17 |
| 20 | Web UI Modernization (Preact + SSE) | ✅ | 2026-02-20 |
| 21 | Build System Optimization | ✅ | 2026-02-18 |
| 22 | Launch Tree IR & WASM | ✅ 22.1–22.8 | 2026-02-24 |
| 23 | Code Quality (round 2) | ✅ | 2026-02-25 |
| 24 | Web UI Parameter Control | ✅ | 2026-02-27 |
| 25 | Runtime Graph & Topic Introspection | 🔄 25.1–25.9 done | — |
| 26 | Web UI Metrics Dashboard | ✅ | 2026-03-01 |
| 27 | Runtime Dependency Check | ✅ | 2026-03-03 |
| 28 | Parser Integration Test Coverage | ✅ | 2026-03-03 |
| 29 | RCL Interception & Frontier Tracking | ✅ | 2026-03-11 |
| 30 | Launch Tree Scoping | ✅ | 2026-03-19 |
| 30b | Group Scopes | ✅ | 2026-03-20 |
| 31 | Launch Manifest | 📋 Planned | — |
| 36 | Runtime Enforcement | ✅ 36.1–36.7 | 2026-05-11 |
| 38 | Linux RT Scheduling Apply-Layer | ✅ 38.1–38.10 | 2026-07-15 |
| 39 | RT Example Workspace | ✅ 39.1–39.4 | 2026-07-15 |
| 40 | Contract Shipping (sidecar + overlay) | ✅ 40.1–40.7 | 2026-07-15 |
| 41 | RT Config v2 (derived scheduling) | 🔄 41.1–41.5 done, 41.6 gated | — |
| 42 | Autoware System Model Study | ✅ 42.0–42.6 | 2026-07-17 |
| 43 | Runtime Consumes the SystemModel | 🔄 43.1–43.3, 43.5 done; 43.4 re-scoped | — |
| 44 | Vocabulary v2 + Chain-Aware Mapper | 🔄 44.1–44.6, 44.8 done; 44.7 handoff | — |
| 45 | Scheduling SSoT Unification | ✅ 45.1–45.8 | 2026-07-19 |
| 46 | Unified SystemModel (one artifact) | ✅ 46.1–46.6 | 2026-07-20 |

---

## Active & Planned

### Phase 17: Context Unification (in progress)

17.1–17.6 complete (unified LaunchContext, global statics removed, 6 parity fixes). 17.7 remaining: 6 categories of parser parity gaps (namespace, exec_name, params_files, container ordering, global params in XML cmd, exec path fallback).

See [phase-17-context_unification.md](./phase-17-context_unification.md).

### Phase 22: Launch Tree IR (remaining items)

22.1–22.8 complete (IR types, builder, evaluator, WASM codegen/runtime, 18 round-trip tests). Remaining: 22.9 Python AST compiler, 22.10 Python WASM integration, 22.11 Autoware smoke tests.

See [phase-22-launch_tree_ir.md](./phase-22-launch_tree_ir.md).

### Phase 25: Runtime Graph & Topic Introspection (in progress)

25.1–25.9 complete (graph builder, topic/QoS endpoints, TopicsTab, node badges, Cytoscape.js graph view with ELK layout). 25.10 in progress.

See [phase-25-topic_introspection.md](./phase-25-topic_introspection.md).

### Phase 30: Launch Tree Scoping (complete)

Scope table in both Rust and Python parsers. 83 scopes / 119 entities for Autoware, both parsers match exactly. Context extraction tool (`scripts/launch_context.py`).

See [phase-30-launch_scoping.md](./phase-30-launch_scoping.md).
Design: [docs/design/record-format.md](../design/record-format.md), [docs/design/launch-context-tool.md](../design/launch-context-tool.md).

### Phase 30b: Group Scopes (complete)

ScopeEntry refactored: `origin: Option<ScopeOrigin>` (null for groups, object for files). Groups with namespace attribute create anonymous scope entries. Autoware: 83 file scopes, 0 group scopes (uses `<push-ros-namespace>` pattern). Cross-parser comparison passes.

See [phase-30b-group_scopes.md](./phase-30b-group_scopes.md).

### Phase 31: Launch Manifest (planned)

Per-launch-file manifest system. Describes expected communication graph: topics, services, actions, QoS, timing contracts. Manifest crate, parser integration, executor audit. Uses Phase 30 scope table as the bridge between manifests and record.json.

See [phase-31-launch_manifest.md](./phase-31-launch_manifest.md).
Design: [docs/design/launch-manifest.md](../design/launch-manifest.md).

### Phase 36: Runtime Enforcement (in progress)

Closes the static → runtime contract loop. Extends Phase 29 LD_PRELOAD interception with an RMW-layer hook set (36.1 in progress) and adds a `RuleEngine` consumer that evaluates manifest contracts against live traffic (36.3). Also: QoS-negotiation visibility (36.2), `--enforce-rules` CLI flag (36.4), `GraphPlugin v2` warn-only (36.5), lifecycle-aware gating (36.6), and optional v3 blocking enforcement (36.7).

See [phase-36-runtime_enforcement.md](./phase-36-runtime_enforcement.md).

### Phase 37: Crate Split + Boundary Cleanup (planned)

Refactor monolithic `play_launch` (~19k LOC) into 6 focused crates with sharp interfaces between executor, checker, enforcement, and parser. Driven by [`docs/design/architecture-review.md`](../design/architecture-review.md). Seven sub-phases: 37.1 `play_launch_interception_types`, 37.2 `play_launch_record_format`, 37.3 trait-ify ManifestIndex, 37.4 per-subcommand CLI args, 37.5 `play_launch_manifest_index` + `play_launch_enforcement`, 37.6 `play_launch_executor`, 37.7 `play_launch_web`.

See [phase-37-crate_split.md](./phase-37-crate_split.md).

### Phase 38: Linux RT Scheduling Apply-Layer (complete)

Turns the shared scheduling spec (`ros-launch-manifest-sched` + `check --sched`) from validate-now into apply: `--sched <file.toml>` + `--sched-apply {off,warn,strict}` sets `SCHED_FIFO`/`SCHED_RR` + priority + CPU affinity from the resolved `posix` tier — **per thread** (`/proc/<pid>/task/*` sweep; later threads inherit). Composables scheduled via `ComponentEvent.pid` + start-time identity (38.9). **Non-root** via the ROS-free `play_launch_rt_helper` holding `CAP_SYS_NICE` only (38.10) — the main binary must never carry a file capability (`AT_SECURE` would drop `LD_LIBRARY_PATH` and its ROS libs). Verified on-kernel, unprivileged: `fifo=11/11` on every node. `SCHED_DEADLINE` deferred. User guide: [docs/guide/rt-scheduling.md](../guide/rt-scheduling.md).

See [phase-38-linux_rt_scheduling.md](./phase-38-linux_rt_scheduling.md).
Design: [apply-layer](../superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md) · [RT helper (38.10)](../superpowers/specs/2026-07-14-rt-helper-design.md).

### Phase 39: RT Example Workspace (complete)

A small real colcon workspace (`tests/fixtures/rt_workspace/`, new `rt_demo` package: sensor node, FIFO-pinned control node, one composable) that is both the runnable example behind `docs/guide/rt-scheduling.md` and an integration fixture — committed `system.toml` + manifest validated in CI, sched-apply smoke on our own nodes. Shipped: `rt_demo` package (sensor/control/filter), committed contract sidecar + user-overlay example + `system.toml`, `tests/tests/rt_workspace.rs` (5 tests; skip when unbuilt; in `just test-all`).

See [phase-39-rt_workspace_example.md](./phase-39-rt_workspace_example.md).
Design: [docs/superpowers/specs/2026-07-15-rt-workspace-fixture-design.md](../superpowers/specs/2026-07-15-rt-workspace-fixture-design.md).

### Phase 40: Contract Shipping — Provider Sidecars + User Overlay (complete)

The manifest stays a separate file; its shipping changes. Provider channel: `<name>.contract.yaml` next to `<name>.launch.{xml,py,yaml}`, installed with the package. User overlay (general user-side source): `<overlay>/<pkg>/launch/<name>.contract.yaml` (`--contracts <dir>`) — primary use today is supplying contracts for packages that ship none (Autoware; its manifest set migrates to this layout in-phase). Precedence overlay > provider > legacy `--manifest-dir` (retired in-phase once the Autoware set — `NEWSLabNTU/autoware-contract`, 75 manifests — migrates to the overlay layout); document-level replacement in v1. Requires `ScopeOrigin.path` (additive record-format change). Embedding contracts inside launch files was investigated and rejected — see the [research note](../research/manifest-annex-in-launch-files.md).

See [phase-40-contract_shipping.md](./phase-40-contract_shipping.md).
Design: [docs/superpowers/specs/2026-07-15-contract-shipping-design.md](../superpowers/specs/2026-07-15-contract-shipping-design.md).

### Phase 41: RT Config v2 — Derived Scheduling (41.1–41.5 done, 41.6 gated)

Answers design feedback on the three-part RT config (scatter, format heterogeneity, hand-written `system.toml` conflicting with derivable context, hard-wired contract→sched mapping). Scheduling context is now derived from launch+contract via a pluggable `SchedMapper` (trait + built-ins: `manual`, `rate_monotonic`, `deadline_monotonic`); the platform file shrinks to platform facts + explicit overrides, unifies on YAML, is per-target (`<stem>.system.<target>.yaml`), and ships through the Phase 40 provider/overlay channels with overlay-root discovery (`$PLAY_LAUNCH_CONTRACTS` / XDG / `/etc`) — auto-applied at launch/replay when `--sched` is absent. `check --explain` shows the merged plan with per-node provenance; `contract eject` seeds overlays from installed providers (contract + platform file together). `tests/fixtures/rt_workspace/` exemplifies the v2 model (provider sidecar, Zephyr stub proving per-target coexistence, user overlay) alongside the kept `system.toml` bridge test; `docs/guide/rt-scheduling.md` is rewritten around v2. Legacy `system.toml` parses via the bridge — deprecated but supported until nano-ros migrates (41.6, gated on that track, not yet scheduled).

See [phase-41-rt_config_v2.md](./phase-41-rt_config_v2.md).
Design: [docs/superpowers/specs/2026-07-16-rt-config-v2-design.md](../superpowers/specs/2026-07-16-rt-config-v2-design.md).

---

### Phase 43: Runtime Consumes the SystemModel (planned)

`replay --model system_model.yaml`: the checked artifact from `play_launch resolve` becomes the runtime's single source for identity, contracts (RuleEngine), and scheduling (AppliedTier) — record.json stays the spawn-info companion, bound by sha256 in `meta.inputs` (mismatch refuses). Five stages: model↔record binding, RuleEngine view-struct with `from_model`, sched from `execution.tiers`/`bindings`, web-UI scopes from the model, model-path default for `launch`. Orthogonal to Phase 42 (content vs plumbing).

See [phase-43-runtime_consumes_system_model.md](./phase-43-runtime_consumes_system_model.md).
Design: [docs/design/system-model.md](../design/system-model.md) + nano-ros RFC-0050.

### Phase 44: Vocabulary v2 + Chain-Aware Mapper (44.1–44.6, 44.8 done; 44.7 handoff)

Linux implementation of the Phase 42 designs: additive contract vocabulary (explicit path triggers timer/input/once/spontaneous, `sync:`, `buffer:`, integrator-owned `chains:` with checked `via:` links) + the `chain_aware` mapper (clock-segmented chains — PiCAS drain-toward-sink within event segments, RM boundaries, criticality-RM/DM fallback) with 9 new checker rules (`explicit-trigger`, `inherited-rate`, `once-durability`, `sync-feasibility`, `queue-drain-rate`, `chain-shape`, `chain-link`, `chain-budget`, `chain-sampling-feasibility`). Validated on rt_workspace (`points_to_cmd` chain example, mapper switched to `chain_aware`) and at scale on Autoware planning_simulator (trigger annotation from the W3 census + 3 chains, closing the `/planning/trajectory` declaration gap by authoring the real bridge node `autoware_planning_validator`) — 2 real defects found and fixed: interior clock boundaries (`chain-link` rejected any boundary but the first — fixed via the boundary-consumption rule) and a group-nesting node-identity gap (a node behind a bare `<group>` resolved to a phantom FQN — fixed by indexing identity under the nearest ancestor file scope too). `docs/guide/rt-scheduling.md` gained a "Chains" authoring-workflow section (§1.7); `launch-manifest.md` gained the vocabulary-v2 field reference + all new rule severities. SystemModel layer-2 embedding: coordination note delivered ([system-model-vocab-v2-embedding.md](../design/system-model-vocab-v2-embedding.md)); the embedding itself is the Phase 43/SystemModel track's follow-up.

See [phase-44-vocab_v2_chain_mapper.md](./phase-44-vocab_v2_chain_mapper.md).
Designs: [vocabulary v2](../superpowers/specs/2026-07-17-contract-vocabulary-v2-design.md) · [chain-aware mapper](../superpowers/specs/2026-07-17-chain-aware-mapper-design.md).

### Phase 45: Scheduling SSoT Unification (planned)

Makes the SystemModel the single source of truth for scheduling: `resolve` runs the mapper once and embeds its complete output (resolved chains, per-path ranks, mapper identity) into the model, so runtime apply, `--explain`, analysis, monitoring, and nano-ros all read scheduling from the model instead of re-deriving it. Also unifies the diagnostic renderer (Autoware's 111-line warning flood → ≤3) and the three FQN builders.

See [phase-45-sched_ssot_unification.md](./phase-45-sched_ssot_unification.md).
Design: [docs/design/system-model-sched-ssot.md](../design/system-model-sched-ssot.md).

### Phase 46: Unified SystemModel — One Complete Artifact (complete)

Makes `system_model.yaml` the ONE user-facing artifact, retiring the Phase 43 two-artifact (model + `record.json`) split. `dump <launch> -o m.yaml` and `resolve` now emit the same complete model (structure + contracts + scheduling) for BOTH parsers — contracts/sched apply on the shared scope table (`ScopeOrigin.path`, Phase 40.1) independent of parser, so `--parser python` gets full parity with Rust, not a structure-only subset; a stale pre-40.1 Python install now fails loud instead of silently degrading. `replay --model system_model.yaml` spawns directly from the model (no `record.json` companion needed) — cmdline assembly (exec path, argv, injected env, materialized param files) relocated from parse-time into `record.json` to spawn-time from the model (46.3). Also fixed nano-ros issue #236: `<node machine=>` now flows into `execution.deploy[fqn].host`. `record.json` is retired to a deprecated compat/dev surface: `dump --format record` (parser-parity tooling, `just compare-dumps`) and `replay --input-file record.json` without `--model` (warns, one release's grace) — no hard removal.

See [phase-46-unified_system_model.md](./phase-46-unified_system_model.md).
Design: [docs/design/unified-system-model.md](../design/unified-system-model.md).

---

## Reference

- **Phase 20** (Web UI Modernization): [phase-20-web_ui_modernization.md](./phase-20-web_ui_modernization.md) — Preact + htm architecture reference
- **Future ideas**: [future-considerations.md](./future-considerations.md)
- **Archived phase docs**: `archive/`
