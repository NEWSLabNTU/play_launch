# play_launch Roadmap

Started October 2025. 29 phases total; 25 complete, 2 in progress, 2 planned.

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

---

## Reference

- **Phase 20** (Web UI Modernization): [phase-20-web_ui_modernization.md](./phase-20-web_ui_modernization.md) — Preact + htm architecture reference
- **Future ideas**: [future-considerations.md](./future-considerations.md)
- **Archived phase docs**: `archive/`
