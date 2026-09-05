# play_launch Roadmap

Started October 2025. 36 phases total; 27 complete, 3 in progress, 6 planned.

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
| 47 | CLI cleanup + hard record.json removal | ✅ A1–A2, B1–B6 | 2026-07-20 |
| 48 | Detangle namespace from scope tree | ✅ minimal + full | 2026-07-20 |
| 49 | Retire last record.json reader + replay --model degradations | 📋 Planned | — |

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
Design: `docs/launch-manifest.md` in the `ros-launch-manifest` crate — moved
there in phase 31, and since phase-55 W2 that crate is a git dependency pinned
by tag, so the file is no longer in this repo (find it under
`~/.cargo/git/checkouts/ros-launch-manifest-*/`).

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

Makes `system_model.yaml` the ONE user-facing artifact, retiring the Phase 43 two-artifact (model + `record.json`) split. `dump <launch> -o m.yaml` and `resolve` now emit the same complete model (structure + contracts + scheduling) for BOTH parsers — contracts/sched apply on the shared scope table (`ScopeOrigin.path`, Phase 40.1) independent of parser, so `--parser python` gets full parity with Rust, not a structure-only subset; a stale pre-40.1 Python install now fails loud instead of silently degrading. `replay --model system_model.yaml` spawns directly from the model (no `record.json` companion needed) — cmdline assembly (exec path, argv, injected env, materialized param files) relocated from parse-time into `record.json` to spawn-time from the model (46.3). (The `<node machine=>` → `execution.deploy[fqn].host` mapping added here for nano-ros #236 was REVERTED on 2026-07-31 — `machine=` is ROS 1 syntax, not ROS 2.) `record.json` is retired to a deprecated compat/dev surface: `dump --format record` (parser-parity tooling, `just compare-dumps`) and `replay --input-file record.json` without `--model` (warns, one release's grace) — no hard removal.

See [phase-46-unified_system_model.md](./phase-46-unified_system_model.md).
Design: [docs/design/unified-system-model.md](../design/unified-system-model.md).

---

## Reference

- **Phase 20** (Web UI Modernization): [phase-20-web_ui_modernization.md](./phase-20-web_ui_modernization.md) — Preact + htm architecture reference
- **Future ideas**: [future-considerations.md](./future-considerations.md)
- **Archived phase docs**: `archive/`

- **Phase 50** — ✅ FQN member identity + full web visibility (issue 0001 resolved, 0006 partial)
- **Phase 51** — ✅ state ownership refactor: reducer + container split (issues 0002/0003 resolved)
- **Phase 52** — ✅ timing config + failure surfacing (issues 0004/0005 resolved)
- **Phase 53** — ✅ web UI organization remainder (issue 0006 resolved)
- **Phase 54** — ✅ ordered parameter sources (issue 0007; the ordering was
  built then discarded three times before 2026-08-02 completed it end to end)
- **Phase 55** — 📋 launch toolchain consolidation: fold layer 2 into this
  repo, keep its workspace separate. **Blocked on nano-ros amending RFC-0060**
  (Stable, specifies three repositories).
  [phase-55-launch-toolchain-consolidation.md](./phase-55-launch-toolchain-consolidation.md).
  Design: [docs/design/launch-toolchain-topology.md](../design/launch-toolchain-topology.md).
  Counterpart: nano-ros phase-332.
- **Phase 56** — ✅ CLI verb reshape for 0.9.0: `replay` → `up`; the removed
  verb errors naming its replacement, and the hidden variant deletes at 1.0.0.
  The wave that moved `resolve`/`dump`/`check`/`plot`/`contract` onto
  `ros-launch-resolve` was **reverted** (Phase 56 amendment, D2/D5) — it left
  no `play_launch` verb able to write the `system_model.yaml` that `up`
  requires, and pointed users at a developer binary they do not have.
  [phase-56-cli-verb-reshape.md](./phase-56-cli-verb-reshape.md).
- **Phase 57** — ✅ mixed-criticality RT demo + Chrome timeline export. Proves
  the RT apply path does something: on one CPU the safety chain goes from
  272/1013 frames past its 60 ms deadline to 0/1039 (p99 106.6 → 13.3 ms), at
  a 26–37% cost to best-effort throughput. Verified against live processes with
  `chrt -p`, because both runs' logs were byte-identical until this phase added
  apply logging. Workspace: `examples/rt_av_demo/`. Report:
  [docs/reports/rt-mixed-criticality/](../reports/rt-mixed-criticality/).
  [phase-57-rt-mixed-criticality-demo.md](./phase-57-rt-mixed-criticality-demo.md).
- **Phase 58** — 🚧 deriving scheduling from contracts. Cost was unauthorable in
  the v2 platform schema, so `sched_derive.rs` substituted a path's *deadline*
  for its cost. W1 (authorable) and W4 (reservations) shipped in Phase 60;
  **W2 is done** — `play_launch measure` turns a run into `budget_us` from
  `CLOCK_THREAD_CPUTIME_ID` rather than asking an integrator to invent one,
  and reports response time beside it because the same rt_av_demo callback
  costs 3.05 ms of CPU and 40.43 ms of wall clock. Remaining: deadline
  decomposition (W3) and synthesis (W5). Study:
  [docs/research/scheduling-derivation-prior-art.md](../research/scheduling-derivation-prior-art.md).
  [phase-58-scheduling-derivation.md](./phase-58-scheduling-derivation.md).
- **Phase 66** — 🚧 cgroup-per-container: `isolated` already surrendered
    everything that makes a ROS container a container (shared address space,
    shared executor, one participant); cgroup v2 gives the resource and
    lifecycle half back to separate processes, and gives it back **better** —
    `memory.oom.group` turns the failure model into a per-container choice
    (`=1` all members die together, `=0` only the offender does) where both a
    real container and plain processes each give you one row and no say. The
    grouping needs no new config: the launch file already names each
    composable's target container, and a composable inherits its container's
    cgroup at fork, so per-container grouping costs **zero** changes to
    `play_launch_container`. Also fixes a wrong number — summing child RSS
    over-counts shared pages **2.4x**. Explicitly no performance claim: process
    count, thread count and runqueue depth are untouched.
    [phase-66-cgroup-per-container.md](./phase-66-cgroup-per-container.md).
  - **Phase 69** — ✅ the field table: one source for the contract grammar.
    The manifest parser had **no unknown-key rejection anywhere**, so
    `max_latencyy: 5ms` deleted a budget and `rate_hzz: 100` deleted the
    declaration `derivable-rate` reads — silencing the diagnostic that pointed
    at the mistake, while `check` reported `1 clean`, exit 0. The grammar lived
    in three places nothing compared; **six of 66 fields appeared nowhere in
    the 1752-line specification**. `field_table.rs` is now the single source
    and `docs/format-reference.md` is generated from it. Severity was measured
    first: across 42 files and 132 key paths **exactly one key was dead**, so a
    hard error rather than a deprecation window. Found on the way: a parse
    failure silently dropped the whole file at exit 0, and a vacuous test whose
    scope path named topics that did not exist.
    [phase-69-contract-field-table.md](./phase-69-contract-field-table.md).
  - **Phase 68** — 📋 contract consequences: analysis, mapper, verification,
    retirement. Makes the checker and mapper read what phase 67 added, then
    RETIRES the old paths only after a running system agrees. Fixes a silent
    wrong answer measured on a two-output node — a route that never traverses
    `masks` is charged for it, `to_tracks` costing **45ms where the truth is
    30ms**, tracking an unrelated sibling path one-for-one — plus the sampling
    cost missing from `scope-budget` (**40% of the real total** on
    `rt_workspace`) and six declared facts no arithmetic reads. Two of the
    mapper's jobs are REFUSALS, not derivations: an exclusive group is one
    schedulable entity rather than N, and a per-thread reservation is refused
    where exclusive paths span threads. Retirement is gated on `rt_av_demo`
    reproducing its published 217 -> 9 missed-frame result, not on the checker
    agreeing with itself.
    [phase-68-contract-consequences.md](./phase-68-contract-consequences.md).
  - **Phase 67** — 📋 contract primitives: the vocabulary. A contract states
    what the code does and what it must achieve; anything computable from those
    is derived. Measured, `rt_workspace`'s three-node contract writes **100 nine
    times** and is about a third irreducible. Adds jitter as a requirement,
    deadline-miss handling, path exclusion (from which callback groups derive —
    nano-ros already keys its tiers on `(node, callback_group)`), and the two
    ROS 2 QoS policies we describe but never ask for. Strictly ADDITIVE: nothing
    is removed, nothing is consumed, and the acceptance criterion is that every
    existing contract resolves BYTE-IDENTICALLY.
    [phase-67-contract-primitives.md](./phase-67-contract-primitives.md).
  - **Phase 65** — 📋 mixed isolation granularity: phase 61's W3 made concrete
  after a second vehicle hit the cost. One container, per-composable policy:
  a short isolate list keeps fork+exec (segfault boundary, per-node OOM,
  restart) for the nodes that need it; the rest load as threads on the paths
  that already exist. 93 processes and ~110 DDS participants become ~8 and
  ~34 on the motivating stack.
  [phase-65-mixed-isolation-granularity.md](./phase-65-mixed-isolation-granularity.md).
- **Phase 64** — 📋 a private load channel for the isolated container:
  play_launch speaks LoadNode over rmw to its own container binary, and a
  launch of ~150 fresh processes jams that layer exactly when load status
  matters — one AutoSDV launch produced 14 spurious 30s LoadNode timeouts and
  8 ComponentEvent waits, all false alarms confirmed loaded by ListNodes.
  Both ends of the conversation are ours; a per-container unix socket retires
  five timeout-and-fallback mechanisms on the default path, and LoadNode
  remains for stock containers, which have no other interface.
  [phase-64-isolated-container-ipc.md](./phase-64-isolated-container-ipc.md).
- **Phase 63** — 📋 the duration type campaign: executing Phase 59 as a type
  change rather than a parser change, across ~455 Rust sites in two tag-pinned
  repositories. W1 (the `Duration` type, deprecated-name aliases) is built.
  Chosen over the cheap parse-only option because that leaves every consumer
  holding a bare `f64` whose unit lives in a field name — the shape of the bug
  where a *deadline* was substituted for a *cost*.
  [phase-63-duration-type-campaign.md](./phase-63-duration-type-campaign.md).
- **Phase 59** — 📋 timing vocabulary: units move onto the value
  (`max_latency: 12ms`), because `budget_us: 8` meaning 8 ms is a 1000x error
  that type-checks. Split out of Phase 58 — independent work, and cheaper now
  than it will ever be again: the real corpus uses two renameable field names.
  [phase-59-timing-vocabulary.md](./phase-59-timing-vocabulary.md).
- **Phase 60** — ✅ completed the Linux scheduling realizer. The `posix`
  realizer emits exactly one policy: `SCHED_FIFO` for every ranked node,
  nothing at all for the rest. `SCHED_RR` parses and is never emitted,
  `SCHED_DEADLINE` has a *"not applied in v1"* warning, and `budget_us` /
  `deadline_policy` reach `system_model.yaml` and are consumed by nothing.
  Migrates the apply layer to `sched_setattr`, adds a typed `posix:` block,
  derives RR on ties and DEADLINE under an opt-in all-or-nothing rule, and
  makes `isolated_cpus` mean something. Absorbs Phase 58 W1+W4 — a
  reservation's runtime has no source until cost is authorable. Four findings
  shape it: a per-TID sweep multiplies a reservation ~11x; DEADLINE outranks
  every FIFO thread regardless of priority; a DEADLINE task cannot `fork()`
  without reset-on-fork, and isolated containers fork per composable; and an
  exclusive cpuset is **measurably** unreachable from an unprivileged session
  on stock Ubuntu, so the product path errors with a setup command while the
  A/B arm runs privileged. **Outcome:** the three-arm A/B says reservations
  LOSE to fixed priority on vanilla `rclcpp` (217 → 9 misses under
  `SCHED_FIFO`, 217 → 42 under `SCHED_DEADLINE`), at a best-effort cost of
  −16% vs −5% — CBS's sporadic release model does not match an
  `rclcpp::spin()` event loop. Reported, not tuned away.
  Result: [docs/reports/rt-mixed-criticality/reservations-result.md](../reports/rt-mixed-criticality/reservations-result.md).
  Design:
  [2026-08-10-linux-sched-feature-surface-design.md](../superpowers/specs/2026-08-10-linux-sched-feature-surface-design.md).
  [phase-60-linux-sched-surface.md](./phase-60-linux-sched-surface.md).
- **Phase 61** — ✅ W1: the edge startup storm. A 144-process Autoware launch
  on a 12-core AGX Orin put **484 tasks in the runnable queue at load1 203**
  and held ~10 of 12 cores for 49 s; on the vehicle the OOM killer took the
  operator's GNOME. Measured, and the obvious fix measured wrong: pacing the
  spawns doubled a 10.6 s startup for a 10% cut in peak runnable tasks,
  because the cost of a wide launch is the processes *existing*, not their
  starting together. What moves it is the process count — `--container-mode
  observable` brings the same system up on **3.9 cores instead of 10.2**,
  peak load1 45 vs 190, 1.4 GiB vs 3.5, and finishes *faster* (8.4 s vs
  10.7 s). So both throughput gates ship off, the `MemAvailable` floor ships
  on (it never blocks until memory is actually short), children get
  `oom_score_adj +300` so the kernel picks a node over the desktop, and
  `isolated` mode warns when it would fork more than `4 * ncpu` processes.
  Two defects found on the way: `max_concurrent_load_node_spawn` was dead
  config, and `Startup complete` fired before anything had started
  (issue #0016). Also unblocked the reporter's project: 0.9.0 could not parse
  `<arg><choice>` at all.
  [phase-61-edge-startup-storm.md](./phase-61-edge-startup-storm.md).
