# Phase 43: Runtime Consumes the SystemModel

**Status:** 🔄 43.1–43.3, 43.5 landed (2026-07-17); 43.4 re-scoped (see below)
**Design:** [docs/design/system-model.md](../design/system-model.md) (producer side) + nano-ros RFC-0050 (consumer side)
**Builds on:** `play_launch resolve` (work item 2, landed), the `model` crate in ros-launch-manifest, Phase 36 runtime enforcement, Phase 41 sched v2.
**Relation to Phase 42:** orthogonal — 42 studies the *content* of the model (chains, mappers); 43 changes *which artifact the runtime reads*. No shared files beyond `replay.rs` plumbing; land in either order.

## Problem

`play_launch replay` re-derives everything at spawn time: it re-loads the
contract manifests (`load_manifests`, replay.rs) and re-derives the sched
plan (`SchedPlan::build` → `derive_sched_plan`). The checked, reviewed
SystemModel emitted by `resolve` is not consumed anywhere, so:

- the artifact that was checked is NOT the thing that runs (validity gap —
  a manifest edited between resolve and replay silently changes behavior);
- contract + sched derivation cost is paid on every replay;
- nano-ros and play_launch can drift on how they interpret the same
  contracts, since play_launch reads manifests, not the model.

## Scoping fact (exploration, 2026-07-17)

The SystemModel deliberately carries **no spawn information**. The spawn
pipeline (`execution/context.rs` + `execution/node_cmdline.rs` + member
actors) needs, per node: raw `cmd` (raw-exec/containers), `env`, inline
`params` + `global_params`, `params_files` *contents*, `remaps`,
`ros_args`/`args`, `respawn`/`respawn_delay`, plus the dump-level
`variables` table and `file_data` cache. None of that belongs in the
shared model (nano-ros doesn't want Linux process detail in the schema).

**Decision: two-artifact runtime.** `record.json` (LaunchDump) remains the
spawn-info artifact; the SystemModel becomes the SINGLE source for
identity, contracts, and scheduling. The two are bound by provenance: the
model embeds the record's sha256, and replay refuses a mismatched pair.
A slimmed per-FQN exec record (replacing LaunchDump wholesale) is
explicitly out of scope (revisit after 43 lands, if ever — LaunchDump
already does the job and record/replay tooling depends on it).

## Work items

### 43.1 — Bind model ↔ record

- `resolve` gains `--record <path>`: reuse an existing record.json instead
  of re-parsing (same LaunchDump-deserialize path `check` uses), and ALWAYS
  hash the effective record into `meta.inputs` (path + sha256), whether
  parsed fresh (serialize + hash the in-memory record written next to the
  model as `<out>.record.json`) or reused.
- `launch` one-shot flow becomes: parse once → write record.json → build
  model in-memory → replay from (model, record). No behavior change when
  contracts/sched are absent (empty layers).
- **Done when:** `resolve -o m.yaml` + `replay --model m.yaml` on a
  record whose hash mismatches `meta.inputs` fails loud with the paths +
  hashes; matching pair proceeds.

### 43.2 — Replay accepts `--model`; RuleEngine reads model contracts

- `replay --model <system_model.yaml>` loads the model (schema-version
  gated) instead of calling `load_manifests`.
- `RuleEngine` today is keyed on `ManifestIndex`
  (`.topics`/`.manifests[*].manifest.nodes`/`.scope_paths`). Introduce the
  narrow input the engine actually reads (rates, ages, drops, QoS, path
  budgets, lifecycle flags, externals) as an engine-owned view struct,
  with TWO constructors: `from_manifest_index` (legacy path, unchanged
  behavior) and `from_model` (`model::Contracts` + `structure`). The
  engine's rule code stops touching `ManifestIndex` directly.
- Blocking allowlist + lifecycle subscription lists derive from the model
  (`structure.topics` keys + `contracts` external marks + `lifecycle`
  flags on `structure.nodes`).
- Note: the model's endpoint keys are launch-side FQNs (reconciled by
  `model_builder`); the engine's runtime lookups already use launch FQNs,
  so this REMOVES the bare-name fallback matching the manifest path needs.
- **Done when:** `replay --model` on the rt_workspace fixture flags the
  same violations as the manifest path (existing runtime_enforcement
  tests run in both modes); without `--model`, byte-identical behavior.

### 43.3 — Sched from `model.execution`

- `AppliedTier` population (`ActorConfig.sched` for regular/container/
  composable members) gets a second source: `model.execution.bindings`
  (FQN → tier) + `model.execution.tiers[tier].<target>` sub-table →
  priority/sched_class/core, replacing `SchedPlan::build`'s
  re-derivation when `--model` is given.
- Validation (priority band, contradictions) already ran at resolve time;
  replay only re-checks platform *applicability* (target sub-table
  present, rt_helper privilege) — mapper re-derivation is gone.
- **Done when:** rt_workspace `sched_apply` test passes in `--model` mode
  with the same applied priorities as the legacy path (compare
  `/proc/<pid>/sched` assertions), and a model lacking the requested
  target's sub-table fails loud at startup, not at first spawn.

### 43.4 — Web UI + scope data from the model — RE-SCOPED (dropped)

Decision (43 implementation, 2026-07-17): the web UI's scope feed is keyed
by member names + dump scope ids — spawn-info territory, and the record is
ALWAYS present in the two-artifact runtime. Converting the model's
string-keyed scope table back into synthetic ids would be a lossy shim
with no user-visible benefit (the model's scopes were derived from the
same dump). The model's scope table serves EXTERNAL consumers (nano-ros,
tooling); the in-process UI keeps reading the record.

### 43.5 — Make the model path the default for `launch`

- `play_launch launch` uses the in-memory model unconditionally (it just
  built it); the manifest/sched re-derivation path remains only for bare
  `replay` without `--model` (records from older versions, hand runs).
- Deprecation note in docs/guide: `check` stays the diagnostic front-end;
  `resolve` is the build step; `replay --model` is the runtime.
- **Done when:** `just build && play_launch launch rt_demo
  bringup.launch.xml --sched …` exercises resolve→replay-from-model
  end-to-end and the guide documents the three-verb split.
- Landed: launch = record → resolve (`--record record.json`, refuses on
  contract Errors) → replay `--model system_model.yaml`; verified on
  rt_workspace (contract source + sched source logs both say SystemModel;
  4 members spawn).

## Non-goals

- Replacing LaunchDump / record.json as the spawn-info artifact.
- Composable LoadNode args, params-file contents, or env in the model.
- Chain-aware mapper work (Phase 42's territory).
- nano-ros-side model consumption (tracked by nano-ros RFC-0050's own
  phase).

## Risks / notes

- `RuleEngine` refactor (43.2) is the largest item — its rules read
  `ManifestIndex` in ~6 places; the view-struct indirection must be
  behavior-preserving under the existing runtime_enforcement tests.
- Model staleness UX: 43.1's hash gate must produce an actionable message
  (`re-run play_launch resolve`), not a bare refusal.
- `run` (single-node path) synthesizes a LaunchDump in memory and skips
  contracts entirely — untouched by this phase.
