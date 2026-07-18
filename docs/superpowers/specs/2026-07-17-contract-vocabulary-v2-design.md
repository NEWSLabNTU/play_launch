# Contract Vocabulary v2 — Trigger, Sync, Buffer, Chains — Design

**Date:** 2026-07-17
**Status:** Approved (design), pending implementation
**Phase:** 42.5 (see `docs/roadmap/phase-42-autoware_system_model.md`)
**Evidence base:** `docs/research/autoware-system-model.md` (Phase 42 study, Q1–Q5).
**Feeds:** SystemModel layer 2 (`docs/design/system-model.md`) — every field here
is designed to embed verbatim in the resolved artifact; W6 chain-aware mapper spec.

## Decisions

| Question | Decision |
|---|---|
| Direction | **Enrich `paths:`** — the existing causal primitive carries the new facts; no parallel node-level trigger enum. |
| Trigger declaration | **Explicit closed taxonomy** on every path: `timer` / `input` / `once` / `spontaneous`. Implicit `input: []` = self-clocked was rejected: the author must state the clock, not imply it. |
| Cross-scope chains | **Named composition**, integrator-owned `chains:` section with explicit `via:` connecting topics; checker verifies every link. |
| Buffering | `buffer: latest | queue` discriminator on `state:` subs. |
| Fan-in sync | `sync:` policy on `input`-triggered paths. |
| Mode conditioning | **Deferred.** Range facts (`min_rate_hz`) express the engaged-mode obligation; per-mode contract variants are future work. |
| Per-sub consumption rate (`consumes_hz`) | **Deferred** (YAGNI) — measured lens supplies it; no consumer needs the declared form yet. |

All changes are **additive and optional**; existing contracts (75 Autoware files,
rt_workspace) stay valid unmodified.

## 1. Path triggers

Every path may declare `trigger:` (one of four kinds). When absent, the legacy
derivation applies: non-empty `input:` list ⇒ input-triggered (today's 75
contracts parse identically); empty or missing `input:` with no `trigger:` ⇒
**unclassified** — no trigger facts are derived (never silently assumed to be a
timer), and the `explicit-trigger` lint (§5) flags it. A new lint nudges all
paths toward explicit triggers.

```yaml
nodes:
  vehicle_cmd_gate:
    paths:
      forward:                                  # message-driven
        trigger: { input: [control_cmd_in] }
        output: [control_cmd_out]
        max_latency_ms: 5
      status_tick:                              # periodic, self-clocked
        trigger: { timer: { rate_hz: 10 } }
        output: [gate_status]
  map_loader:
    paths:
      publish_map:                              # one-shot at startup
        trigger: once
        output: [map]
  remote_interface:
    paths:
      operator_cmd:                             # externally caused, irregular
        trigger: spontaneous
        output: [external_cmd]
```

Semantics and consumers:

| Trigger | Meaning | Mapper fact | Checker rules |
|---|---|---|---|
| `timer: {rate_hz}` | Periodic self-clocked callback | rate is REAL (RM input); CPU needed each period | timer path MUST carry `rate_hz` inside the trigger; `rate_hz` elsewhere on the path is an error |
| `input: [eps]` | Output caused by these inputs | `max_latency_ms` = relative deadline (DM input); rate inherited from upstream | declaring a path-level `rate_hz` → warning "inherited rate, do not author" |
| `once` | Published once (startup latch) | scheduling-irrelevant; excluded from RT derivation | output topic QoS not `transient_local` → warning (late joiners lose the message) ; freshness/rate rules exempt |
| `spontaneous` | Caused outside the graph (operator, network, hardware) | event-like, no upstream inheritance; non-RT unless overridden | exempt from rate-hierarchy/contradiction rules unless the pub endpoint declares an explicit `min_rate_hz` bound |

Evidence: W3 census — 4 timer / 2 event / 2 hybrid / 1 configurable; hybrid nodes
(`vehicle_cmd_gate`) dissolve into one path per trigger. W2 — false contradiction
warnings (43/78 pairs) largely stemmed from treating inherited rates as authored
facts. Runtime-configurable nodes declare the **deployed** discipline: a contract
describes a deployment, not the code's option space.

`state: true` subscriptions interact naturally: a `state:` sub never appears in
any `trigger.input` (it cannot cause an output). Lint §5 enforces the coherence.

## 2. Fan-in sync (`sync:` on input-triggered paths)

```yaml
paths:
  fuse:
    trigger: { input: [cloud_top, cloud_left, cloud_right] }
    sync:
      policy: approximate        # exact | approximate | timeout_any
      max_interval_ms: 50        # match window (exact/approximate)
      timeout_ms: 100            # timeout_any: publish partial set after this
    output: [cloud_fused]
    max_latency_ms: 30
```

- `exact` — message_filters ExactTime; `approximate` — ApproximateTime;
  `timeout_any` — collect-until-timeout, publish partial (the
  `concatenate_and_time_sync_node` pattern, W3).
- Checker: `max_interval_ms` must be satisfiable given declared input rates
  (window ≥ max inter-arrival spread implied by the slowest input) — new rule
  `sync-feasibility`, severity warning.
- Chain latency semantics: the output's data age inherits from the **oldest
  matched input** (TIMEX data-age composition; see
  `docs/research/data-quality-semantics.md` §2).

## 3. Buffer discriminator on `state:` subs

```yaml
sub:
  control_cmd: { state: true }                  # buffer: latest is the default
  twist:       { state: true, buffer: queue }   # drained batch-wise per tick
```

- `latest` (default = today's implicit semantics): staleness is the failure mode
  — monitors watch data age.
- `queue`: bounded queue drained by the consuming timer callback (EKF pattern,
  `ekf_localizer.cpp:137-215`, W3) — backlog is the failure mode; the consuming
  timer's rate must satisfy the SUM of producer rates (new checker rule
  `queue-drain-rate`, warning).

## 4. Cross-scope chains (`chains:`)

Integrator-owned (root contract or overlay — same owner as the platform file).
Composes per-scope paths into end-to-end budgets with **explicit** connecting
topics; no silent FQN matching.

```yaml
chains:
  sensing_to_actuation:
    semantics: reaction              # reaction | age  (TIMEX distinction)
    max_latency_ms: 150              # E2E budget
    segments:
      - { scope: /perception, path: preprocess }
      - { via: /perception/objects }
      - { scope: /planning, path: plan }
      - { via: /planning/trajectory }
      - { scope: /control, path: follow }
```

- Checker rule `chain-link` (error severity): every `via` topic must exist, the
  preceding segment must output it, the following segment must consume it. The
  Phase 42 Q3 hole (`/planning/trajectory`: 5 declared subs, 0 declared pubs)
  becomes a loud error instead of silent non-composition.
- Checker rule `chain-budget`: sum of segment `max_latency_ms` (+ declared
  `max_transport_ms`) must fit the chain budget — extends the existing
  budget-overflow family to chains.
- `semantics:` selects composition math: `reaction` (first-reaction latency) vs
  `age` (data staleness at the sink) — junction sampling makes the two diverge
  (W2 measured both junction species).
- Chains are the W6 mapper's unit of assignment (PiCAS-style): chain
  criticality/budget orders member nodes.

## 5. Lints (authoring hygiene)

- `explicit-trigger` (info→warning after migration): path without `trigger:`.
- Generalized `state-consistency` (extends the shipped rule): a sub neither
  `state:` nor referenced by any `trigger.input` on its node → warning (the Q1
  authoring-bug shape, caught mechanically).
- `inherited-rate`: `rate_hz` authored on an input-triggered path → warning.
- `once-durability`: `once` output on a volatile topic → warning.

## 6. Schema/serde home and compatibility

- Types: `ros-launch-manifest` `types` crate — additive fields on the existing
  path struct (`trigger`, `sync`), sub struct (`buffer`), new top-level `chains`
  section. Serde: all optional, `deny_unknown_fields` policy unchanged.
- SystemModel: layer 2 embeds the resolved forms verbatim (chains resolved to
  concrete FQNs). Coordinate the schema addition with the SystemModel track —
  additive, but the model crate's contracts layer should gain the same structs
  rather than a parallel copy.
- Manifest format version: additive ⇒ no version bump; document in
  `launch-manifest.md` with a vocabulary-v2 section.

Per the SystemModel-as-scheduling-SSoT decision
([system-model-sched-ssot.md](../../design/system-model-sched-ssot.md),
work items in [phase-45](../../roadmap/phase-45-sched_ssot_unification.md)),
this coordination is now committed: the authored facts here (`trigger:`,
`sync:`, `buffer:`, `chains:`) embed into SystemModel layer 2 verbatim from
the `types` crate structs, alongside the resolved chains/per-path ranks
that layer 3 gains — one derivation at `resolve` time, no parallel copies.

## 7. Out of scope

Per-mode contract variants (mode conditioning); `consumes_hz`; automatic trigger
inference from source; mapper implementation (W6 spec consumes this).

## 8. Migration plan (sketch, phases when implemented)

1. Types + parse + lints (warn-only) in ros-launch-manifest.
2. Checker rules (`sync-feasibility`, `queue-drain-rate`, `chain-link`,
   `chain-budget`, `once-durability`).
3. rt_workspace fixture adopts explicit triggers + one chain (runnable example).
4. Autoware contracts: annotate the W3-censused nodes' triggers + one
   sensing→actuation chain in the autoware-contract repo (validates at scale).
5. SystemModel layer-2 embedding (coordinate with that track).
