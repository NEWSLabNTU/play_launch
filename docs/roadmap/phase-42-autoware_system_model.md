# Phase 42: Autoware System Model Study → Chain-Aware Scheduling Design

**Status:** 🔄 In progress — waves 42.1–42.4 (study) complete; 42.5–42.6 (proposal/design) pending
**Design draft:** [docs/design/autoware-system-model-study.md](../design/autoware-system-model-study.md)
**Builds on:** Phase 41 (RT config v2; exercise findings), Phase 29 (interception), manifest `paths:`/`state:` semantics.
**Gates:** 41.7 (criticality mapper) — deliberately held until this study lands.

## Overview

The Phase 41 Autoware exercise validated the v2 mechanics but exposed the
mapper's model gap: contracts express cause-effect *chains* (paths, budgets,
polled-vs-causal subscriptions), while mappers consume node-local scalars.
Real Autoware adds cycles, non-1:1 junctions (sync/fan-in), and mixed
trigger semantics. This phase studies the real system model empirically —
declared (contracts) vs measured (interception) vs source (sampled) — then
designs the chain-aware mapper on evidence.

## Work items

- **42.0** Profiling infra readiness (from `.superpowers/sdd/p42-infra-readiness.md`,
  2026-07-16 idle-baseline run: composable injection 100%, rates plausible):
  (a) persist hash→topic-name map in `frontier_summary.json`/`stats_summary.json`
  (consume `TopicNameDeclared` in the Phase 29 consumer; today the join rides on
  the RuleEngine's incidental `discovered_topic_types.tsv`); (b) drop counters
  for SPSC ring overflow (producers silently discard `Err(Full)`) reported in
  the summaries; (c) fix CLAUDE.md's stale `autoware_config.yaml` reference;
  (d) 42.2 runbook: engaged autonomous scenario (goal pose + engage) so the
  sim clock advances and frontier stamps are non-zero — idle baseline showed
  68/161 declared topics active.
- **42.1** ✅ Graph export tooling: `play_launch check --export-graph` (JSON/DOT),
  reuses Phase 35's `manifest_graph`. Autoware: 74 nodes/169 topics/13 cycles
  (12 state-cut, 1 not — cross-scope, invisible to per-manifest `causal-dag`).
  Schema: `docs/design/causal-graph-export.md`.
- **42.2** ✅ Measured model: engaged planning_simulator run (1.37 m/s, 1.54M
  events, 610 measured topics, 119/169 declared active, 0 ring drops). Found
  + corrected a stamped-topic double-count bug (source fix still needed);
  join tool `scripts/p42_join.py` committed.
- **42.3** ✅ Source census (9 nodes): 4 timer / 2 event / 2 hybrid / 1
  runtime-configurable trigger disciplines; Q1 cycle diagnosed as a
  contract-authoring bug (one missing `state: true`), not a vocabulary gap.
- **42.4** ✅ System-model report: `docs/research/autoware-system-model.md`
  answers Q1–Q5 (cycles, junctions, chain composability, triggers,
  rate monoculture) and lists consequences for 42.5 (vocabulary), the
  checker, the mapper, and infra follow-ups.
- **42.5** Contract vocabulary proposal: manifest extensions justified by
  42.4 findings (trigger, cross-scope chains, chain semantics tag) —
  platform-agnostic, optional, additive.
- **42.6** Chain-aware mapper design spec (design only; implementation is a
  later phase). Decides the fate of the interim `criticality_rm` idea (41.7).

## Order

42.1 → 42.2/42.3 (parallel) → 42.4 → 42.5 → 42.6.

## Out of scope

Mapper implementation; nano-ros work; Autoware source changes; WCET/schedulability analysis.
