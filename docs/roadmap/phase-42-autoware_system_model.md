# Phase 42: Autoware System Model Study → Chain-Aware Scheduling Design

**Status:** 📋 Planned (design draft under discussion)
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
- **42.1** Graph export tooling: extend the causal-dag machinery to export the
  declared causal graph (nodes, causal edges, `state:` cuts, paths, budgets)
  as JSON/DOT from `check`.
- **42.2** Measured model: interception-enabled planning_simulator run;
  per-topic rates + frontier data joined against the declared graph.
- **42.3** Source census (sampled): trigger semantics classification for
  representative node patterns (timer/event/hybrid, sync policies).
- **42.4** System-model report: `docs/research/autoware-system-model.md`
  answering Q1–Q5 (cycles, junctions, chain composability, triggers,
  rate monoculture).
- **42.5** Contract vocabulary proposal: manifest extensions justified by
  42.4 findings (trigger, cross-scope chains, chain semantics tag) —
  platform-agnostic, optional, additive.
- **42.6** Chain-aware mapper design spec (design only; implementation is a
  later phase). Decides the fate of the interim `criticality_rm` idea (41.7).

## Order

42.1 → 42.2/42.3 (parallel) → 42.4 → 42.5 → 42.6.

## Out of scope

Mapper implementation; nano-ros work; Autoware source changes; WCET/schedulability analysis.
