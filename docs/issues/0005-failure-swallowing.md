---
id: 5
title: "Failure swallowing — startup-with-failures continues silently; dropped sends; unbounded event channel; stringly errors"
status: resolved
type: tech-debt
severity: medium
github: https://github.com/NEWSLabNTU/play_launch/issues/5
---

- `signal_handler.rs:494`: "Startup complete with failures" logged, run
  continues — a partially failed launch looks started; no exit-code or UI
  surfacing.
- 46 `let _ = …send(…)` sites drop channel failures (web transitions lost
  silently).
- Unbounded ComponentEvent bridge channel (`container_actor/mod.rs:391`) —
  DDS-rate producer, no backpressure.
- `eyre` strings in 56 files; only sched + ipc have typed errors — callers
  can't distinguish busy/missing/rejected.

Plan: `docs/roadmap/phase-52-config-and-failure-surfacing.md`.

## Resolution (phase-52, 2026-07-25)

`--on-startup-failure {continue|exit}`: exit mode names failed members,
emits `STARTUP_SUMMARY {json}`, exits non-zero (and correctly reaches the
actors' shutdown channel + PGID); continue mode shows a persistent web
banner. Typed `LoadError` replaces eyre on the load path. ComponentEvent
bridge bounded (1024, drop-newest + counted). State-event send drops were
eliminated in phase-51 (`events::emit`).
