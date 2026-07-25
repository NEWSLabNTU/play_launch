# Phase 52: Timing config + failure surfacing

**Status:** ✅ Done (2026-07-25). Independent of 50/51 except timing.rs
placement (51.4).
**Fixes:** issues 0004 (GH #4), 0005 (GH #5).

## Work items

- **52.1 — grouped CLI/config.** Split options.rs into
  `#[command(flatten)]` concern structs (web / containers / monitoring /
  sched / output); config-file parity in cli/config.rs.
- **52.2 — timing knobs.** timing.rs values fed from config: load
  timeouts/budget/attempts, verify poll, warmup, SSE reconnect. Defaults =
  current consts; document the Autoware-scale guidance (long TensorRT
  ctors → raise LOAD_TOTAL_BUDGET).
- **52.3 — startup failure surfacing.** `--on-startup-failure
  {continue|exit}` (default continue for compat); non-zero exit code in
  exit mode; persistent web banner listing failed members (needs 50.3's
  visible failure states); machine-readable summary line.
- **52.4 — error taxonomy at the actor boundary.** Typed LoadError
  {ServiceMissing, Busy, Rejected(msg), Timeout, ResponseLost} replacing
  stringly eyre on the load path; StateEvent carries it; web shows it.
- **52.5 — channel hygiene.** Bound the ComponentEvent bridge channel +
  define the overflow policy (drop-oldest + counter surfaced in web);
  audit the remaining unbounded channels; remove blanket `let _ = send`.

## Acceptance

- `play_launch launch … --load-total-budget 1200` observable in logs.
- Kill a node binary pre-launch → exit mode returns non-zero with the
  member named; continue mode shows the web banner.
- Cold Autoware receipt PASS.

## Outcome (2026-07-25)

- **52.1 grouped CLI**: `CommonOptions` split into `#[command(flatten)]`
  concern structs — `FeatureOptions` / `ContainerOptions` / `WebOptions` /
  `ContractOptions` / `SchedOptions`. Flag names unchanged; help output
  now grouped. Config-file parity via the existing `RuntimeConfig`
  sections (load knobs live in `composable_node_loading`).
- **52.2 timing knobs**: `LoadTimings` (container_actor/timing.rs) built
  from `composable_node_loading` config + CLI overrides
  (`--load-total-budget SECS`, `--load-node-timeout SECS`), threaded
  builder → actor → supervisor → ros_client. New config fields:
  `load_retry_timeout_millis`, `load_total_budget_secs`,
  `load_verify_poll_interval_secs`, `loading_event_timeout_secs`,
  `post_service_ready_warmup_ms`; the previously parsed-but-ignored
  `load_node_timeout_millis`/`load_node_attempts` are now honored.
  Defaults = former consts. Effective values logged at startup
  ("Load timings: …"). SSE reconnect sleeps NOT wired (server-side
  internals, no tuning story found — dropped from scope).
- **52.3 startup failure surfacing**: `--on-startup-failure
  {continue|exit}` (default continue). Exit mode names each failed
  member, emits machine-readable `STARTUP_SUMMARY {json}`, and shuts
  down non-zero. Two shutdown-plumbing traps found live: the actors
  listen to the BUILDER's watch channel (`member_handle.shutdown()`),
  not the replay-level one, and on Unix actors wait for the process
  GROUP SIGTERM instead of killing their own child — the exit path now
  mirrors the signal path exactly (watch + handle.shutdown + PGID kill).
  Continue mode: persistent web failure banner (click → failed facet).
- **52.4 typed load errors**: `LoadError` (thiserror) —
  ServiceMissing / ServiceNotReady / CallFailed / TimedOut /
  Unresponsive / BadRequest — replaces stringly eyre on the whole
  LoadNode path (`LoadCompletion.result: Result<_, LoadError>`).
  `Display` prefixes the kind (`timeout:`, `container-busy:`, …), so the
  web UI's Failed text carries it without a wire change.
- **52.5 channel hygiene**: the ComponentEvent bridge is bounded (1024,
  try_send) with drop-newest + power-of-two-counted warnings; the
  stuck-Loading promoter + ListNodes verification recover from
  individual losses. Remaining unbounded channels are bounded by
  construction (load completions ≤ outstanding loads) or negligible-rate
  (/parameter_events) — left documented. Blanket `let _ = state-event
  send` was already removed in phase-51 (`events::emit`).

Acceptance: `--load-total-budget 1200` visible in the startup log; bad
composable plugin + `exit` mode → exit 1 with `Failed member:
composable:/doomed` + STARTUP_SUMMARY, no orphan processes; continue
mode keeps running with the member failed in /api/nodes + banner. 288
tests green; cold Autoware receipt PASS.
