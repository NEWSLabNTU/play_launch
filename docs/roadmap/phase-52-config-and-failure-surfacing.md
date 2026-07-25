# Phase 52: Timing config + failure surfacing

**Status:** 📋 Planned (2026-07-25). Independent of 50/51 except timing.rs
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
