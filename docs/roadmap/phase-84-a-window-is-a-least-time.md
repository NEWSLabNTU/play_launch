# Phase 84 - a window is a least time, and its notice is charged once

Status: **complete** (2026-09-29). The grammar's text shipped as
`ros-launch-manifest` **v0.1.47** (text only, workspace still 0.1.7) and is
pinned here. Follows phase 83. Asked for by the Autoware Safety Island's
unit phase8-W12, from its takeover trace (`docs/takeover-trace.md`
there, finding 1).

## Why

Phase 83 gave a ladder rung a `window:`. The island's takeover request holds
for 10 s, then the handler calls the comfortable stop. Its runs (phase8-W7)
measured the request ending 10000.00 ms or 10099.00 ms after it went on,
because the handler reads the deadline on its 100 ms tick. Set against the
checker's WINDOWS term (the rung's route 110 + the window 10000 = 10110 ms,
from the verdict), one run was 30.21 ms over on the island clock and another
77.09 ms over on host time. Two readings were possible:

- the window is a MAXIMUM ("stay at most this long", as rlm v0.1.46 said),
  so the handler is late and the checker under-charges by a tick; or
- the window is a MINIMUM, the driver's guaranteed time (Drive Pilot's
  10 s), and the tick is the notice of the deadline, which belongs to
  whatever runs after it.

The code settles it. `check_fault_reaction` charges each rung below a window
`route + window`, then the rung's OWN route, walked from the guard: for the
island that route starts at `/mrm_handler/call_mrm`, whose 110 ms is the
100 ms tick plus the 10 ms call. The tick the handler reads the deadline on
was therefore charged -- as the first hop of the route below, the same way
the checker charges the detection tick after the 500 ms staleness window
(`hpc_loss`: detect 500, then a route that starts with the tick). The W7
table cut branch B at the request-OFF edge, which lies inside that route,
and so counted the tick twice. Nothing in the arithmetic was wrong; two
things around it were:

1. rlm's text said "at most", which is the opposite of the promise the
   number makes, and which nothing can meet exactly (every enforcer notices
   a deadline some time after it passes).
2. The checker never CHECKED that the route below holds the notice. It held
   it on the island only because `call_mrm`'s declared 110 ms happens to
   contain the tick. A window owned by a node with a slower clock, or by a
   node the route below does not start at, would have been under-charged
   silently.

Adding a tick to WINDOWS instead (the other candidate) would charge the same
100 ms twice: that is slack, not a fix.

## What changed

- **The meaning** (rlm v0.1.47, `docs/launch-manifest.md` and the field
  table): while it stays available a windowed rung lasts AT LEAST
  `duration`; the rung below starts no sooner than the deadline (the rung's
  route, then `duration`) and no later than its own route after it. The
  window ends at the deadline, not where the owner notices it.
- **`window-expiry`** (new rule, `manifest_loader.rs`, `check_window_expiry`):
  the node the window's `param:` names is its owner; its slowest timer path
  that publishes the rung's output is the clock it reads the deadline on.
  For every rung charged the window:
  - error if the rung's route does not start at the owner (nothing charges
    the owner's notice);
  - error if that first hop is charged less than the owner's period (the
    difference is charged nowhere);
  - warning, once per window, if no timer of the owner publishes the rung's
    output (when the deadline is noticed is undeclared).
  The check is necessary, not sufficient: the checker sees the hop's
  latency, not how it divides between the wait for the tick and the call.
- **`check --explain`** prints the window as `window >=10000.00`, a footer
  line saying where the notice is charged, and per window rung the interval
  the request ends in:

```
odd_exit  takeover_request  window   100.00      0.00  110.00  window >=10000.00         -  30000.00         -
odd_exit  comfortable_stop  rung     100.00  10110.00  110.00    9996.67 derived  20316.67  30000.00   9683.33
  TOTAL = DETECT + WINDOWS + ROUTE + SETTLE; WINDOWS is the route and window of every windowed rung passed on the way, up to its deadline.
  A window is a least time (`window >=`); noticing its deadline is the first hop of the ROUTE below it (`window-expiry`), never a second charge.
  odd_exit/takeover_request: lasts at least 10000.00ms once on, and ends within 10110.00ms: /mrm_handler reads the deadline on its 100.00ms timer ('on_timer'), charged inside /mrm_handler/call_mrm 110.00ms, the first hop of the route below
```

  (the island's live contract). Every TOTAL is unchanged.
- `ReactionHop` carries its charge (`ms`), which the rule reads.

## Tests

`tests/tests/manifest_check.rs`: `takeover_ladder_fits_and_explains_itself`
now has a handler tick (10 Hz, `on_timer` publishes `/tor_state`) and asserts
the `ends within 10110.00ms` note and no `[window-expiry]`;
`window_expiry_names_a_notice_the_route_does_not_hold` (new fixture
`contract_takeover_expiry_tick`: a 5 Hz tick under a 110 ms hop, 90 ms
charged nowhere, one error per rung below, budget unchanged);
`window_expiry_names_an_owner_off_the_route` (new fixture
`contract_takeover_expiry_owner`: the window bound to the planner, off the
route, plus the undeclared-notice warning once);
`takeover_budget_rules_fire_on_their_fixture` gains the warning (its handler
declares no tick). `cargo test --test manifest_check`: 39 passed.

## Consumers

The island's W6 demo contracts keep their verdicts (`l3_takeover` passes,
`l3_takeover_window20` and `l3_takeover_65kmh` fail `ladder-rung-budget` on
the comfortable-stop rung and nothing else). nano-ros vendors this checker;
its pin moves in a chore PR.
