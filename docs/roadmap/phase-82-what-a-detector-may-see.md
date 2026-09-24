# Phase 82 - what a detector is allowed to see, and what a reaction is allowed to cross

Status: **proposed** (2026-09-25). Follows phase 81 (the docs describe the
code) and the service-edge fix on `main` (`c1474126`, a fault reaction crosses
a service). Ships as `ros-launch-manifest` **v0.1.43** for the one grammar
change, pinned here.

## Why

The Autoware Safety Island's phase 6 encoded the Reference Design WG's L4
design as a contract and ran it. Three things the checker did were wrong,
and one thing it could not do was real:

1. **Saying less about a fault bought slack.** On the L4 fixture,
   `hazards.hpc_loss` with `on: omission` reads detection 30 ms (the lease)
   and closes its 70 ms budget with 0.00 ms of slack. Delete the `on:` line
   and detection reads 20 ms (the age limit) and the budget has 10 ms of
   slack. `detector_interval_ms` (`manifest_loader.rs:1113-1147`) takes the
   MIN over every mechanism of every wanted kind, so widening the set of kinds
   can only shorten the interval. A contract that names its fault class
   precisely is scored more harshly than one that declines to.
2. **`hazards.<h>.on` takes exactly one kind** (`Option<FaultKind>`, a list is
   a parse error), while `on_violation.on` takes a list. A guard watched by a
   lease AND an age limit must drop one.
3. **`hazard-unguarded` recommends detectors that cannot satisfy it**: its
   text lists `lease_duration, deadline, max_age or min_rate_hz`; for
   `on: omission` only the lease counts, and `min_rate_hz` is read by no
   branch under any kind. Measured: a contract that declares `max_age` fails
   with a message that recommends `max_age`. Issue #0046.
4. **A reaction that waits for a clock is invisible.** The island's
   `mrm_emergency_stop_operator` latches the `OperateMrm` request in its
   service callback and publishes the braking command from its 30 Hz timer
   (`mrm_emergency_stop_operator_core.cpp:96-135`). The service-edge walk
   (`c1474126`) correctly stops at a server with no `srv:`-triggered path,
   so up to 33 ms between "the handler decided" and "the vehicle was
   commanded" is charged to nothing. On a 70 ms L4 budget that is nearly half.

## The rule this phase sets

A fault-detection interval must cover EVERY fault class the hazard claims.
So `FDTI = max over the hazard's kinds of (min over that kind's mechanisms)`,
and a kind with no mechanism is unguarded FOR THAT KIND. Under that rule an
omitted `on:` means "all kinds" and is the strictest reading, so saying less
never buys slack; and a list is a strictly more informative declaration than
an omission rather than an unspellable one.

## W1 - `on:` is a set, and the interval is a max over it

- rlm: `HazardDecl.on` becomes a set (`on: omission` and `on: [omission, late]`
  both parse; the closed-set refusal is unchanged; `on: []` is a parse error,
  "state at least one fault class or omit the key"). Field table, format
  reference and the parse tests follow. Tag **v0.1.43**.
- play_launch: pin v0.1.43; `check_fault_reaction` computes the FDTI per
  kind and takes the max; `hazard-unguarded` fires per uncovered kind. The
  L4 fixture (`simple-autoware-safety-island/docs/demo-l4/l4_designed.*`)
  must still read `detection 30.00ms ... 0.00ms of slack`; the same file
  with `on:` deleted must read 30.00 ms, not 20.00 ms; `ladder-rung-budget`
  and phase 75's `autoware.rs` assertions (1944 / 56 ms) unchanged.

## W2 - the message names what counts for THIS hazard

`hazard-unguarded` lists the mechanisms that count for the hazard's declared
kinds, and separately names any detector the subscriber DOES declare that
this hazard cannot use ("`max_age: 20ms` is declared but counts only for
`on: late`"). `min_rate_hz` is never listed as a detector. Fixture: issue
#0046's run 4.

## W3 - an age limit detects an omission when something evaluates it

`on_violation.mechanism` (`qos | diagnostics | application`,
rlm `types.rs:174-184`) exists and the FDTI never consults it. `max_age`
counts toward `omission` if and only if the mechanism is `diagnostics` or
`application`, i.e. the node evaluates staleness on its own clock; under
`qos`, the default, only DDS liveliness and deadline fire. Matches phase 74's
`(mechanism: qos)` tag. Resolves #0046's open half; the issue moves to a
ruling.

## W4 - a sampling hop in the reaction walk

In `from_service` (and symmetrically for a `state:` subscriber read on a
tick): when the server has NO path triggered by the `srv:` endpoint but has a
timer path whose output leads on toward the sink, charge one period
(`1000 / rate_hz`, the quantity `derive/src/view.rs:133 sampling_cost_ms`
already defines) and continue. The route names the hop as a sampling hop so
the reader can see a clock in the chain. The L4 design accounts the same way
(section 9.2 charges I1 -> I16 as "2 cycles = 20 ms"). Fixture: the island's
own contract, where the route must now reach
`/mrm_emergency_stop_operator/on_timer` with `+ 33.33ms (sampling)`, and
criticality derives `reacts` for the operator.

## Order

W1 first, because W2's message depends on the per-kind result. W3 is a
one-line branch inside W1's function and lands with it. W4 is independent
and touches `walk_reaction`, not `detector_interval_ms`.

## Not in this phase

Whether `settle` should absorb any sampling delay: no. Settle is the plant's
term, the one software cannot derive; a tick is software latency the island
controls, and "shorten the reaction route" is the remedy that applies to it.
