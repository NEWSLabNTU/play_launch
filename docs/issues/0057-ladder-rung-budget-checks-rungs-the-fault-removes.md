---
id: 57
title: "`ladder-rung-budget` checks, and charges, rungs the hazard's own fault makes unavailable"
status: resolved
type: correctness
severity: medium
---

# 0057 - one ladder for two faults failed on the rung neither could reach

**Repo:** `play_launch` at `22b6baf3`
**Affects:** `src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs`,
`check_fault_reaction` (the `rungs.iter().take(len - 1)` loop) and
`resolve_ladder` (the floor test)

## Symptom

The Autoware Safety Island's takeover contract (phase 8, brief D section 1.6)
declares two hazards on one input, `odd_exit` (the availability gate clears
`autonomous`, `on: reported`) and `hpc_loss` (the stream stops,
`on: omission`), and one ladder for both: takeover request, comfortable stop
(the HPC's planner brakes; `requires: [hpc_alive]`), emergency stop. The
checker judged every non-floor rung against every hazard's interval, so
`hpc_loss` was charged the comfortable stop's 9996.67 ms settle against its
10 s interval -- a rung that requires the very function the HPC loss removes.
Once windows are charged cumulatively the same loop would also charge
`hpc_loss` the 10 s takeover window it can never enter.

## Cause

Rung selection did not know fault classes. `ladder-unterminated` asked the
one question that matters -- can this fault take the rung away -- for the
FLOOR only, and only by name (a guard topic or a guard function named in
`requires:`).

## Fix (phase 83)

`value_rules::removed_by(hazard)`: the functions the fault removes in its
scope. A function is removed when one of its topics is guarded by the hazard
(or the hazard guards it by name) and the fault's classes reach its kind: a
`reported` fault removes the VALUE functions on its topic (rlm v0.1.46
`of:` + `when:`), an omission, late or loss fault removes both kinds, because
silence also means "not known to be in range". A rung that requires a removed
function is skipped -- not checked, and its window not charged -- and listed
as skipped in `check --explain`. `resolve_ladder`'s floor test uses the same
set.

Verified on `tests/fixtures/contract_takeover` (`hpc_loss` at 3.0 m/s fits
3 s at 2676.67 ms; the comfortable rung it skips would have read 5576.67 ms)
and on the island's demo contract (4808.67 ms against 10 s).

Two existing fixtures change with it. `contract_modes_bad`'s `degraded` rung
required `perception`, which its own hazard removes; it now requires nothing,
so `ladder-rung-budget` still has a reachable rung to fail on. And the
Autoware fixture's `comfortable_stop` -- phase 75's "cannot make 2 s" -- is
skipped: it requires `operation_mode_availability`, which `mode_unavailable`
removes, and `mrm_handler` forces EMERGENCY_STOP on that timeout anyway.
