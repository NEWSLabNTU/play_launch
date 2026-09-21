---
id: 35
title: "a composable's derived priority is silently not applied under `--container-mode observable|stock`"
status: open
type: correctness
severity: medium
---

# 0035 - the plan promises a per-node priority the container mode cannot deliver, and nobody says so

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affects:** `src/play_launch/src/member_actor/container_actor/component_events.rs:182-190`;
`src/play_launch/src/execution/sched_plan.rs:372-377`, `:408-460`;
`src/play_launch/src/commands/up.rs:706-721`; `src/play_launch/src/member_actor/container_actor/actor.rs:327-345`

## Symptom

`play_launch resolve --sched <platform>` (or `launch --sched`) derives one
`SCHED_FIFO` tier per node with a rate or deadline fact, composables
included, and `--explain` prints them. Under `--container-mode stock` (the
mode the runtime captures in brief C had to use) or `observable`, only the
container process is scheduled; the composables inside it run at the
container's priority. No line in the terminal or in `play_launch.log`
records that node X's tier was skipped, and the model's
`execution.bindings` still says it has one.

## Cause

`component_events.rs:182-190`, on the composable's LOADED event:

```rust
// Phase 38.9: apply resolved Linux scheduling to this composable's
// own process, now that the load report carries its pid. Observable/
// stock mode and failure reports carry pid 0, hence the guard.
...
if config.sched_mode != SchedApplyMode::Off
    && pid > 0
    && let Some(tier) = sched_tier.as_ref()
```

The guard is correct (there is no PID to schedule; under `observable` and
`stock` a composable is a thread pool inside the container, under
`isolated` it is fork+exec'd and gets its own PID) but it is silent: when
`pid == 0` and a tier exists the code falls through with no `warn!`, no
`plan.warnings` entry, nothing.

There IS a warning nearby, and it covers a different, narrower case.
`chain_container_colocation_warnings` (`sched_plan.rs:408-460`, called from
`up.rs:714`) warns when two or more CHAIN-MEMBER composables share a
non-isolated container (`:451`, `members.len() >= 2`). It says nothing when:

- a single tiered composable lives in a container (the common case: one
  container per node group), or
- the mapper is `rate_monotonic` / `deadline_monotonic`, whose plans carry
  no `chain_member_nodes` at all, or
- the plan came from a `SystemModel` (`from_model` leaves
  `chain_member_nodes` empty by its own doc comment at `sched_plan.rs:44-50`).

The container's own tier is applied at `actor.rs:327-345`, so the container
does get a priority; the composables' derived priorities are the ones that
vanish.

## Impact

The scheduling story is "derive the priority from the contract, apply it
per process". Under two of the three container modes the derivation
happens, is shown to the user, is written into the model, and is then
dropped for every composable. A user reading `--explain` and then
`ps -eLo tid,cls,rtprio` sees a contradiction with no message between
them. Autoware runs almost everything as composables.

## Fix direction

- In the LOADED handler, when `sched_tier.is_some() && pid == 0`, log at
  `warn` once per composable: "composable 'X' has tier 'T' (prio P) but
  runs inside container 'C' under --container-mode stock; not applied.
  Use --container-mode isolated to schedule it." Under `--sched-apply
  strict` make it an error at the start boundary, the way a missing
  capability already is (`up.rs:728-750`).
- Better, decide it before spawning: in `up`, after both the plan and
  `container_contexts` exist (where `chain_colocation_warnings_for_plan`
  already runs), walk every composable that has a tier and warn/refuse
  per container mode. That covers the single-composable and non-chain
  cases the co-location warning misses, and is mode-aware in the one place
  that knows the mode.
- Record the outcome in the model's execution layer (or `run_info.json`)
  so a later `measure` does not attribute a priority that was never set.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, section 4.1 step 4), 2026-09-18;
re-verified against the 0.11.0 source at `5eaa3191` on 2026-09-21.

## Update 2026-09-21 - the co-location warning cannot fire at all

Stronger than "covers a narrower case". `up.rs:693-704` builds the plan with
`SchedPlan::from_model` and, since 47.B3, that is the ONLY plan source `up`
has ("there is no legacy record-only replay path left to consult them from").
`sched_plan.rs:44-51` documents `chain_member_nodes` as "Empty for
`SchedPlan::from_model`". `chain_colocation_warnings_for_plan` at `:714`
therefore walks an empty set on every user path, so no composable co-location
warning has ever reached a user - the narrow case is not narrow, it is
unreachable.

The comment directly above that call (`up.rs:705-711`) asserts the opposite:
"both carry their own `chain_member_nodes` directly now - no ManifestIndex
re-parse fallback needed on either path". Two comments in one call chain
contradicting each other is why an always-silent warning read as a working
one. Planned as phase 80
(`docs/roadmap/phase-80-the-plan-and-what-was-applied.md`).
