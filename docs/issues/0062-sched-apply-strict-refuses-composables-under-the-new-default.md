---
id: 62
title: "`--sched-apply strict` refuses every launch whose composables carry a tier, since `observable` became the default"
status: resolved
type: defect
severity: medium
---

# 0062 - `--sched-apply strict` refuses composables under the new default

**Repo:** `play_launch` at `785cddb7`
**Affects:** `src/play_launch/src/execution/sched_plan.rs:548`
(`composable_tiers_not_applied`), `src/play_launch/src/commands/up.rs:843-860`
**Found:** review of the commits pulled 2026-10-11, confirmed by reading.

## What happens

`play_launch launch <pkg> <file> --sched-apply strict`, with no
`--container-mode`, now exits non-zero before spawning anything when a
composable node gets a scheduling tier (derived or from `overrides:`):

```
--sched-apply strict: N composable tier(s) cannot be applied under --container-mode observable: ...
```

The same command started and applied every tier on 0.14.

## Why

`composable_tiers_not_applied` returns early only for `ContainerMode::Isolated`,
where every composable is its own process and so every tier reaches a pid. In
any other mode it lists each composable that has a tier, and strict refuses
at the start boundary. That was right while `isolated` was the default: asking
for `observable` was a deliberate choice, and refusing told the user what it
cost.

615d0de2 made `observable` the default. A user who never chose a container
mode now gets the refusal. The commit message, the changelog and CLAUDE.md
don't mention it. The tests were changed to pass `--container-mode isolated`
explicitly, which is the only reason the suite stayed green.

The refusal itself is sound. Several composables in one process can't hold
different per-composable priorities. What's missing is a decision about the
interaction.

## Options

1. Under strict, when the user did not choose a mode and some composable
   carries a tier, pick `isolated` automatically and say so in one `info!`
   line. Strict means "deliver the plan or don't start", and isolated is the
   mode that can deliver it.
2. Keep the refusal, but name the fix in it (`--container-mode isolated`), and
   document the interaction in `docs/guide/rt-scheduling.md` and the 0.15
   changelog.
3. Apply the container's tier to the whole container process under
   `observable` and warn per composable. That changes what strict promises, so
   it needs its own design note.

Option 2 is the minimum. Option 1 is what a user running strict most likely
wants.

## Verification, when fixed

A `sched_apply` integration test with no `--container-mode` and a composable
tier, under `strict`: it must either start with every tier applied (option 1),
or refuse with a message naming `--container-mode isolated` (option 2).

## Resolution (2026-10-11)

Option 1. `up.rs::effective_container_mode`: under `--sched-apply strict`,
with no `--container-mode` on the command line and a composable that the plan
gives a tier, the run uses `isolated` and logs one `info!` saying why. An
explicit `--container-mode` is never overridden, so strict still refuses
under `--container-mode observable` written out
(`options::container_mode_given_explicitly`, sharing one `given_explicitly`
with `--enforce-rules`). The plan is now computed before the container
contexts are built, since the mode decides how they are built.

Tests (`sched_apply.rs`): `composable_tier_strict_without_a_mode_uses_isolated`
and `composable_tier_strict_with_explicit_observable_still_refuses`. Negative
control: with the switch disabled, the first fails (strict refuses).
