---
id: 103
title: "A lost LoadNode reply deferred to ComponentEvent was never bounded"
status: resolved
type: defect
severity: medium
---

# 0103 - A lost LoadNode reply was not bounded by the load budget

**Repo:** `play_launch` 0.35.1 to 0.41.1
**Affects:** the container actor's lost-load sweep
(`container_actor/component_events.rs::rescue_lost_loads`)
**Found:** swb 139 via swb 133 (scensim issue 088), 2026-11-21, on an
Autoware start that hung; reduced to a three-composable repro that stops
the container with SIGSTOP after it is up

## What happened

Under `--container-mode observable` (and `isolated` without the control
socket), a LoadNode call that times out on `play_launch_container` is not
retried: the container is authoritative through `ComponentEvent`, so
`ros_client` logs "LoadNode response lost ... deferring to ComponentEvent"
and leaves the composable `Loading` with no id. The lost-load sweep picks
it up once `load_total_budget_secs` (default 600 s) has passed and asks
ListNodes. When the container cannot answer that either (stopped,
deadlocked, or a constructor holding its executor), the answer is
`Unavailable`, and the sweep logged at debug and tried again next tick,
forever. The report waited 12 minutes at "0/3 loaded (3 pending)" with no
`LOAD_FAILED` and no `STARTUP_SUMMARY`, although the run printed
`busy_budget=600s`.

The direct path, used for a stock container, already ends the same case
with `LoadError::Unresponsive` at the budget.

## Fix

The sweep's `Unavailable` branch now fails the composable with that same
error (`container-busy: unresponsive for <budget>s while loading <name>`)
and emits `LoadFailed`, so startup completes and names it. No resend: a
LoadNode the container may still be running is not safe to repeat.

The `Absent` branch is unchanged: for our own container ListNodes hides a
composable still constructing under `isolated`, so absence is ambiguous
there and the sweep still only reports it.

A late `ComponentEvent LOADED` for a composable failed this way is ignored,
as it is for every failed entry; a container that answers after the budget
has run out has to be restarted to bring the composable back. Raise
`load_total_budget_secs` for constructors longer than ten minutes.

## Test

`tests/tests/load_policy.rs::test_a_lost_reply_from_a_busy_container_fails_at_the_budget`
runs a 60 s constructor under `observable` with a 2 s LoadNode timeout and
an 8 s budget: the call is deferred, the constructor holds ListNodes, and
the run must end with `container-busy: unresponsive for 8s` and a
`STARTUP_SUMMARY`. Without the fix it fails at 40 s with no summary.
