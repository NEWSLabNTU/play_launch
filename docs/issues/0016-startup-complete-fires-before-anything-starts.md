---
id: 16
title: "Startup complete fires before anything has started, and silences all further progress"
status: resolved
type: correctness
severity: medium
---

# 0016 — `Startup complete` could fire before anything had started

**Repo:** `play_launch`
**Affects:** `commands/signal_handler.rs`, `member_actor/model/member_view.rs`
**Status:** fixed in phase-61 (`HealthSummary::startup_complete()`).
**Severity:** reporting only — nothing spawned differently, but the operator was
told the opposite of the truth and then received no further progress at all.

## Symptom

Two tenths of a second into a three-minute startup:

    INFO Startup complete: all nodes ready (nodes 12/44, containers 0/16, composable 0/84)

and then silence — no `Startup:` progress lines for the rest of the run.

## Cause

`signal_handler.rs` tested completion as:

```rust
let startup_complete = health.composable_pending == 0;
```

A composable whose container has not started yet is `Blocked`, which counts as
neither pending nor loaded nor failed. So `composable_pending == 0` is true at
t=0, stays true until the first container is up, and the completion check runs
every 100 ms.

The progress task exits once it reports completion, which is why the run then
goes quiet: the message is not merely wrong, it suppresses everything that
would have corrected it.

## Why it was not seen before

A race that used to be won reliably. Every container spawned within the first
100 ms, so by the first completion check there were already pending loads.
Phase 61 put a gate in front of process spawn; the first paced run made
containers queue, the race started losing every time, and the bug surfaced
immediately.

It does not need the gate to bite, though: any container slow enough to miss
the first 100 ms window reproduces it. A cold TensorRT engine build would do it.

## Fix

`HealthSummary::startup_complete()` requires every member to have reached a
settled state — nodes and containers running/stopped/failed, composables
loaded/failed — rather than inferring completion from the absence of one
intermediate state.

## Lesson

The test asked "is nothing in flight?" when it meant "is everything done?".
Those differ exactly at the start, which is the one moment a startup detector is
guaranteed to be evaluated.
