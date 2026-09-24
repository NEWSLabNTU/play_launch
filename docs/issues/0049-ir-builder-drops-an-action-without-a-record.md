---
id: 49
title: "the IR builder discards an unsupported action with a `debug!` and no `DroppedAction` — the shape `<timer>` had"
status: open
type: correctness
severity: low
---

# 0049 - a subtree can vanish from the IR with nothing to show for it

**Repo:** `play_launch` at `a6506261`
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/traverser/ir_builder.rs:438-440`

## Symptom

```rust
other => {
    log::debug!("Skipping unsupported action type in IR builder: {}", other);
}
```

An action the IR builder does not implement is discarded, along with
everything nested under it, leaving a `debug!` that no default `RUST_LOG`
shows.

The evaluating path does not do this. `traverser/delay.rs:93`'s
`note_dropped_action` records a `DroppedAction` on the record so `check` can
refuse on it, under a comment that names the reason: *"a warning nobody reads
is how `<timer>` dropped whole subtrees while `check` exited 0."*

## Cause

The IR path was written before `DroppedAction` existed (it landed with the
`<timer>` work, `72a547e9`) and was not revisited, because nothing builds it:
the `ir` feature is off by default and had not compiled at all for weeks
(issue #0040). The two paths have disagreed about what a dropped action costs
ever since.

## Impact

Bounded today and only today: the IR produces no records, so nothing downstream
consumes a drop from it, and the feature is dormant with no consumer (see
CLAUDE.md's Launch IR section). It matters if the IR is ever given one — a
static analyser over the IR would silently analyse a launch tree with branches
missing, which is exactly the failure `DroppedAction` was introduced to stop.

## Fix direction

Call `note_dropped_action` (or record the equivalent on whatever the IR's
output type is) instead of the bare `debug!`, so the two traversals agree on
what a dropped action costs. Keep the log line — a `RUST_LOG` reader expects
it — and add the entry beside it, which is what the evaluating path does.

A test in the IR suite asserting that building a tree containing an
unimplemented action records it; `just test-ir` is what runs that suite.

## Provenance

Found 2026-09-21 while fixing #0040 (the IR feature had stopped compiling);
recorded there as a deliberate non-fix and filed now.
