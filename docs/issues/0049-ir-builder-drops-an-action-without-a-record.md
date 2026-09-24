---
id: 49
title: "the IR builder discards an unsupported action with a `debug!` and no `DroppedAction` — the shape `<timer>` had"
status: resolved
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

## Resolution 2026-09-25

The two traversals agree now. `ir_builder.rs`'s `other =>` arm calls
`note_dropped_action(other)` — the same call the evaluating path makes —
which keeps a log line and records a `DroppedAction` beside it.

**Where the entry goes.** `note_dropped_action` was already reachable: the IR
builder is an `impl LaunchTraverser`, so the entry lands on the traverser's
own `dropped_actions`, exactly as on the evaluating path. What the IR lacked
was an OUTPUT to carry it — `analyze_launch_file` discards the traverser and
returns a `LaunchProgram`. So `LaunchProgram` gained
`dropped_actions: Vec<DroppedAction>`, filled at every construction site in
`build_ir_file`. That is the smallest thing that gives the IR somewhere to put
it; no new type, and the entry is the one `check` already knows how to refuse
on.

Two holes found while wiring it, both the same defect in a second place:

- `build_ir_include` built a child `LaunchTraverser` and **never merged its
  `dropped_actions` back**, unlike `traverser::include` and
  `traverser::xml_include`, which both do. A drop inside an included file
  would have been recorded and then thrown away. It merges now.
- `evaluate_launch_file` builds the IR with one traverser and evaluates with a
  FRESH one, so a drop from the BUILD could not reach the record it produces —
  the IR's only record-producing entry point would have reported a truncated
  launch tree as clean, `check` exit 0, which is the whole failure. The
  entries are carried across before `evaluate_ir`.

Tests (`just test-ir`, **511 run, 511 passed**, up from 508):

- `ir_tests::test_ir_records_an_unimplemented_action` — `<timer>`, the
  motivating action, is still unimplemented on the IR path. Asserts BOTH that
  the nested node is gone from `all_nodes()` (so the test is not vacuous
  about there being a real loss) and that the drop is on the program.
- `ir_tests::test_ir_records_an_unimplemented_action_from_an_included_file` —
  the merge.
- `ir_eval_tests::test_evaluate_reports_an_action_the_ir_builder_dropped` —
  compares `evaluate_launch_file` against `parse_launch_file` on the same
  file, so the two paths are held to the same count.

Non-vacuity confirmed by mutation: reverting the one-line `other =>` arm to a
bare `debug!` fails all three (0 passed, 3 failed).

Left as-is: the standalone `<composable_node>` arm still drops with a bare
`debug!` — so does the evaluating path (`entity.rs:259`), so the two agree,
and it is not a subtree.
