---
id: 40
title: "`--features ir` has not compiled on `main`, because no recipe and no CI job ever built it"
status: resolved
type: correctness
severity: medium
---

# 0040 - the IR suite was dead, and the way it died is the point

**Repo:** `play_launch` (0.11.0, `5eaa3191`; found at `e1142744`)
**Affects:** `src/ros-launch-resolve/parser/crates/play_launch_parser/src/traverser/ir_builder.rs:521`,
`.../traverser/ir_evaluator.rs:251`; `justfile` (no `--features ir` recipe);
`.github/workflows/` (no job); `CLAUDE.md` (names the command as runnable)

## Symptom

```
$ cargo test -p play_launch_parser --features ir
error[E0063]: missing field `dropped_actions` in initializer of `LaunchTraverser`
   --> parser/crates/play_launch_parser/src/traverser/ir_builder.rs:521:35
error[E0063]: missing field `dropped_actions` in initializer of `LaunchTraverser`
   --> parser/crates/play_launch_parser/src/traverser/ir_evaluator.rs:251:35
error: could not compile `play_launch_parser` (lib) due to 2 previous errors
```

Found while verifying the #0034 fix, and confirmed pre-existing by stashing
that fix and rebuilding. CLAUDE.md lists
`cargo test -p play_launch_parser --features ir` under Testing as a runnable
suite ("42 tests, not included in default").

## Cause

`dropped_actions` was added to `LaunchTraverser` by the `<timer>` work
(`72a547e9`, "a lazily-evaluated timer period can silently drop its children").
Three of the five places that construct a child traverser by struct literal
were updated - `include.rs:316`, `xml_include.rs:121`, and the field's own
initializer at `lib.rs:114`. The two behind `--features ir` were not, and no
default build compiles them.

The `ir_evaluator.rs` half is the sharper half: it WAS given the merge loop
that reads the new field,

```rust
for dropped in std::mem::take(&mut child_traverser.dropped_actions) {
    self.note_dropped(dropped);
}
```

at `:271`, while the initializer forty lines earlier was never given the field.
A half-finished edit, in a file that no build in this repository compiles.

Nothing ran it: `grep -n "features ir" justfile .github/workflows/*.yml`
returned nothing. The suite was reachable only by a human typing the command
out of CLAUDE.md.

## Impact

43 tests (the IR suite is 43 now, not the 42 CLAUDE.md records) had not run for
however long, and the feature they cover could not be built at all - so the IR
layer was not "optional", it was broken, and the repository's own documentation
pointed at a command that fails. The failure is the quiet kind this tree keeps
meeting: green everywhere anyone looks, because the thing that would have gone
red is never built. Same family as the stale-submodule misdiagnosis and #0020
(a pip install shadowing the build).

## Resolution 2026-09-21

`dropped_actions: Vec::new()` added to both initializers - the IR BUILD path
records no drops today (its unknown-action branch at `ir_builder.rs:438` only
`log::debug!`s, where the evaluating path calls `note_dropped_action`), so the
builder's child is correctly empty, and the evaluator's merge loop now has the
field it already read.

The compile fix alone would have left the hole open, so the gate came with it:
`just test-ir` runs the suite, and `just test-all` calls it beside the C++
unit tests. `just test-ir`: **501 tests run, 501 passed** (458 default + 43 IR).

Left alone deliberately, and worth a separate decision: the IR builder's
unknown-action branch discards an action with a `debug!` and no
`DroppedAction` entry, which is exactly the shape `<timer>` had before
`72a547e9` - a dropped subtree with no record of it. The IR path does not
produce records today, so nothing consumes the entry yet; it should still
note one.
