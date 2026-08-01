---
id: 7
title: "Parameter source ordering is lost — inline <param> always wins over a later <param from=>, diverging from ROS 2"
status: resolved
type: bug
severity: medium
---

## Summary

ROS 2 treats a node's parameter sources as ONE ORDERED list. In `launch_ros`,
`parameters=[{'a': 1}, 'later.yaml']` applies the dict first and the file
second, so `later.yaml` **wins** for `a`. The XML form is the same: sibling
`<param name= value=/>` and `<param from=/>` children apply in document order.

play_launch splits the two kinds into separate collections at parse time and
re-joins them with a fixed precedence — every `--params-file` first, inline
values last — so **inline always wins**, regardless of the order the launch
file wrote them. A launch file that deliberately places a file after an inline
default gets the opposite of the upstream result, silently.

## Where the ordering is lost

`src/play_launch_parser/.../actions/node.rs:71` walks `entity.children()` in
document order (correct), then immediately forks:

```rust
"param" => {
    if let Some(from_attr) = child.optional_attr_str("from")? {
        param_files.push(parse_substitutions(&from_attr)?);   // ← vec A
    } else {
        parameters.push(Parameter::from_entity(&child)?);      // ← vec B
    }
}
```

Two vectors, no interleaving index. Everything downstream inherits the loss:

- `NodeRecord { params, params_files }` (record/types.rs:278) — two fields.
- `SystemModel`'s `NodeInstance { params, params_files }` — same split.
- `execution/node_cmdline.rs:415` writes all inline params into a single
  `overrides.yaml` and renders it as the LAST `--params-file`, explicitly "so
  it overrides all earlier files". That is the correct behavior for the common
  case (inline written after files) and wrong whenever the launch ordered a
  file last.

## Impact

- **Linux spawn (play_launch):** wrong final value for any key set by both an
  inline `<param>` and a later `<param from=>`. Silent — no warning that a
  file's value was discarded.
- **nano-ros compile-time bake:** shares the divergence by construction, since
  it consumes the same split model (`NodeInstance::resolved_params` applies
  files then inline). Recorded in nano-ros issue #276's fidelity audit; the two
  engines agree with each other and differ from upstream together.
- **Porting hazard:** upstream Autoware-style configs that layer
  `default → override.yaml` in one `parameters=[...]` list produce different
  runtime values under play_launch than under `ros2 launch`, which is exactly
  the class of drift the SystemModel exists to prevent.

## Verified against ROS 2 (humble source, 2026-07-27)

Read `launch_ros/actions/node.py` rather than arguing from memory. Three facts
settle the design:

1. **`parse_nested_parameters` (line 276)** walks the `<param>` children in
   document order and appends into ONE `normalized_params` list — a
   `ParameterFile` for `from=`, a dict for `name=`. One element per `<param>`,
   order preserved, no grouping of consecutive inline params.
2. **`execute` (line 443)** walks that single list in order and materializes a
   DICT INTO A TEMP FILE:

   ```python
   if isinstance(params, dict):
       param_argument = self._create_params_file_from_dict(params)
       is_file = True
   ...
   cmd_extension = ['--params-file' if is_file else '-p', param_argument]
   ```

   So an inline dict is **not a precedence class** — it becomes a params file
   and takes its position in the same `--params-file` sequence. "Inline wins"
   has no basis in ROS; position is the only rule.
3. **Global params come first** (`params_container` loop, line 429), then the
   node's `parameters` entries; rcl applies `--params-file` / `-p`
   left-to-right, later wins.

The required model is therefore literally an ordered sequence of sources.

## Fix options

### A. Ordered source list in the record + model (correct, invasive)

Replace the two fields with one ordered vector:

```rust
enum ParamSource {
    Inline { name: String, value: String },
    File(String),          // resolved path or verbatim content
}
pub param_sources: Vec<ParamSource>,
```

- Parser: push into ONE vec inside the existing document-order walk — the fork
  disappears, the fix is at the site of the loss.
- `node_cmdline`: emit `--params-file` / inline-overrides chunks in list order.
  Consecutive inline runs still coalesce into one temp YAML; a file between two
  inline runs simply splits them into two chunks.
- Model: `NodeInstance.param_sources`, with `params`/`params_files` kept as
  DERIVED serde-skipped views for back-compat during migration.
- nano-ros: `resolved_params` folds the list in order — strictly simpler than
  today's two-phase merge.
- Cost: schema change in the shared `model` crate (both consumers vendor it),
  plus record-format change. Migration: emit both shapes for one release,
  readers prefer `param_sources` when present.

### B. Order-index sidecar (cheap, ugly)

Keep both vectors, add `params_order: Vec<(SourceKind, usize)>` recording the
interleaving. Consumers that care read the index; consumers that don't keep
working unchanged. Avoids touching every field but leaves the model carrying
a denormalized ordering it can silently get out of sync with — the sort of
mirrored-state drift that already bit us elsewhere.

### C. Preserve order only in the record; flatten at resolve (partial)

Fix the parser + `node_cmdline` (Linux spawn correct), and have the resolver
flatten to the final effective values so the model carries a single
already-merged `params` map with no `params_files`. Embedded bakes get correct
values for free and lose nothing (they cannot pass files to rcl anyway).
Downside: the model stops carrying param FILES verbatim, which
`--params-file`-based replay and the "model is the complete record of the
launch" principle both want.

### D. Warn-only (not a fix)

Detect an inline `<param>` preceding a `<param from=>` that sets the same key
and emit a checker warning. Cheap, honest, but leaves the wrong value in
place.

## A vs B, decided

| | A — ordered `param_sources` | B — two vecs + order index |
|---|---|---|
| Matches ROS's model | isomorphic; order is intrinsic | only if the index is right — order lives outside the data it orders |
| Illegal states | unrepresentable | index desync, stale/missing entries |
| **Existing readers** | compat accessor returns the EFFECTIVE merge → naive consumers become correct for free | `params`/`params_files` keep today's meaning → every reader stays **wrong by default** |
| Spawn path | walk list, one `--params-file`/`-p` per element — 1:1 with ROS's loop | reconstruct interleaving first; absent/stale index silently degrades to today's bug |
| Model as reviewable artifact | sources appear in the order they apply | two lists + `[(kind, idx)…]`; precedence requires mentally zipping |

B's only advantage — not touching existing fields — IS its defect: the wrong
shape survives, so the fix's reach depends on finding every consumer. A
removes the shape. This repo has also been bitten repeatedly by exactly B's
hazard (derived state mirrored beside its source and drifting).

**Decision: A**, with three details that make it match ROS exactly:

1. **Global params occupy the head of the list**, not a separate field —
   mirroring `params_container` running before `parameters`.
2. **One element per source.** Document in the type that a dict is a FILE in
   ROS's execution model, so a bake-time consumer folds it with file semantics
   (`ros__parameters` sections, wildcards), not as a bare key/value overlay.
3. **Fold order:** within a file, rcl's section precedence (`/**` then more
   specific); across sources, later wins. A fixes the second — the first must
   not regress.

Regression fixture: a launch with `<param name="a" value="1"/>` followed by
`<param from="a_is_2.yaml"/>`, asserting the resolved value is 2 on both the
spawn path and the model.

## Related gap (not fixed by A)

`ParameterFile(allow_substs=…)` — ROS re-resolves substitutions INSIDE a param
file when asked. Our model carries file content verbatim, so an
`allow_substs="true"` file needs its substitutions resolved at resolve time.
Record here so it is not discovered later as a second divergence.

## Resolution (2026-07-27, INCOMPLETE — see 2026-08-02 below)

Fixed by phase 54 along option A: one ordered `param_sources` list, carried
parser → record → model → spawn. `params` / `params_files` remain as legacy
views; a consumer that sees a non-empty ordered list must use it alone.

## Reopened and completed (2026-08-02)

The phase-54 claim above was not true end to end. The parser built the list;
it was then discarded **twice** on the way out, so the shipped behaviour was
still the original bug. Verified by launching the issue's own fixture and
reading the spawned command line:

```
--params-file .../0.yaml          -> a: 2   (the FILE, emitted FIRST)
--params-file .../overrides.yaml  -> a: 1   (the INLINE, emitted LAST)
```

rcl applies `--params-file` in order and later wins, so the node received
`a = 1` — the inline value — although the launch file wrote the inline param
FIRST and the file SECOND. Exactly the divergence this issue describes.

Three drops, each silent:

1. **`model_builder` hard-coded `param_sources: Vec::new()`** at all three
   `NodeInstance` sites. The comment claimed an empty list tells consumers to
   use the legacy views — but the model's own field doc says a non-empty list
   is authoritative, so the ordering was produced and thrown away.
2. **`NodeCommandLine::from_node_record` computed the ordered chunk files,
   WROTE them to disk, then hard-coded `Vec::new()` into its own return
   struct.** The files existed in `play_log` and were never referenced.
   (`rustc` had been emitting `unused variable: ordered_params_files` for
   this; the warning was correct.)
3. **The YAML frontend never built the list at all** — it constructed an
   empty one, so YAML launch files were unaffected by phase-54 entirely.

All three fixed. The same fixture now spawns:

```
--params-file .../ordered-0-inline.yaml   -> a: 1
--params-file .../ordered-1-file.yaml     -> a: 2
```

document order, so `a` resolves to **2** as ROS 2 requires — on both
frontends.

Regression coverage: `tests/tests/param_ordering.rs` asserts the model's
`param_sources` is present, two entries, inline-then-file, with the winning
value in the file — for XML and YAML. Fixtures under
`tests/fixtures/simple_test/launch/param_ordering.*`.

Commits: parser `f1cad5b`, ros-launch-resolve `089abce`, play_launch (below).

**Still not fixed — composable nodes.** `ComposableNodeRecord` carries no
`param_sources`; the parser never builds one for that path
(`actions/container.rs` constructs them with `Vec::new()`). A composable node
whose `<param from=>` follows an inline `<param>` still gets the legacy
files-then-inline order. The `model_builder` site now says so explicitly
rather than implying coverage. Fixing it needs the ordered list built in
`ComposableNodeAction` and carried through `ComposableNodeRecord` first.

## Implementation

docs/roadmap/phase-54-param-source-ordering.md

## Cross-reference

nano-ros issue #276 (archived) — "Fidelity audit vs standard ROS (2026-07-27)"
records the same divergence from the consumer side and notes that exact
fidelity needs an ordered parameter-source list in the model, i.e. this issue.
