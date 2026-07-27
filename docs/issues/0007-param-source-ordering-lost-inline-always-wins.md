---
id: 7
title: "Parameter source ordering is lost — inline <param> always wins over a later <param from=>, diverging from ROS 2"
status: open
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

## Recommendation

**A**, with the `params`/`params_files` accessors retained as derived views so
neither consumer breaks mid-migration. The fix belongs at the fork in
`node.rs` — everything downstream is just carrying the mistake forward. B and
C both leave a shape that cannot represent what ROS 2 means; D documents the
bug instead of fixing it.

Worth pairing with a regression fixture: a launch with
`<param name="a" value="1"/>` followed by `<param from="a_is_2.yaml"/>`,
asserting the resolved value is 2 on both the spawn path and the model.

## Cross-reference

nano-ros issue #276 (archived) — "Fidelity audit vs standard ROS (2026-07-27)"
records the same divergence from the consumer side and notes that exact
fidelity needs an ordered parameter-source list in the model, i.e. this issue.
