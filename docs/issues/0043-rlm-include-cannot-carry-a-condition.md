---
id: 43
title: "rlm: an `include:` cannot carry a condition, though the docs said carrying conditions is what it is for"
status: open
type: correctness
severity: low
---

# 0043 - the conditional include is unrepresentable, and was documented as the point of the feature

**Repo:** `ros-launch-manifest` (rlm) at `v0.1.39`; filed here because rlm has
no tracker.
**Affects:** `types/src/types.rs:569-574` (`IncludeDecl`);
`types/src/cond.rs:45` (`filter_manifest`); `types/src/parse.rs`
(`parse_includes`)

## Symptom

Neither spelling of a conditional include parses:

```yaml
includes:
  perception:
    manifest: perception.yaml
    if: use_perception          # unknown key in `includes.<name>`
```

```yaml
includes:
  perception:
    if: use_perception          # an inline include is a nested manifest,
    nodes: { ... }              # which has no `if` key at its root
```

The first is rejected because the external form accepts only `manifest:`; the
second because the inline form IS a manifest and a manifest root has no
condition.

## Cause

`IncludeDecl` has nowhere to put one:

```rust
pub enum IncludeDecl {
    /// External: loaded from separate manifest file.
    External { manifest: String },
    /// Inline: embedded manifest (from <group> block).
    Inline(Box<Manifest>),
}
```

and `filter_manifest` (`cond.rs:45`) calls `should_include(...)` for fifteen
entity kinds — nodes, topics, services, actions, endpoints — and never for an
include. So even if the key parsed, nothing would act on it.

This was found by the doc test added in v0.1.38 once it was taught to see
indented fences (v0.1.39): two examples in the spec were parse errors, and
the prose around them said *"the include entry exists to carry per-child
conditions"* — a claim that was never true of any released version.

## Impact

Low in practice: a launch file's own conditions decide which nodes exist, and
a manifest describes what a scope declares rather than which branch ran. The
cost is that an author following the documentation wrote something that
silently did not parse before v0.1.38 (an unknown key was discarded without a
diagnostic until phase 69) and now fails with a message naming keys that do
not include the one the docs recommended.

## Fix direction

Decide which of the two it is, and say so in `docs/design-issues.md` rather
than leaving the question in the prose:

- **It should exist**: `IncludeDecl::External` gains `if`/`unless`, the inline
  form gains a wrapper that can carry them, and `filter_manifest` drops a
  whole include the way it drops a node — including the refs pointing into it,
  which is the part that needs care (the cleanup pass already infers optional
  refs from conditional nodes; an include's members would need the same).
- **It should not**: say that a manifest describes a scope's declarations and
  that selection belongs to the launch file, and keep the grammar as it is.

The documentation was corrected in v0.1.39 to describe what the grammar does,
so nothing is actively misleading while this is decided.

## Provenance

Found 2026-09-23 by the v0.1.39 documentation pass, after the doc test was
fixed to see indented fences; the type and the filter were read at that tag.
