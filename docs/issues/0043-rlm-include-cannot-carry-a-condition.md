---
id: 43
title: "rlm: an `include:` cannot carry a condition, though the docs said carrying conditions is what it is for"
status: resolved
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

## Ruling 2026-09-24 — implement it

The project owner settled it against this report's own lean:

> Our contract should reflect the launch file structure. If the launch file has
> a condition on any X, the contract should have one too.

The usage measurement above (one fixture, zero real contracts, a block the
consumer never reads) argued for retiring `includes:` rather than extending
it. The ruling overrides that, on better grounds: the contract's shape mirrors
the launch tree so the two can be read side by side, `IncludeDecl::Inline`
comes "from `<group>` block" by its own doc comment, and a `<group>` is
exactly what a launch author writes `if=` on. Low usage is then a statement
about adoption rather than about whether the feature belongs — and a feature
that cannot express the common case is one reason adoption stays low.

Being implemented as: conditions on both spellings, `filter_manifest` dropping
a whole include whose condition is false, and the existing dangling-ref
cleanup treating a dropped conditional include the way it already treats a
dropped conditional node. A root `if:` on a standalone manifest is refused,
naming the include entry as where a condition belongs.

Reasoning recorded as design issue #56 in the manifest repository.

## Resolved 2026-09-24 — rlm v0.1.41

`IncludeDecl` became a struct carrying `if_condition`/`unless_condition`
beside an `IncludeKind`, which is where every other declaration keeps them.
Conditions parse on both spellings — beside `manifest:` on the external form,
and at the nested manifest's root on the inline one — and a root condition on
a STANDALONE manifest is refused, naming the include entry as where it
belongs.

`filter_manifest` drops a whole include whose condition is false and reuses
the existing ref-cleanup mechanism: conditional include names join the
`conditional_nodes` set, and SURVIVING include names join the owner set. Both
halves matter — adding only the first silently drops every ref into an include
that survived its own `if: "true"`.

Found and closed on the way, a gap that predates the issue: **the filter never
recursed into an inline include**, so a node declaring `if: "false"` inside a
surviving group stayed, with its condition still set — the one surviving
entity in a filtered manifest that kept one. Invisible while the container
itself could not be filtered. Pinned by its negative: with the recursion
removed, the false node is still present.

The workspace Cargo version moved `0.1.4` → `0.1.5`, the API break the
changelog's own rule calls for. rlm tests 554 → 560, 0 failed; play_launch at
the new pin: lib 346, resolver 215.
