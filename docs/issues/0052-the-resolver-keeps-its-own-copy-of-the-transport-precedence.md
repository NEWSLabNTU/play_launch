---
id: 52
title: "the resolver still computes the transport precedence itself, because the shared one is unreachable from where its graph is built"
status: open
type: tech-debt
severity: low
---

# 0052 - one derivation, two copies, now with a comment at each

**Repos:** `play_launch` at `a6506261`, `ros-launch-manifest` at `v0.1.42`
**Affects:** `src/ros-launch-resolve/resolve/src/ros/manifest_graph.rs:337`;
rlm `derive/src/view.rs` (`TopicView::transport_ms`)

## Symptom

Issue #0042 carried a subscriber's `max_transport` onto the model so the shared
`derive` crate could apply the precedence — subscriber first, topic as
fallback. The resolver still computes the same precedence itself, so the rule
exists in two places that must agree by discipline rather than by construction.

## Cause, which is why this was not closed with #0042

Two independent obstacles, both verified at `v0.1.42`:

1. **Visibility.** `TopicView` is `pub(crate)` in `derive`, and the crate
   re-exports only `parse_criticality_label`. The type is neither namable nor
   constructible from outside.
2. **Input shape.** `build_global_graph(index: &ManifestIndex)` runs at LOAD
   time from types-level declarations, while `TopicView::from_model` is built
   from a `SystemModel`. The resolver's graph exists before any model does, so
   even a public `TopicView` would have nothing to be built from at that site.

Deleting the resolver's copy therefore means deleting the resolver's graph,
which is a much larger change than #0042 was.

## Impact

Low today and bounded by a test: `tests/tests/endpoint_transport.rs` fails if
the two copies disagree, and it is built on a FORK so that the override decides
which branch is the critical path rather than only shifting a total. Both sites
now name the other and that test.

The risk is drift at the next edit — which is the shape this whole campaign
keeps meeting, and the reason phase 78 moved the derivation in the first place.

## Fix direction

One of, both in `ros-launch-manifest`:

- make `TopicView` public and have the resolver build one (which means giving
  the resolver a `SystemModel` at that point, or building the view from the
  index — the second may be simpler than it sounds, since the view's inputs are
  the topic's transport plus per-subscriber overrides); or
- extract a two-argument `transport_ms(sub_override, topic_default)` helper in
  rlm that both sides call. Smaller, uglier, and it does make the rule single.

Whichever, delete the resolver's copy in the same change. Two copies with
comments pointing at each other is a holding position, not a resting place.

## Provenance

Established 2026-09-24 while implementing #0042 wave B; both obstacles were
checked rather than assumed.
