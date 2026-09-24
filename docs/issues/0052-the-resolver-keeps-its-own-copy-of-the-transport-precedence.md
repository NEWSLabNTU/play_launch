---
id: 52
title: "the resolver still computes the transport precedence itself, because the shared one is unreachable from where its graph is built"
status: resolved
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

## Resolved 2026-09-25 — rlm v0.1.44, shape (a)

`TopicView` and its fields are `pub` and re-exported at rlm's crate root; the
resolver builds one from its `ManifestIndex` and calls `transport_ms` per
edge. The inline precedence expression is gone from `manifest_graph.rs`, and
so is the fourteen-line "SECOND COPY, deliberately" comment that named the
other site — `grep -rn 'SECOND COPY'` now returns nothing.

**The second obstacle dissolved rather than being worked around.** Both held
when checked: the type was `pub(crate)`, and `build_global_graph` does run at
load time with no `SystemModel`. But the obstacle belonged to
`ModelView::from_model`, not to `TopicView` — the view is four declared facts
(publishers, subscribers, the topic's transport, the per-subscriber
overrides) and holds no model reference, so a consumer with only a
`ManifestIndex` can build one. The resolver never gets a model.

Shape (b) — a two-argument `transport_ms(sub, topic)` helper — was rejected
because it makes the rule single while leaving the view private, so design
issue #55's step 2 (a per-edge transport class derived from placement) would
have to be plumbed as a second argument list rather than as a field on the
shared type, and the resolver would keep assembling the inputs itself, which
is where the precedence was hiding in the first place.

One consequence recorded for whoever touches the gate next:
`tests/tests/endpoint_transport.rs` asserts BOTH that the two consumers agree
on a route and what that route absolutely is. Now that they call one function,
the equality half can no longer catch a wrong precedence — both sides move
together. What caught the sabotage (`transport_ms` returning the topic value
unconditionally) was the absolute `EXPECTED_ROUTE`/`EXPECTED_TOTAL`
constants, which flipped to the `producer → fast → sink = 45.00ms` branch the
fixture documents. Those constants are the live half of that test and must not
be traded for the equality check.

rlm 566 passed; resolver 226 passed; `endpoint_transport` 2 passed at the
v0.1.44 pin.
