---
id: 11
title: "Three dispatched launch elements have no AttrSpec, so their attributes are silently unvalidated"
status: open
type: bug
severity: low
---

# 0011 — Dispatched elements with no `AttrSpec` fail open

**Repo:** `play_launch_parser` (`src/ros-launch-resolve/parser`)
**Affects:** `src/xml/attr_spec.rs`, `src/traverser/entity.rs`
**Found:** whole-branch review of the `machine=` removal (2026-08-01)

## Summary

`spec_for()` returns `None` for elements with no table entry, and
`validate_attrs`/`validate_yaml_keys` treat `None` as "skip validation". That
is deliberate for `<launch>` — measured: ROS 2 does not validate the root
element, `<launch zzz="1">` parses fine.

But three elements the traverser actually **dispatches** also have no spec, so
they inherit the skip by accident rather than by decision:

- `declare_argument` (`traverser/entity.rs`)
- `unset_env` / `unset-env` (both frontends)
- `pop-ros-namespace`

Unknown attributes on these are accepted silently — the pre-existing behavior
the allowlist work set out to remove.

## Why it slipped through

The design's decision #7 was "the strict check covers every element the parser
handles", and the spec's out-of-scope list justified skipping `unset_env` on
the premise that the parser "does not dispatch it at all". It does — in both
frontends. So the stated scope was not actually met, and nothing failed to
signal that, because failing open produces no error.

## Fix direction

Add specs for the three, measured against the ROS 2 oracle the same way the
existing tables were (inject one candidate attribute at a time; see
`tests/attr_differential.rs`). From `launch`'s frontend sources these should
be roughly:

- `unset_env`: `name`, plus `if`/`unless`
- `pop-ros-namespace`: `if`/`unless` only
- `declare_argument`: check whether this is this parser's own alias for `arg`;
  if so it should share `arg`'s spec rather than get a new one

Then extend `attr_differential.rs`'s `FIXTURES` so the oracle covers them —
otherwise the new tables have the same blind spot the `include-arg` /
`executable-arg` specs originally had.

Consider also making the fail-open explicit: a spec table entry meaning "no
attributes beyond conditions" reads differently from an absent entry, and
today they are indistinguishable.
