---
id: 12
title: "Attribute allowlists are pinned to Humble's surface, and the differential oracle silently skips on any other distro"
status: open
type: bug
severity: medium
---

# 0012 — Allowlists pinned to Humble; the oracle skips instead of reporting drift

**Repo:** `play_launch_parser` (`src/ros-launch-resolve/parser`)
**Affects:** `src/xml/attr_spec.rs`, `tests/attr_differential.rs`
**Found:** whole-branch review of the `machine=` removal (2026-08-01)

## Summary

Two coupled problems, both about ROS distro assumptions.

**1. The tables encode Humble's exact attribute surface.** They were measured
against `/opt/ros/humble` and will *over-reject* on newer distros. Concretely,
upstream's own conformance fixture
(`launch_xml/test/launch_xml/executable.xml`) uses `sigkill_timeout` and
`sigterm_timeout` — valid on Jazzy and Rolling, absent from Humble, and
therefore absent from our `known_unsupported` lists. On Jazzy those become
hard `ParseError::UnexpectedAttribute` on launch files that stock `ros2
launch` accepts.

This is the same failure shape the final review caught for `on_exit` and
`node-name`, just triggered by distro rather than by incomplete measurement.

**2. The safety net cannot detect it.** `attr_differential.rs` hard-codes the
oracle:

```
source /opt/ros/humble/setup.bash || exit 42
```

Exit 42 is the "no ROS 2" sentinel, which the test treats as *skip*. So on a
Jazzy or Rolling machine the differential test does not report drift — it
reports nothing at all and passes. The one mechanism designed to catch table
drift is disabled exactly where drift is guaranteed.

## Why medium, not low

The tables now hard-error on unknown attributes. A user on a supported-by-ROS
distro other than Humble can hit a parse failure on a valid launch file, and
the project's own test suite will be green on their machine while it happens.

## Fix direction

- Source the *active* distro rather than hard-coding Humble: prefer
  `$ROS_DISTRO` / an already-sourced environment, and fall back to a search
  rather than a fixed path.
- Distinguish "no ROS 2 at all" (legitimate skip) from "a ROS 2 that is not
  the one the tables were measured against" (should run, and should report
  disagreements as findings). The three-way `Ros2Batch` enum added in the
  differential test's second fix round already models "skip vs. fail loud";
  this needs a third case for "ran against a different distro".
- Decide the policy question the tables imply: are the allowlists meant to be
  the *union* across supported distros (so `sigkill_timeout` goes in
  `known_unsupported` unconditionally), or per-distro? The union is simpler and
  fails safe — a distro that lacks an attribute will merely warn rather than
  error.

## Note

`docs/guide/` does not currently state which ROS distros the parser's strict
attribute checking is validated against. Whatever policy is chosen should be
written down there, since the failure is user-visible.
