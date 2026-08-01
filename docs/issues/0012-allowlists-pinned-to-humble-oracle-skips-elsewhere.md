---
id: 12
title: "Attribute allowlists are pinned to Humble's surface, and the differential oracle silently skips on any other distro"
status: resolved
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

## Resolution (2026-08-01)

**Policy: the tables are the UNION across supported distros.** The justfile
already targets both (`ros_distro := if 24.04 { "jazzy" } else { "humble" }`),
so both were in scope. On a distro lacking an attribute we are now more
permissive than it — the attribute warns instead of erroring, and stock `ros2
launch` still refuses the file there. That direction fails safe; the reverse
breaks working launch files. It is the same call already made for the legacy
aliases (`<group ns=>`).

The differences were **diffed from source**, not guessed —
`external/diff_attrs.sh` against Humble's site-packages and a `jazzy` clone:

| Element | Difference |
|---|---|
| `execute_process` (so `<node>`, `<node_container>`, `<executable>`, via `super().parse`) | Jazzy adds `respawn_max_retries`, `sigkill_timeout`, `sigterm_timeout` |
| `<include>` | Jazzy adds a `let` **child** (`data_type=List[Entity]`, not an attribute) |
| `<node>` | Humble has `node-name`; Jazzy dropped the deprecated alias |
| `lifecycle_node` | Jazzy adds `autostart` — but this parser never dispatches that element, so no spec to update |

**The oracle is no longer pinned.** It prefers an already-sourced
environment, then `$ROS_DISTRO`, then any distro under `/opt/ros`, and
additionally verifies `import launch.frontend` before claiming an oracle
exists. It reports which distro ran (`ROS 2 oracle: humble`) — with union
tables, a passing result is only interpretable if you know what it was
checked against. Verified it still finds ROS 2 with the environment
stripped.

**Divergences are now directional.** `ALLOWED_DIVERGENCES` became
`PERMISSIVE_DIVERGENCES`, checked at comparison time rather than by skipping
the probe entirely. Being MORE permissive than the oracle is sanctioned for
listed pairs; being LESS permissive never is — that is a valid launch file we
would refuse. Verified by deleting `on_exit` from the node spec: the test
reports `<node on_exit=>: ROS 2 accepts, Rust rejects  <-- we reject what ROS
2 accepts; this is never sanctioned` rather than passing.

The union entries are listed as permissive divergences because on Humble the
oracle rejects them. On Jazzy they produce no divergence and the entries
simply go unused — which is the point: one table, correct on both.

Parser commit `ac8eaff`. 462/462, 505/505 with `--features ir`, oracle 3/3.

**Not closed by this:**
- The differential oracle still covers XML only, so the YAML tables have no
  ROS-2-measured safety net (noted when #0010 was fixed).
- Element-level conformance: `pop-ros-namespace` and `declare_argument` are
  parser extensions ROS 2 rejects outright (found while fixing #0011). This
  issue was about attributes. Whether the parser should reject non-ROS-2
  ELEMENTS is a separate policy question, and the union framing above is the
  natural place to decide it.
