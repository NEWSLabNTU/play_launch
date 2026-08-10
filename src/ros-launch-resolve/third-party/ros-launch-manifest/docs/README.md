# Documentation Index

What each document is, and a suggested reading order.

## Reading order

**New to the project?**

1. [slides.md](slides.md) — presentation deck: the problem, the manifest
   idea, and the checker in ~25 slides. Fastest orientation.
2. [launch-manifest.md](launch-manifest.md) — the **specification**: the
   manifest format, worked Autoware examples, the format reference, and
   the validation-rule inventory. The normative document.
3. [contract-theory.md](contract-theory.md) — the **theory**: why the
   composition rules are what they are (latency, drops, age, chains,
   burstiness), with the formal derivations.

**Working on or with the checker?**

4. [contract-verification.md](contract-verification.md) — the
   **implementation**: parsing with spans, the rule registry (20
   single-manifest rules), emitters, and which checks run here vs in the
   consumer's cross-scope layer.

**Working on scheduling?**

5. [scheduling.md](scheduling.md) — the **sched crate**: platform files,
   the `SchedMapper` registry (`manual`, `rate_monotonic`,
   `deadline_monotonic`, `chain_aware`), the platform-agnostic ranking
   core + POSIX realizer split, validation helpers, and the legacy
   `system.toml` bridge.

**Archaeology / rationale?**

6. [design-issues.md](design-issues.md) — the issue log: every design
   question raised against the spec, with resolution and rationale.
   Read when you want to know *why* the spec says what it says.

## Document roles

| Document | Role | Authority |
|----------|------|-----------|
| `launch-manifest.md` | Manifest format specification | Normative for the format |
| `contract-theory.md` | Formal foundations | Normative for composition math |
| `contract-verification.md` | Checker implementation | Descriptive (follows the code) |
| `scheduling.md` | Scheduling crate reference | Descriptive (follows the code) |
| `design-issues.md` | Decision log | Historical record |
| `slides.md` | Marp presentation deck | Informal overview |

Consumer-side documentation lives with the consumers:

- play_launch user guide: `docs/guide/rt-scheduling.md` (play_launch repo)
- Scheduling design of record: play_launch
  `docs/superpowers/specs/2026-07-16-rt-config-v2-design.md` (v2) and
  `2026-07-01-shared-scheduling-crate-design.md` (v1 / shared crate)
- nano-ros integration: nano-ros RFC-0050 / RFC-0052

`img/` holds diagram sources (Typst) and rendered assets;
`compile-images.sh` regenerates them.
