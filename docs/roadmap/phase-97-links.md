# Phase 97 - links: transport measured, sourced and monitored

**Status (2026-11-03): planned.** Design of record: rlm
`docs/design/links.md` (refines design issue 55). Spans rlm, play_launch and
nano-ros (pull requests). First consumer: the safety island (RTSS demo phase
10, tier 3 as revised).

## Why

A contract's `max_transport` today is a number per subscriber with no
evidence behind it, read three ways by three readers (issue 55, play_launch
#0042), and never checked at run time. For a best-effort link (serial, TCP,
DDS) the number cannot be derived; it can only be measured and then held by a
monitor. The safety island writes `max_transport` eleven times as
per-subscriber overrides, exactly the case the three readers total
differently.

## Units

| Unit | Work | Acceptance |
|---|---|---|
| P1 (design L2) | One transport reader: the per-subscriber value in the model, one derivation, the other readers use it | a two-subscriber override fixture; the walk and `scope-budget` agree; the island's totals before/after recorded (any change explained) |
| P2 (L1) | `links:` in rlm; topics and subscribers name a link; `source:` rules (`link-unsourced`, `transport-inline`) | fixtures; rlm release; `--explain` shows each edge's link and source |
| P3 (L3) | `measure` proposes link bounds from an interception run; clock caveat | a recorded run gives a proposal naming the run |
| P4 (L4) | `check --against`: `transport-exceeded`, `link-unmeasured` | a fixture run over its bound fails |
| P5 (L5) | `monitor:` on a link: the realizer emits a receiver-side age (or gap) monitor; the walk charges its detection; `link-unmonitored` | nano-ros PR generating the monitor; a fixture; the island's link fault seen on the board |
| P6 (L6) | Guaranteed-link extensions (TSN, CAN schedule) | deferred until a consumer has one |

## Order

P1 first (a correctness fix on its own), then P2 + P4 (one release), P3,
then P5 with nano-ros. P6 waits.
