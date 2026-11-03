# Phase 97 - links: transport measured, sourced and monitored

**Status (2026-11-03): P1-P4 done (rlm v0.3.4, play_launch 0.24.0); P5 not adopted (user decision, below); P6 deferred.** Design of record: rlm
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

## Results

- **P1 done.** The last reader that did not use the one precedence was
  rlm's per-manifest `chain-latency` flat sum (topic-level values only);
  it now charges each hop its largest edge by `types::edge_transport_ms`,
  which `TopicView::transport_ms` calls too. A third inline copy sat in
  this repo's reaction walk (the guard edge, `manifest_loader.rs`); it
  calls the same function now. Gate: rlm's `endpoint_transport` fixture
  (two subscribers, one override; flat sum 35 -> 45, the remote edge now
  counted) and `types::transport` tests. The walk's totals do not move: it
  already read the subscriber's value first.
- **P2 done (rlm v0.3.4).** `links:`, `sub.<ep>.link`, `link-unsourced`,
  `transport-inline`, `source.measured.run`. A link's number is put on its
  subscribers at parse, so the walk, the derivation and the chain sums read
  it with no change. Only a subscriber names a link (a topic does not):
  phase 90 G3 made a transport the hop into one subscriber. `--explain`'s
  route names the link and the number's source after the guard edge
  (`link 56.00ms (max_transport into '<sub>', sub-level; link
  'host_serial'; measured)`); the leading text is unchanged, tools read it.
  The model carries `SubContract.link` and `Contracts.links`.
- **P3 done.** `measure <run> --model <m>` prints a `links:` fragment:
  each declared link's observed publish -> take max x 1.2, with the run as
  `source.measured.run`, or "unmeasured" with the clock caveat.
- **P4 done.** `check --against <run>`: `transport-exceeded` (error),
  `link-unmeasured` (warning) from the run's `interception/events.jsonl`.
  A subscriber on another host takes nothing the record sees, so its link
  is reported unmeasured rather than guessed.
- **P5 not adopted (user decision 2026-11-03).** On the first consumer
  the monitor cannot be added without changing what the system does. The
  safety island's monitored candidate is the gate's availability across
  the UART, whose clocks are not synchronised, so a monitor could only time
  arrival gaps: no arrival within the sender's period + the bound (100 +
  56 ms, + the executor's check). That is an omission detector tighter
  than the handler's 500 ms, so it would replace hpc_loss's detector and
  its reaction (the image can start only the island supervisor's safe
  state). Observed availability gaps in normal driving reach 124.8 ms (846
  gaps over four a/b acts), leaving about 31 ms against host-side
  scheduling. Decision: no runtime link monitor; links stay measured and
  sourced, `measured-unsupervised` stays a warning, and `check --against`
  holds the bound (`transport-exceeded`; gate: the fixture run
  `contract_takeover_named_link/run_over`, 73.5 ms against 57). The walk's
  link-monitor charge and `link-unmonitored` are not built; `monitor:`
  stays in the grammar, read by nothing yet.
