# Phase 85 - what the island's board runs left open

Status: **planned** (2026-10-01). Follows phase 84. Records the open work
found by the Autoware Safety Island's RTSS@Work demo on the S32K344 board:
`simple-autoware-safety-island`, `docs/takeover-trace.md` section 10 ("The
board budget", phase8-W30, follow-ups F1-F3) and section 11 ("Fresh runs
against the board-derived budgets", phase8-W31, "Runtime contract
violations"), and `docs/roadmap/phase-8-rtss-work-demo.md` entries W30 and
W31. The grammar items (D1-D3) belong to `ros-launch-manifest` (rlm,
`~/repos/ros-launch-manifest`); they are tracked here because the checker
that reads them is play_launch's.

## Why

phase8-W30 sized the island handler's `call_mrm` hop from all 13 W8 board
runs, each term its observed maximum + 20 %:

- link (gate publish on the host -> island take, over the UART): 57 ms
  (max 47.43);
- tick (a take waits for the next 100 ms tick, ticks up to 114.84 ms
  apart): 118 ms (100 + 18);
- work (tick start -> the reaction's last publish): 31 ms (max 25.04);
- `call_mrm` = 206 ms (was 110). `driver_exit` (tick + work, no link)
  = 149 ms (was 100).

The link is also stated as `max_transport: 57ms` on the handler's
`operation_mode_availability` subscriber, the per-subscriber key rlm has
for it. play_launch 0.13.0 does not read it in the fault arithmetic:
`walk_reaction` (`manifest_loader.rs`) sums path latencies and sampling
periods only, and a reported fault's detection is the publisher's period
plus its path latency. With an external publisher the nominal graph has no
edge to weigh either, so the island contract's `check --explain` output is
byte-identical with and without the key. The 57 ms is therefore counted
inside `call_mrm`, which also charges it on the first hop after the
window's deadline, where no message crosses the link: there it is slack.

Three more costs have no key to be stated in. A timer has a rate and no
release jitter, so the tick share (100 + 18) lives inside `call_mrm` and in
the island's `tools/timeline/analysis.py` as `TICK_MS`. A service edge has
no transport or queueing key, so the comfortable-stop operator's
serve-after-tick cost (0.85-1.18 ms after the handler returns) is also
inside `call_mrm`; it cannot go on the operator's own path, because a
node-path `max_latency` also becomes nano-ros's derived node deadline and a
runtime latency monitor in the image. And an on-demand topic cannot say it
has no minimum rate.

phase8-W31 ran nine fresh acts on an image built from W30's contract. Every
row passed. The executor's violation ring, read over SWD at bring-up, was
already full with 8 start-up entries, 2 of them `rate-hierarchy-runtime`
on the comfortable-stop operator's `clear_velocity_limit` and
`max_velocity_candidates`: on-demand topics the contract gives a 10 Hz
`min_rate_hz`. The 206 ms `max-latency-runtime` monitors bound one
dispatch: the longest handler callback that published a monitored topic
was 24.93 ms. The route the checker charges is the wait for the tick plus
the link plus the work, which no runtime check measures. The link also
reached 67.02 ms in one run (r01), past the stated 57.

## Design

- **D1 (island F2, rlm): a release-jitter key on a timer trigger.** A timer
  trigger has a rate and no release jitter. The island states the tick
  share (100 + 18 ms) inside `call_mrm`'s 206 ms and as `analysis.TICK_MS`.
  A jitter key on the timer would let `walk_reaction` and `window-expiry`
  charge period + jitter themselves, and leave `call_mrm` holding only the
  call. Decide the key's name and place (the trigger, next to the rate)
  and how `window-expiry` compares a first hop against period + jitter.
- **D2 (island F3, rlm with nano-ros): a service edge's cost, and budget
  versus deadline.** A service edge has no transport or queueing key. A
  node-path `max_latency` doubles as nano-ros's node deadline and its
  runtime monitor, so the comfortable operator's serve-after-tick cost
  (0.85-1.18 ms) cannot be stated where it happens without changing the
  image's scheduling. Needs a key on the service edge, and a decision on
  separating "budget" (what the checker charges) from "deadline / monitor"
  (what the image schedules and checks at run time). Cross-repo with
  nano-ros.
- **D3 (rlm): an on-demand topic with no minimum rate.** There is no way to
  say "no minimum rate". The island contract's 10 Hz `min_rate_hz` on the
  comfortable-stop operator's `clear_velocity_limit` and
  `max_velocity_candidates` produced 2 `rate-hierarchy-runtime` violations
  at start-up on the board (W31 ring readout). Decide: an on-demand key in
  rlm, or the contract just drops `min_rate_hz` on those topics.
- **D4 (with nano-ros): route versus callback.** What the runtime latency
  monitor checks (one dispatch's elapsed time) is not the route the checker
  charges (tick wait + link + work). Decide what a route-level runtime
  check means, so that the contract's static and runtime halves state the
  same quantity, or say explicitly that they do not.

## Implementation

- **I1 (island F1): transport on the reaction walk.** Charge
  `sub.max_transport ?? topic.max_transport` on the guard edge in
  `walk_reaction` and in a reported fault's detection, but not on the first
  hop after a window's deadline (the owner reads its own clock; nothing
  crosses a link there). Today 0.13.0's `walk_reaction` sums path latencies
  and sampling periods only, and the island contract's `--explain` output is
  byte-identical with and without the key. After this, the island's
  `call_mrm` drops 206 -> 149 ms and the window ends within 10,149 ms.
  `check --explain` should show the transport term where it is charged.

## Test/check

- **T1: the F1 regression.** `--explain` must change when `max_transport`
  is present. An island-style fixture (external publisher behind a
  transport, a windowed rung owned by a 100 ms tick) expecting `call_mrm`
  149 and the window's end within 10,149 ms, and no transport on the hop
  after the deadline.
- **T2: fixtures for D1, D2 and D3** once the keys exist: a jittered timer
  charged period + jitter by the walk and by `window-expiry`; a service
  edge whose cost is charged without becoming a node deadline; an on-demand
  topic that `rate-hierarchy` does not hold to a minimum rate.
- **T3: the open Dependabot alerts** on NEWSLabNTU/play_launch. 6 open, all
  severity high, all in the root `package-lock.json`: 5 on `fast-uri`
  (#40, #41, #42, #43, #46) and 1 on `js-yaml` (#45). Bump or override the
  two packages and confirm the alerts close.

## Consumers / cross-repo

- The island contract change waits on I1: `call_mrm` 206 -> 149 ms, and
  the link's `max_transport` re-sized from 57 to about 81 ms (+20 % over
  W31's 67.02) once more runs size it again. Until I1 ships the re-size
  moves no verdict, because 0.13.0 reads neither number in the fault
  arithmetic.
- D2 and D4 pair with the nano-ros phase for the island's findings: the
  budget / deadline split and a route-level runtime check need both
  toolchains to agree.
- D1-D3 land in rlm first and are pinned here, as phases 83 and 84 were.
