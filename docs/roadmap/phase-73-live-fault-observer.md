# Phase 73 — the fault observer runs live

Status: **complete** (2026-09-06). Closes phase 71's first "not done".

Phase 71 W3 observed a fault and its reaction *after* the run, from
`events.jsonl`. The runtime already had the pieces to see it while the
system ran: `runtime_enforcement::RuleEngine` consumes every interception
event, including the DDS liveliness and deadline events the rmw hooks
already deliver (phase 36). What it lacked was the hazard vocabulary and a
clock.

## What it does

`ContractView` carries one `HazardWatch` per model hazard: guard topics,
sink topics, `ftti`, the sink's `settle`, the fastest declared detector on
a reacting subscriber (`max_age` — the model does not carry a lease), and
whether a reacting subscriber reports through `/diagnostics`.

`RuleEngine` keeps per-hazard state and emits three rules into
`runtime_violations.jsonl` and the web UI:

- **`hazard-detected`** (error) — three ways in. A DDS
  `LivelinessChanged`/`LivelinessLost`/`RequestedDeadlineMissed` on a guard
  is the middleware detecting it (`mechanism: qos`). Otherwise the
  observer's **tick** judges silence: the guard's last publish is older than
  the declared detector, or than ten of the topic's median inter-publish
  gaps (at least 50 ms) when none is declared — the same rule `measure`
  uses after the fact. The tick exists because a dead topic never produces
  an event; it runs on the interception consumer's 10 ms poll.
- **`hazard-reaction`** (warning if it fits `ftti`, error if not) — the
  first sink publish after the fault that carries **no guard provenance**.
  The nominal pipeline forwards the guard's `header.stamp`; a reaction
  originates its own. Same rule phase 71 W3 was forced into.
- **`hazard-recovered`** (warning) — the guard resumed.

## The defect the first run found

The observer detected the silence at 207 ms (ten periods) and reported no
reaction — because the reaction had happened at 106 ms, *before* the
observer's own threshold tripped, while no fault was active. The node's
watchdog (100 ms lease) is faster than a cadence-based observer by
construction. Sink publishes are now buffered (256, with stamp), and
detection looks back through them for the first provenance-free one after
the fault. A unit test pins the order: reaction first, tick second, both
reported on the tick.

## Measured

`just fault` in `rt_av_demo`, one run:

```
post-hoc (measure):  reaction 103.55ms after the fault
live (observer):     reaction 103.55ms after the fault, observer noticed at 209.67ms
derived (check):     FDTI 100 + route 7
```

The recipe now fails if the live and post-hoc reaction times disagree by
more than 5 ms.

## Not done

- ~~The observer's silence threshold is `max_age` or cadence, never the
  declared `lease_duration`.~~ Phase 74 lowered the lease to the model and
  applied it to the node; DDS now reports the lapse itself.
- `hazard-detected` fires once per fault; a flapping guard produces a
  detected/recovered pair per flap, which is right but noisy.
