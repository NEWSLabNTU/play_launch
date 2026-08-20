# Phase 62: diagnostics observability

Status: **W1 done** (four registry and transport defects fixed, `a7c2da7`,
`adee4f6`, `adf6516`). W2 (node-card badges) and W3 (transition strip) not
started.

## The report

A golf cart Autoware stack (`~/repos/2026-golf-cart`) ran for four hours on the
vehicle with diagnostics monitoring enabled. It produced **405,480 diagnostic
statuses across 69 distinct names**, and not one of them was WARNING or STALE.
Every single row was OK or ERROR.

That distribution is not what a healthy system looks like and not what a broken
one looks like either. It is what a monitor looks like when it cannot represent
the states in between.

Evidence throughout is `play_log/2026-08-18_14-42-44/diagnostics.csv` from that
repo: 405,480 rows, real timing, 69 names. It is a replay corpus for anything
built here, and it needs no vehicle.

## W1: four defects, fixed

### The registry hoarded every status it ever saw

`DiagnosticRegistry::update` pushed each accepted status onto a `Vec`. The only
reader was `get_history`, which had **zero callers** and an `#[allow(dead_code)]`
attribute on it.

All 405,480 statuses stayed resident for the life of the process, each carrying
three `String`s and a `HashMap<String, String>` of key-values. `DiagnosticCsvWriter`
had already written every one of them to `diagnostics.csv`, which is durable and
outlives the run, so the `Vec` was a second copy of a record nobody could read.

Removing it also removed the `tokio::Mutex` guarding it, so `update` is no longer
async. The callers had been awaiting a lock taken solely to push onto a vector
nobody read.

### The debounce destroyed exactly the rows worth keeping

The 100 ms debounce is keyed on `hardware_id/name` and was applied to every
status alike.

Autoware publishes diagnostics at 10 Hz. That is exactly 100 ms, so the debounce
window and the publish interval coincided. A leaf going OK, then ERROR, then OK
inside one window was dropped from the registry **and** from the CSV: no trace
anywhere that it had ever faulted.

The debounce is right for a chatty publisher repeating itself at the same level,
and wrong for a transition. `update` now reports whether the level changed, and a
change bypasses rate limiting.

The hardware-id filter moved ahead of the debounce at the same time, so a
filtered-out diagnostic costs nothing and leaves no bookkeeping behind for a key
that is never recorded.

### Silence read as health

`DiagnosticLevel::Stale` was only ever set from a publisher declaring level 3
itself. A publisher that dies stops publishing; it does not announce anything. So
its last status sat in the registry reading OK for the rest of the run, and a
crashed node was indistinguishable from a healthy one.

That is the mechanism behind the zero-STALE count in the report above.

`diagnostics.stale_after_ms` (default 30 s, `0` disables) now ages an entry that
stops updating. The default is deliberately loose: real systems mix 100 Hz
sensors with 0.1 Hz housekeeping checks, and a threshold below the slowest
publisher's period would flap. Tighten it per deployment.

Ageing is applied on read rather than by a sweeping task, so there is no window
in which the stored value and the reported value disagree, and no second writer
contending for the map. The consequence matters downstream: a diagnostic going
stale raises no change notification, because silence cannot announce itself.

### The view polled

`DiagnosticsView` ran `setInterval(fetch, 5000)`, so an ERROR could sit five
seconds behind the system that raised it, and the level filter re-fetched nothing
in between. The SSE machinery already existed for state updates and metrics; the
diagnostics view simply never used it.

`/api/diagnostics/stream` now sends the whole table plus counts as one JSON
event. Not a delta: the table is tens of entries, the client re-renders all of it
regardless, and a delta protocol would only add a resync path to get wrong.

Two things wake it, and both are load-bearing:

- the registry's change notification, which fires the instant a diagnostic
  changes level. That is the latency this exists to fix.
- a 1 Hz tick, because staleness is evaluated on read and a silent publisher
  generates no event by construction. Without the tick, a diagnostic that went
  stale would keep reading OK until something unrelated changed.

`/api/diagnostics/{list,counts}` remain for scripts and curl.

### Also: a dead default subscription

`/diagnostics_agg` was in `default_diagnostics_topics()` and is published on
almost no system. It requires the classic `diagnostic_aggregator` node, and
nothing launches one by default.

Autoware, the heaviest diagnostics user there is, does not even ship that package
in its overlay. It aggregates with `autoware_diagnostic_graph_aggregator`, which
publishes `tier4_system_msgs/DiagGraph{Struct,Status}` on
`/diagnostics_graph/*`: different topics, different types, not a
`DiagnosticArray`. There is a back-converter in `autoware_diagnostic_graph_utils`
but it is not in the default launch and emits `/diagnostics_array` anyway.

The default now names `/diagnostics` alone. `diagnostics.topics` still takes a
list for anyone who does run an aggregator.

## W2: diagnostics on the node cards

play_launch knows every node it spawned. It has a `/diagnostics` feed keyed by
`hardware_id/name`. It does not join them, so the diagnostics table is a separate
screen rather than an annotation on the thing an operator is already looking at.

Proposal: a badge per node card carrying the worst level among that node's
diagnostics.

**The join is a heuristic and must be presented as one.** `hardware_id` is free
text chosen by whoever wrote the publisher, and `DiagnosticStatus.name` is
conventionally `"<node_name>: <check_name>"` but nothing enforces it. In the
corpus, names look like `controller_node_exe: control_state` and
`topic_state_monitor_vector_map: map_topic_status`, which do match spawned node
names, alongside `/adapi/node/localization: state`, which does not match any
process because it is a composable node inside a container.

Acceptance:

- an unmatched bucket exists and is visible, never a silent drop and never a
  guess
- a composable node's diagnostics attribute to something an operator can act on,
  either the composable itself or its container, decided by measurement against
  the corpus rather than by assumption
- the badge derives from the same aged view as the table, so a node whose
  diagnostics went stale does not show a green badge

Open question: whether the match should be exact-prefix only, or whether a
configurable mapping is warranted. Answer it against the 69 names in the corpus
before writing code.

## W3: level transitions over time

The registry now sees transitions instead of swallowing them, which makes a
history worth keeping for the first time.

Proposal: a per-diagnostic strip of level over time, answering "was it flapping
or did it fail once". Neither the current table nor the CSV answers that without
a spreadsheet.

**Bounded ring buffer, explicitly sized, with the drop visible in the UI.** W1
was precisely what an unbounded one costs, and re-introducing an unbounded buffer
under a nicer name would be the same defect with a chart on top.

Acceptance:

- the buffer is capacity-bounded, and the UI shows when the window has truncated
  rather than implying the visible span is the whole run
- replaying the 405,480-row corpus through it holds steady memory
- a fault that lasted one publish interval is visible in the strip. This is the
  W1 debounce fix's payoff and the test that proves it end to end

## Scope boundary

Everything in this phase is defined over `diagnostic_msgs/DiagnosticArray`, which
every ROS 2 system has. That is the test for anything added here.

Specifically **not** in scope: Autoware's diagnostic graph. The aggregator emits
a 120-unit DAG over 59 leaves, and the propagation through it (which leaf made
which operating mode unavailable) is the interesting part. Consuming it means
taking a dependency on `tier4_system_msgs` or `autoware_adapi_v1_msgs`, and the
AND/OR semantics are Autoware's, not ROS's. A launcher meant for any ROS user
should not carry either.

That work belongs to the vehicle side. Its counterpart phase is
`docs/roadmaps/4-diagnostics-observability.md` in the golf cart repo.

## Measurement note (2026-08-21): the zero-WARNING/STALE framing

Added by a second pass, not by the author of the sections above. The W1 fixes
are not in question — a dead `Vec`, a debounce colliding exactly with 10 Hz,
staleness never derived, and a 5 s poll are defects on their own merits, and
the debounce fix is visibly working (below). What this note questions is the
inference drawn from the corpus.

The golf cart stack was run on the bench Orin, isolated on its own
`ROS_DOMAIN_ID`, `host:=master rviz:=false`, 150 s, twice — once with a
play_launch binary built BEFORE the W1 commits and once after:

| | rows | names | OK | ERROR | WARNING | STALE |
|---|---|---|---|---|---|---|
| pre-W1 binary | 102,221 | 50 | 34,641 | 64,414 | **2,694** | **472** |
| post-W1 binary | 134,099 | 151 | 104,488 | 27,573 | **1,962** | **76** |

**The unfixed monitor recorded 2,694 WARNING and 472 STALE.** So "not one of
them was WARNING or STALE" is not what a monitor looks like when it cannot
represent the states in between — the same code, unfixed, represents both in
volume on the same stack. The vehicle log already in this tree
(`2026-08-17_11-18-23`, 572,425 rows, 181 names) also predates the fixes and
contains 6,448 WARNING and 1,116 STALE.

Three runs disagree with the corpus. The likeliest explanation is that
something about that particular four-hour run differs — configuration, which
nodes were launched, or the vehicle state — not that the monitor was incapable.
Worth settling before W2 and W3 lean on it, because both sets of acceptance
criteria are written against that corpus and the framing motivates the phase.

### What the same measurement does confirm

Distinct diagnostic names went **50 -> 151** across the fix. The rarest of the
110 new names are `net_monitor` leaves appearing 2, 3, 5 and 13 times in the
whole run — precisely the short-lived transitions a 100 ms debounce against a
10 Hz publisher would swallow. That is the W1 debounce fix paying off, measured
end to end, and it is a better argument for the fix than the corpus was.

### Caveats on these numbers

- 150 s of startup, against a four-hour steady-state corpus. Startup is where
  transitions cluster, so WARNING/STALE counts are not comparable in magnitude
   — only their presence or absence is.
- Bench, no sensors: the `vehicle_interface/*` leaves that self-declare STALE
  on the vehicle have no hardware to talk to here.
- Run-to-run variance on a 145-process startup is large; the OK/ERROR split
  moved substantially between two runs of the same config.

## Verification without a vehicle

`play_log/2026-08-18_14-42-44/diagnostics.csv` replays the real thing: 405,480
rows, 69 names, genuine inter-arrival timing. Both W2 and W3 can be built and
tested against it.

One caveat on reading it. Twelve distinct `topic_state_monitor_*` leaves sit at
ERROR for the entire run. That is `autoware_component_state_monitor` correctly
reporting absent topics in a planning simulation, not a fault. Any acceptance
threshold phrased as "ERROR count should be zero" will fail on healthy data.
