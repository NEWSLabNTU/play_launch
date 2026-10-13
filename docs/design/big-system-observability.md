# Observability for big ROS 2 systems

**Status: proposal (no code).** Tier 1 is next and designed in
`docs/design/health-page.md`; Tier 2 and 3 are optional. What goes wrong in large ROS 2 systems
(Autoware-sized: 100+ processes, 1,400 endpoints, a dozen containers), which
signal shows each problem, where play_launch can get that signal, and what
the web UI should show. Ranked by value over cost on an AGX Orin.

The organising rule: **play_launch already sits where most of these signals
are cheapest to get.** It spawned every process (pids, exit codes, cgroups,
stdout/stderr), it is inside every process (the interceptor), and it holds
the model of what *should* be running. A tool that attaches afterwards has
to subscribe, poll the graph, or trace the kernel to learn the same things.

## What goes wrong

Collected from the Autoware troubleshooting and DDS-settings guides, the
ROS 2 executor-timing literature, ROS Discourse, and this repository's own
issue history (which is a fair sample: every item below has bitten a run
here or a deployment it serves).

| # | Pain point | How it presents | Seen here |
|---|---|---|---|
| 1 | **Startup storm / slow bring-up** | minutes to "ready", runnable queue in the hundreds, OOM on edge boards | phase 61 (load1 203 on an Orin) |
| 2 | **Composable that never loads, or loads late** | node "running" but its topics silent; LoadNode timeouts | #0016, #0019, phase 64 |
| 3 | **Crash loops** | a node restarts every few seconds; logs scroll past | rviz2 ×11 on today's Autoware run |
| 4 | **Lifecycle node left unconfigured/inactive** | process alive, publishes nothing | phase 86 |
| 5 | **DDS discovery scaling** | n² discovery traffic, packet storms when nodes join, worst over Wi-Fi; Cyclone's "Failed to find a free participant index" past ~32 participants per host | `cyclonedds.xml` in the Autoware fixture exists for this |
| 6 | **Domain collisions / foreign nodes** | another team's robot or a stray `ros2 run` on the same `ROS_DOMAIN_ID` injects traffic | test suite's per-run domain ids exist for this |
| 7 | **QoS mismatch** | endpoints never match; silent topic, no error unless the node polls QoS events | Entities page (done) |
| 8 | **Silent, stale, or slow topics** | a sensor driver hangs; rate halves under load | Entities page (done) |
| 9 | **Messages dropped by shallow queues / starved executors** | subscriber takes less than is published; callbacks run late | Entities take-deficit (done); RTCSA 2023 on Autoware: loss from buffer sizes and config |
| 10 | **Executor starvation and callback latency** | a slow callback delays every other in its group; `/clock` and `/tf` callbacks blocked, TF timeouts follow | executor literature; Discourse |
| 11 | **TF problems** | "extrapolation into the past/future", missing frames, two publishers for one frame, a static transform published as dynamic | a staple of every ROS support forum |
| 12 | **Time** | `use_sim_time` true in some nodes and false in others; `/clock` stalls or jumps; time moving backwards on bag loop | common in simulation |
| 13 | **Remap and naming mistakes** | a publisher on `/sensing/lidar/top/points` and a subscriber on `/sensing/lidar/top/pointcloud`; both "work", nothing flows | #0021, phase 76/77 |
| 14 | **Parameter drift** | someone ran `ros2 param set`; the running system no longer matches the launch file | — |
| 15 | **Resource leaks and pressure** | RSS climbs over hours; fd or thread count grows; cgroup throttling; OOM kills | phase 66 |
| 16 | **Log floods** | one node logs at kHz, fills disk, hides real errors | 40 GB `play_launch.log` in 5 min (fa366b23) |
| 17 | **Host misconfiguration** | small UDP receive buffers (large messages fragment and drop), `/dev/shm` full of segments left by crashed processes, no multicast | Fast DDS / Cyclone tuning guides |
| 18 | **The monitor itself costs too much** | the tool watching the system is a top CPU consumer | play_launch's diagnostics subscriber: ~30% of a core on Autoware (measured 2026-10-13) |

## What to monitor, and where the data comes from

"Have" means play_launch already collects it. "Derive" means the data is
already in hand and only needs computing. "New" needs a new source.

### Tier 1: cheap, high value, mostly data already in hand

**A. "What is wrong right now": a Health page (the landing view).**
One list, newest first, every open problem across all the sources below,
each with the node, how long it has been true, and a link to the page that
explains it. Today the information is spread over Nodes, Diagnostics and
Entities, and a reader has to know where to look. Derive, no new data.
Cost: nothing measurable.

**B. Crash loops and exits** (pain 3). Per node: restarts in the last
10 min, last exit code or signal, time alive before each death, and whether
the kernel killed it for memory (`memory.events` `oom_kill` on the node's
cgroup, phase 66, when delegated; else the exit signal). Flag "crash loop"
at ≥3 restarts in 5 min. Have: exit status and respawn counts (the actor
logs "crash 11 since last healthy run"). Cost: none.

**C. Startup timeline** (pains 1, 2, 4). A Gantt chart, one row per member:
spawned → first endpoint created → first message published → loaded /
lifecycle active. Shows at a glance which nodes held up "ready" and which
never got there. Data: actor spawn times (have), the interceptor's init and
first-publish events (have, timestamped), container load reports (have),
lifecycle state (have from the transition-event subscriber). Cost: none;
computed once.

**D. Delivery latency per subscription** (pains 9, 10). For stamped
messages the interceptor sees the publish (monotonic time, stamp) and every
take (monotonic time, same stamp), on one host clock. Pairing them gives,
per subscription, how long a message waited between publish and the
callback taking it: transport plus executor queueing. A p99 that grows
under load is a starved executor, and this measures it **without tracing**.
Show p50/p99 per subscription in Entities, and flag p99 above 100 ms or
above the topic's own period. Derive from events already on the ring.
Cost: one hash lookup per take (stamp → publish time, bounded map per
topic, as `RuleEngine` already keeps for max-latency). ~0.1 µs per event.

**E. Log rate and error bursts** (pain 16). Per node: lines/s on
stdout/stderr and `/rosout`, ERROR/WARN counts per minute, and the top
repeated messages after stripping numbers (today's Autoware run printed one
`QoSProfile.conversion: History policy is unknown` line per endpoint).
Flag a node logging over 100 lines/s, or any new ERROR signature. Data:
`/rosout` rate per node is already in Entities; the out/err files are
already tailed for the log viewer. Cost: line counting on files already
read; the signature table is bounded.

**F. Graph versus model** (pains 2, 6, 13). Live diff of the running ROS
graph against the SystemModel:
- **missing**: a node the model expects that is not on the graph;
- **unexpected / foreign**: a node on the graph play_launch did not start.
  Most often a domain-id collision or a stray process; show its host if
  DDS reports it;
- **near-miss topics**: a silent subscriber whose topic is one edit (or one
  path segment) away from a live publisher's. This catches remap typos that
  every other check misses.
Data: interceptor endpoint records (have), model (have), graph API for
foreign nodes (have, `graph_builder.rs`). Cost: the graph query is the only
expensive part; run it every 10 s, and only while the page is open.

**G. play_launch's own cost** (pain 18). One panel: play_launch's CPU and
RSS, CPU per subsystem (monitor, diagnostics, interception consumer, web
server), ring drops, `events.jsonl` size, SSE clients. The measurement that
found the diagnostics subscriber at ~30% of a core took an A/B of six
Autoware runs; the panel would have shown it at a glance. Data: per-thread
CPU from `/proc/self/task` (name the tokio tasks' threads, or time each
subsystem's loop). Cost: one `/proc` read a second.

### Tier 2: high value, needs a new but contained source

**H. TF health** (pain 11). A TF page: the frame tree, each edge's
publisher node, update rate and staleness, and flags for a frame published by
two nodes, a dynamic transform that never changes (should be static), a
static transform republished at rate, and a gap between frames that leaves
the tree disconnected. Plus a counter of `extrapolation` errors per node from
the logs (E). Data: `/tf_static` is latched and rare, so subscribing to it
is cheap. `/tf` is not: hundreds of messages a second. Two options:
1. subscribe in play_launch and process at most N messages a second, keeping
   only (parent, child, publisher, last stamp, count): bounded, but it is a
   subscription, the kind that costs 30% when done carelessly (the
   diagnostics lesson); or
2. a TF plugin in the interceptor that reads the `TFMessage` frame ids on
   publish, which is already a hook, and ships one event per (frame pair,
   second). No subscription; the cost is a short string walk per `/tf`
   publish, in the publishing process.
Prefer 2. Cost: a few hundred ns per `/tf` publish.

**I. Time consistency** (pain 12). Per node: `use_sim_time` (parameter
query, once at start and on `/parameter_events`); flag a mix. `/clock`: rate,
last value, jumps backwards, stalls over 1 s while nodes are running. Data:
parameters (have the client, phase 24); `/clock` rate via Entities. A stall
needs the value: take it from the interceptor's frontier on `/clock`
(have). Cost: none.

**J. Parameter drift** (pain 14). List every parameter whose running value
differs from the model's, with when it changed and from which node. Data:
`/parameter_events` (already subscribed for the web UI) against the model's
`param_sources`. Cost: none beyond today.

**K. Resource trends** (pain 15). **Done 2026-10-13**, after first cutting
the monitor's own cost from 8.9% to 0.34% of a core on Autoware (49
members, 2 s ticks). A per-phase profile of one tick, ~180 ms of CPU: NVML
queried per PROCESS, 87 ms (`running_compute_processes` on every device
for every member; now once per device per tick, which also matches an
`isolated` container's children, the processes that hold the CUDA
context); sysinfo's per-process refresh, 64 ms (it walks every thread's
`stat`; now one `/proc/<pid>/stat` and `io` read per process);
namespace-wide socket counts, 15 ms (now reused for 30 s); the subprocess
walk rerun in full whenever any pid changed, 8 ms (now only for the
members that came). Trends (`monitoring/trends.rs`): RSS and open fds
sampled ~once a minute over 30 min (`PLAY_LAUNCH_RESOURCE_TREND_WINDOW_S`),
a least-squares fit with its r²; threads; cgroup `cpu.stat` throttling and
`memory.events` where a group exists. Health: `memory_growth` (steady,
r² ≥ 0.8, over 2/3 of the window, > 1 MiB/min and ≥ 10 MiB),
`fd_growth`, `cpu_throttled`, `memory_high`, and a Resources section.
Autoware with a 120 s window: nothing reported. Not kept: context
switches, since `/proc/<pid>/status` counts the main thread only and a
process total reads every thread's file. Original proposal: Today Nodes shows CPU and memory now. Add,
per node: RSS slope over the last 30 min (a leak is a trend, not a level),
fd count, thread count, voluntary/involuntary context switches, and cgroup
`cpu.stat` throttled time and `memory.events` (`high`, `max`, `oom_kill`)
where the cgroup exists. Flag RSS growing for 30 min at over 1 MB/min. Data:
`/proc/<pid>/{status,fd,stat}` (the monitor reads some already); cgroup files
(phase 66). Cost: the monitor's existing sampling; see its own cost in G
first — at 6.6% of a core today it is the second-largest self-cost.

**L. Host checks** (pain 17). **Done 2026-10-13** (`health/host.rs`):
middleware in effect and DDS config variables it ignores, Cyclone's
`SocketReceiveBufferSize min` against `rmem_max`, `/dev/shm` use and
orphaned Fast DDS files (by `/proc/locks`, read-only), free disk under the
log directory, swap rate; issues `rmw_config_ignored`, `socket_buffer`,
`shm_full`, `shm_zombies`, `disk_low`, `swapping`, and a Host section on the
Health page. Multicast on the DDS interface is not checked. Original
proposal: A one-time-plus-periodic panel:
`net.core.rmem_max`/`wmem_max` against the middleware's recommendation
(Autoware's DDS guide lists the values), `/dev/shm` usage and segments
owned by dead pids (Fast DDS leaves `fastrtps_*` behind after a crash),
multicast on the DDS interface, free disk under `play_log`, swap use, and
the RMW and DDS config file in effect (`RMW_IMPLEMENTATION`,
`CYCLONEDDS_URI`, `FASTRTPS_DEFAULT_PROFILES_FILE`). Cost: negligible.

### Tier 3: valuable, larger or narrower

**M. Live chain latency** for paths a contract declares: p50/p99 from
sensor stamp to actuator publish, against the budget. The RuleEngine already
computes max-latency; expose the distribution. Only systems with contracts.

**N. Discovery load** (pain 5): participant count on the domain against the
middleware's limits (warn near Cyclone's participant-index cap), and
discovery traffic, which needs packet counting (`/proc/net/udp` drops, or the
SPDP/SEDP ports' bytes). Container mode is the big lever here (fewer
participants under `observable`); the panel should say how many participants
each mode would create.

**O. Network bandwidth per interface and multi-host clock offset**
(`/proc/net/dev`, chrony). Only matters for multi-host deployments.

## Cost on an AGX Orin

Measured on Autoware (x86, ~2,200 events/s), play_launch's own CPU: 1.1%
baseline, +1.0% for interception and Entities, +6.6% for resource
monitoring, +30% for the diagnostics subscriber. Tier 1 adds per-event work
only in D (~0.1 µs per take) and file reads elsewhere; at Autoware's rate
that is well under 1% of an Orin core. **Fix the diagnostics subscriber
first**: it costs more than everything proposed here combined.

Rules for anything added, from what the measurements showed:
1. **No new per-message subscription in play_launch** unless rate-limited
   and measured. Prefer the interceptor (already in the process, already
   paid for) or a latched/low-rate topic.
2. **Compute on the page, not always**: graph queries and anything O(topics)
   run only while a client watches, at ≤0.1 Hz.
3. **Every feature reports its own cost** in G, so a regression shows up as
   a number, not as a slow robot.

## Suggested order

1. Fix the diagnostics subscriber's cost (not observability work, but it
   pays for everything below).
2. G (self-cost) — so every later step is measured as it lands.
3. A (Health page) with B, E and the existing Entities flags feeding it.
4. D (delivery latency) and F (graph vs model, near-miss topics).
5. C (startup timeline).
6. H (TF), I (time), J (parameter drift), K (trends), L (host checks).

## Sources

- Autoware runtime troubleshooting (participant index limit):
  https://docs.autoware.org/1.8.0/community/support/troubleshooting/runtime-troubleshooting/
- Autoware DDS settings: https://docs.autoware.org/main/installation/additional-settings-for-developers/network-configuration/dds-settings/
- Zenoh on ROS 2 discovery overhead: https://zenoh.io/blog/2021-03-23-discovery/
- Timing-aware ROS 2 architecture (message loss from buffer sizes in
  Autoware.Universe): https://research.utwente.nl/en/publications/timing-aware-ros-2-architecture-and-system-optimization/
- Experimental evaluation of callback behavior in ROS 2 executors:
  https://www.es.mdu.se/publications/6796-Experimental_Evaluation_of_Callback_Behavior_in_ROS_2_Executors
- Discourse, clock and TF callbacks blocked under the default executor:
  https://discourse.openrobotics.org/t/is-fastdds-default-config-totally-unusable-for-simulation-with-rclpy/54553
- ros2_tracing: https://arxiv.org/pdf/2201.00393 ; CARET:
  https://github.com/tier4/caret
- Shared memory and large messages:
  https://discourse.openrobotics.org/t/shared-memory-transport-in-fastdds-for-image-publisher-has-lower-frame-rate-than-udp-transport/39874
