# Phase 89 - helping with DDS without touching it

Status: **complete** (2026-10-14). Design of record:
`docs/design/dds-assist.md`. Follows phase 87's L (host checks) and N
(discovery load).

## The rule

play_launch never changes a DDS setting of the user's nodes: no profile it
writes, no variable it sets, no sysctl. It observes, attributes, and names
the change the user would make. The exception is play_launch's own
participant, which is play_launch's to configure.

## Work

| Step | Work | Acceptance |
|---|---|---|
| D1 | Per-node DDS environment (`ROS_DOMAIN_ID`, RMW, localhost / discovery range, config file) checked statically, in `check` and on Health | a fixture with one node on another domain is reported before spawn |
| D2 | IP fragmentation: messages over 64 KiB crossing processes without shared memory, against `ipfrag_time` / `ipfrag_high_thresh` | a fixture publishing 200 KiB across processes reports topic, rate and the values |
| D3 | `discovery_pressure`: drops at startup, participant count and the container-mode lever as one issue | Autoware under `isolated`: one issue naming 119 participants and 49 under observable |
| D4 | DDS report on the page and as `dds_report.txt` in the bundle | an Autoware run's report holds every fact D1-D3 and L, N use |
| D5 | play_launch's own participant: measure the busy shm listener against orphaned segments first; change only play_launch's participant, only if it reproduces | the measurement recorded either way |

## Context

Measured in phase 87: the Autoware fixture exports `CYCLONEDDS_URI` and runs
Fast DDS; `/dev/shm` held 773-787 orphaned Fast DDS files; Autoware under
`isolated` dropped ~5.2 M discovery datagrams in its first minute at 119 of
Fast DDS's 120 participants; play_launch's own Fast DDS shm listener spun at
27-37% of a core in about half of one day's runs. Autoware sets no DDS
variable per node, so D1 serves hand-written multi-robot launch files.

## Results

**D1, per-node DDS environment.** `ros_launch_resolve::ros::dds_env`
groups the nodes whose launch-declared `<env>` sets `ROS_DOMAIN_ID`,
`RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`, `ROS_AUTOMATIC_DISCOVERY_RANGE`
or `ROS_STATIC_PEERS` (cannot discover the rest: warning), or a DDS config
file variable (other transports or QoS: info), to a value other than the
one the rest inherit, with each variable's default applied, so `0` set on a
node is no split from an unset `ROS_DOMAIN_ID`. One implementation, three
places: `check` prints `warning[dds-env-split]` (never a failure: a
per-robot domain is often meant), `launch`/`up` log it before the first
spawn, and the Health page keeps it as `dds_env_split`. Fixture
`dds_env_split.launch.xml` puts the listener on a domain from a launch
argument; the test claims a second free domain for it, so it lands on no
one else's, and checks all three, the log line ahead of `Spawning`.

**D2, large messages.** `large_udp_message`, per topic: a publisher whose
mean message is over 64 KiB with a subscriber in another member, its size,
rate and the number of processes, and the host's `net.ipv4.ipfrag_time` and
`ipfrag_high_thresh` (new in the host facts) beside Autoware's guide values.
A warning only under Cyclone DDS (no shared memory unless configured) with
settings below the guide's; under Fast DDS, whose shared memory is on by
default, it is information for the multi-host case. Which transport a pair
really uses is not observable from here, which is why. Fixture
`big_message.launch.xml` (200 KiB at 5 Hz across two processes); the test
checks topic, size, rate and the value of `ipfrag_time` read from the host.
Autoware: one topic, 590 KiB at 0.1 Hz to 15 processes. Members stand in
for processes: two composables of one `isolated` container count as one, so
the count is a lower bound.

**D3, `discovery_pressure`.** The domain's sockets are read every 5 s from
the start until 30 s after startup completes, then never again. At 1000
datagrams dropped, one issue names the drops and peak rate, the
participants and the limit, the participants under `observable`, and the
symptoms open at the same moment (`take_deficit`, `missing_node`,
`never_ready`, …) as likely the same cause; the plain `dds_udp_drops` it
subsumes is removed. It stays open for the run: it is a fact about how this
launch starts. Autoware under `isolated`, twice: "discovery storm at
startup: 5994798 datagrams dropped, 119 participants … against a limit of
120; under --container-mode observable it would be 49", evidence 35
`take_deficit` (first run) and 19 `missing_node` (second): under the storm
the graph read missed nineteen running nodes.

**D4, the DDS report.** `health/dds_report.rs`: RMW and its version, the
DDS library and its version (from its CMake config: Humble's Fast DDS has
no `package.xml`), the DDS variables with `NOT READ` beside one the RMW in
effect ignores, the kernel's socket and reassembly settings, host-wide UDP
drops, `/dev/shm` and orphaned Fast DDS files, swap, participants against
the limit and under `observable`, startup drops, the per-node environment
splits, and every open DDS issue, several of one kind folded into one line.
On the Health page (collapsed, with a copy button, `GET
/api/health/dds_report`) and in the bundle as `dds_report.txt`, rewritten
each time the host is read and at the end. Autoware's report holds every
item listed for D1–D3 and the host checks.

**D5, play_launch's own participant: measured, left alone.** The steady
state of play_launch's rclrs executor (from its 30 s self-cost lines, the
first minute of each run excluded) across this repo's Autoware runs: the
busy shared-memory listener showed at 26.8–36.1% of a core in four runs on
2026-10-13 (and 15.5% in a fifth) and in none of the nine since 01:18 on
2026-10-14 (0–5.4%, mostly 2–4%). Over the same span the orphaned Fast DDS
files in `/dev/shm` went from about 780 to 849, so the orphans do not
explain it. No change was made to play_launch's participant, and
`fastdds shm clean` was not run (it deletes files on a shared machine). The
cause is still unknown. Found on the way: each `isolated` Autoware run
leaves about 19 more orphans, from the composables play_launch SIGKILLs
when they ignore SIGTERM for 5 s.

**Seen once, not reproduced.** One Autoware run under `isolated` finished
its shutdown (the log ends with `graceful shutdown completed`) and the
process did not exit: its main thread was in a futex wait with the tokio
workers and the rclrs threads alive, for minutes, until killed. The rerun
exited 21 s after SIGTERM. Parked with the `destroy_client` abort.
