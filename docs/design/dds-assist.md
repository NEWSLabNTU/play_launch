# Helping with DDS without touching it

**Status: design, 2026-10-14; roadmap: phase 89** (`docs/roadmap/phase-89-dds-assist.md`). Builds on the host checks (L) and discovery
load (N) already on the Health page.

## The rule

play_launch never changes a DDS setting of the user's system: no profile it
writes, no environment variable it sets on a node, no sysctl. DDS behaviour
is the user's configuration, and a launcher that quietly alters it makes a
system behave differently under play_launch than under `ros2 launch` — the
opposite of what a better `ros2 launch` should do.

What it does instead: **observe, attribute, and name the fix.** Every DDS
finding says what was seen, why it matters, and the exact change the user
would make (the sysctl, the XML element, the variable), so the decision and
the edit stay theirs.

The one exception is play_launch's OWN participant (its node for parameters,
graph reads and diagnostics). That is play_launch's process, not the user's
system, so how it is configured is play_launch's choice — see item 5.

## What exists (L, N)

`rmw_config_ignored`, `socket_buffer`, `shm_full`, `shm_zombies`,
`participants_over_limit`, `dds_udp_drops`, QoS incompatibility (Entities),
the Host and Discovery sections. Measured this session:
- the Autoware fixture exports `CYCLONEDDS_URI` and runs Fast DDS;
- `/dev/shm` held 773–787 orphaned Fast DDS files (371 MiB);
- Autoware under `isolated`: 119 participants against Fast DDS's 120, and
  its discovery sockets dropped ~5.2 M datagrams in the first minute (single
  ports at 170–219 k), then none.

## Proposals

**1. Per-node DDS environment, checked statically.** A node given its own
`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY` (or Jazzy's
`ROS_AUTOMATIC_DISCOVERY_RANGE`), or DDS config file in the launch file is on
a different network from the rest, silently. The model carries each node's
`env`, so `check` and the Health page can say so before anything spawns.
Autoware sets none (one Qt variable in 119 nodes), so this is for
hand-written multi-robot launch files, where a per-robot domain is a common
pattern and a typo in one is invisible. Cost: none at runtime.

**2. Large messages over UDP.** A message over 64 KiB that crosses processes
without shared memory is IP-fragmented, and with the kernel's defaults
(`ipfrag_time` 30 s, `ipfrag_high_thresh` 4 MiB) a burst of point clouds
fills the reassembly buffer and whole messages vanish — Autoware's DDS guide
exists largely for this. Entities knows each topic's size and whether its
publisher and subscribers share a process; the host check knows the
sysctls and the transport in effect. Report: "/sensing/lidar/points, 1.8 MiB
at 10 Hz to 3 processes; `net.ipv4.ipfrag_high_thresh` is 4 MiB, Autoware's
guide uses 128 MiB". The planning simulator has only two such topics (the
vector map once, the occupancy grid at 6 Hz, 88 KiB); a vehicle has several
at megabytes, so the check matters there. This host is already tuned.

**3. One cause, not five symptoms.** Today the isolated-mode discovery storm
shows as `dds_udp_drops`, a participant count at the limit, startup
`take_deficit`s and late `startup_timeline` rows, each on its own. A
`discovery_pressure` issue opens when drops on the domain's ports coincide
with startup, names the participant count and the container-mode lever
("119 participants; observable would be 49"), and links the others as
evidence. This is presentation over data in hand.

**4. A DDS report to paste.** One copyable block on the Health page (and
`play_launch` writes it into the bundle as `dds_report.txt`): RMW and
version, config files in effect and whether read, the relevant sysctls,
`/dev/shm` state, participants and the limit, drops, QoS mismatches, and
nodes not discoverable. Most DDS help requests stall on exactly these facts.

**5. play_launch's own participant.** Measured this session: in about half
the runs on one day, play_launch's rclrs executor spun at 27–37% of a core
in a Fast DDS shared-memory listener thread of its own participant (gdb; not
diagnostics, not the graph). Its cause is not established; the 770+
orphaned shm files are one lead (`fastdds shm clean` and re-measure, which
deletes files and is the user's call on a shared machine). If it stays, the
fix is in play_launch's participant only — e.g. no shared-memory transport
for play_launch's own node, which carries little data — and would surprise
no user, since none of their nodes changes. Decide after the measurement.

**Not proposed:** changing anything for the user's nodes, auto-cleaning
`/dev/shm`, or setting `RMW_IMPLEMENTATION` to make a config file take
effect. Each is a one-line fix the report hands them.

## Plan

| Step | Work | Acceptance |
|---|---|---|
| D1 | per-node DDS environment check (`check` and Health) | a fixture with one node on another `ROS_DOMAIN_ID` is reported before spawn |
| D2 | large-message fragmentation check | a fixture publishing 200 KiB across processes, with the host's sysctls read, reports the topic, rate and the values |
| D3 | `discovery_pressure` | Autoware under `isolated`: one issue naming 119 participants and 49 under observable |
| D4 | DDS report (page + bundle) | the report from an Autoware run holds every item above |
| D5 | play_launch's own participant | first the orphaned-segment measurement; change only play_launch's participant, and only if the listener cost reproduces |
