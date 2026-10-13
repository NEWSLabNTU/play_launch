# Phase 89 - helping with DDS without touching it

Status: **planned** (2026-10-14). Design of record:
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
