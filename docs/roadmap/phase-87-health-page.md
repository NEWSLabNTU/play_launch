# Phase 87 - the Health page: what is wrong with this run, now

Status: **complete** (2026-10-12 to 2026-10-14). Tier 1 of
`docs/design/big-system-observability.md` (W0-W7 of
`docs/design/health-page.md`) and its Tier 2 items H, I, J, K, L, N, plus
lifecycle activation on the startup timeline. M (live chain latency) is
redesigned as phase 88; O (multi-host) is out of scope; DDS work beyond L and
N is phase 89.

## Why

Before this phase the web UI showed members and their logs, one at a time. A
large launch (Autoware: 119 model nodes, 1,418 endpoints) fails in ways no
single node's view shows: a QoS pair that never matches, a crash loop, a
subscription to a misspelled topic, a parameter the node silently ignored, a
discovery storm. The Health page lists what is wrong NOW, each issue with its
evidence and where to look, opened when a condition appears and closed after
two absent reports, journalled to `play_log/<ts>/health.jsonl`.

## Done

| Step | What | Commit |
|---|---|---|
| — | Per-endpoint traffic (`endpoint_stats.json`); serialized and loaned paths hooked; `/rosout` keyed right | `199d961a` |
| — | Entities page: live rate, bytes, QoS per endpoint; `events.jsonl` off by default, switchable live | `93e67a66` |
| W0-W1 | Diagnostics cost fixed (it was not the 30%); per-subsystem self cost | `c2f53af3` |
| W2 | Issue book, `/api/health/issues` (+stream), Health page | `150536c4` |
| W3 | Exits with signal, crash loops, OOM kills, members that stay down (#0069) | `d42a3bfb` |
| W4 | Log rates, error bursts, new errors | `191c3732` |
| W5 | Delivery latency per subscription | `48bfa84f` |
| W6 | Graph against the run: missing, foreign, duplicate nodes; near-miss topics | `fe997f1c` |
| W7 | Startup timeline (Gantt, `/api/startup`, `never_ready`) | `56afbb84` |
| L | Host checks: RMW and ignored DDS config, socket buffers, `/dev/shm`, disk, swap | `bf2ebbcc` |
| K | Resource trends; the monitor's own cost 8.9% → 0.34% of a core | `4f2f0abc` |
| J, I | Parameters against the launch; one time base, `/clock` stalls and jumps | `40882b9f` |
| H, N | TF read where published; discovery load | `37799418` |
| — | Lifecycle activation on the startup timeline | `e49b7949` |

## What measurement changed

Each item was run on Autoware's planning simulator before it was called done,
and most changed on the first run:

- **The "diagnostics costs 30%" finding was wrong** (W0). Per-subsystem
  accounting put diagnostics at 0.2-0.4%; the cost was an intermittent busy
  Fast DDS shared-memory listener in play_launch's own participant. Open; see
  phase 89 D5.
- **W6's near-miss rule** reported 53 topics under the first rule, all
  optional inputs; the narrowed rule reports 0. Eleven unnamed nodes were
  first reported missing (#0017 keys), which is why `ros_name_known` exists.
- **W5** flagged every `/parameter_events` subscriber during startup
  (`ParameterEvent` has a stamp), and one late message after discovery
  flagged a subscriber that keeps up; event topics are excluded and the rule
  judges p95.
- **W7's spawn times** were ~250 ms late when taken where state events are
  received; they are taken where emitted.
- **K's monitor** spent 87 ms of each 2 s tick on NVML queried per process
  and 64 ms in sysinfo's per-process refresh.
- **J** found Humble's `get_parameters` answers an EMPTY response when any
  requested name is undeclared; and 857 of 1,163 undeclared names were
  broadcasts (the vehicle description sent to 72 nodes).
- **N** measured ~5.2 M discovery datagrams dropped in the first minute of
  Autoware under `isolated` (119 participants, Fast DDS's limit is 120).
- **The test suite's `ROS_DOMAIN_ID` allocator** put concurrent tests on one
  domain about 40% of the time; W6's foreign-node check caught it, twice
  (the allocator, then `runtime_enforcement.rs`'s own).

## Not done

- Extrapolation errors in node logs, counted per node (H).
- Multicast on the DDS interface (L).
- Context switches in resource trends: `/proc/<pid>/status` counts the main
  thread only, and a per-thread sum costs what K removed.
- The autostart-driver path of lifecycle activation is tested by unit test
  only; Humble has no launch-file autostart.

Design: `docs/design/big-system-observability.md`,
`docs/design/health-page.md`, `docs/design/live-entity-monitor.md`.
