# Phase 64 — a private load channel for the isolated container

Status: **implemented** (2026-08-22). The design is unchanged from the
proposal below; what the implementation settled is recorded in
"What shipped" at the end, and the design of record is
`docs/design/container-control-channel.md`.

## The report

An AutoSDV Autoware stack (54 nodes, 17 containers, 93 composables) launched
on a 12-core AGX Orin with sensors attached, `--container-mode isolated`. The
launch converged — every composable ListNodes-confirmed loaded — but the load
path itself produced a wall of false alarms:

- **14 `LoadNode service call timed out after 30s (attempt 1/3)`** warnings in
  one launch, each followed by `LoadNode response lost — deferring to
  ComponentEvent`. The loads had in fact succeeded.
- **8 `ComponentEvent LOADED not received after 10s; ListNodes confirms it is
  loaded`** warnings — the fallback confirming what the service call could
  not.
- The `/adapi/container` alone timed out on 9 of its composables at once.

The pattern is not a slow container: it is the **rmw service layer jammed** by
tens of concurrent LoadNode round-trips while ~150 fresh processes are all
discovering each other. The graph-discovery and service-response machinery is
the contended resource; the actual loading work (which the isolated container
answers from a pre-assigned id in milliseconds) is fine.

## The observation this phase acts on

play_launch speaks the standard `composition_interfaces/srv/LoadNode` service
to **its own container binary**. That interface is the right thing for a
container play_launch does not own — `rclcpp_components` stock containers, a
user's custom container — because it is the only interface such a container
has. But `play_launch_container` is this repository's own C++: both ends of
the conversation are ours, and nothing obliges the supervisor and its own
container to converse over the most congested channel on the machine.

Proposal: **a private control channel for the isolated container, with
LoadNode kept exactly for the containers that are not ours.**

- Transport: a per-container unix domain socket (path handed down by the
  supervisor at spawn, e.g. under the run's play_log directory). Length-framed
  serde messages: `Load { unique_id, remaps, params }`, `Loaded { unique_id,
  full_node_name }`, `Failed { unique_id, error }`, plus the liveness/progress
  reports the ready-wait already infers indirectly.
- The supervisor selects the channel by container provenance, which it already
  knows: `isolated`/`observable` containers get the socket; `stock` containers
  keep the LoadNode client and the existing retry/deferral machinery.
- ComponentEvent subscription stays for stock containers; for our own it
  becomes redundant (the socket is ordered and lossless) and its 10s
  wait-then-poll heuristic retires on that path.

## What this buys, concretely

- The 30s timeout, triple retry, response-lost deferral, ComponentEvent
  timeout and ListNodes fallback — five mechanisms that exist to paper over
  rmw congestion — all become dead code on the isolated path, which is the
  default and the edge-machine path.
- Load status stops competing with application discovery traffic precisely
  when that traffic is at its worst (startup), which is the only time load
  status matters.
- A jammed ros2 daemon can no longer make a healthy launch look sick: every
  warning quoted above was a false alarm.

## What it does not change

- Stock-container behavior: byte-for-byte the current path.
- The load admission design (`composable-load-admission.md`): the mode-aware
  cap logic keeps its structure; only the isolated arm's transport changes.
- The container's own spawn-worker pool and pre-assigned-id reply, which are
  already immediate.

## Open questions

1. Does `observable` (inline, executor-loaded) want the socket too? It is our
   binary as well; the serialised loading is the executor's, not the
   transport's, so probably yes for status and no for pacing.
2. Version skew: a supervisor from wheel N driving a container from wheel N-1.
   The socket protocol needs a hello/version exchange with a LoadNode fallback
   rather than a hard error.
3. The interception channel already ships an spsc-shm transport; whether to
   reuse that machinery or keep the control channel on the simpler socket is a
   measurement question, and control traffic is tiny — the socket should win
   on simplicity.

## Measurements to carry into the work

From the motivating launch (AutoSDV, 12-core Orin, sensors attached):

| symptom | count |
|---|---|
| LoadNode 30s timeouts, load actually fine | 14 |
| ComponentEvent 10s waits resolved by ListNodes | 8 |
| composables that genuinely failed | 0 |

Acceptance: the same stack launches with zero load-path warnings, and a
deliberately wedged composable still reports honestly (the socket's liveness
messages, not a timeout heuristic, say so).

## What shipped

The transport is a **`socketpair(2)` created before the fork**, not a socket
file. A unix path is capped at 108 bytes — `play_log/<timestamp>/` plus a
namespaced container name can exceed it — and a path also needs a listener, a
bind, a cleanup and a rendezvous timeout. The pair needs none of those, and its
lifetime is exactly right: the channel exists while the container process does,
and EOF means it is gone. The fd is named in `PLAY_LAUNCH_CONTROL_FD`, with
`FD_CLOEXEC` cleared only in the child being exec'd — clearing it in the parent,
the way the interception fds are passed, would leak the fd into every unrelated
node spawned in the same window, and a leaked copy holds the socket open so the
supervisor would never see EOF.

Framing is the length-prefixed one the io-helper protocol already uses, with a
JSON payload. JSON because the far end is C++ and a codec there has to stay
readable by a human debugging a launch; the container carries a ~500-line
reader/writer of its own rather than adding an nlohmann or yaml-cpp rosdep key
to every source build, to parse six field types.

**Answers to the open questions:**

1. *Does `observable` want the socket?* For status yes, for loading **no** —
   and the reason is thread ownership rather than pacing.
   `ComponentManager::on_load_node` creates the node and adds it to the
   executor; running that from the channel's reader thread would race the
   manager's bookkeeping against the executor using it. The hello carries
   `loads_over_socket`, true only for `isolated`, whose loading already runs on
   the spawn worker pool. `observable` still mirrors `loaded`/`unloaded`/
   `crashed` onto the socket.
2. *Version skew.* Hello exchange with a version field, and three fallbacks
   that are all normal rather than errors: an older container never answers and
   the wait expires (`control_hello_timeout_ms`, default 10 s); a mismatched
   version closes the channel from both ends; `composable_node_loading.
   control_socket: false` never creates one. In every case the LoadNode client
   — created regardless — takes over unchanged.
3. *Reuse the interception spsc-shm transport?* No. Control traffic is a few
   hundred bytes per composable, once; the socket wins on simplicity, and it
   also gives the EOF-means-dead property shared memory would not.

**Scope note:** `UnloadNode` stays on the ROS service. All five papered-over
mechanisms are on the LOAD path, and unload is user-initiated, one at a time,
never during the storm.

**What is now dead on the isolated path:** the 30 s timeout, the triple retry,
the response-lost deferral, the 10 s ComponentEvent wait and the ListNodes
verification sweeps — the actor skips both sweeps when loads go over the
socket, because they exist to reconstruct facts a congested rmw layer lost and
nothing is lost on an ordered stream.

**Coexistence rather than replacement.** The container still publishes
`ComponentEvent`, and every supervisor-side transition is guarded on the entry
still being `Loading`, so whichever report arrives first wins and the second is
a no-op. The log names the channel that won (`ComponentEvent LOADED for '…'` /
`control channel LOADED for '…'`).

## W2 (implemented 2026-08-22) — failure detection, retry, and long constructors

W1 removed five mechanisms and left a hole: on the socket path **nothing ever
gives up and nothing can be retried**. The design that closes it is
`docs/design/composable-load-lifecycle.md`. Its core: the container can be
ASKED (`query`/`status`) instead of inferred from silence, and a retry becomes
cancel → confirm → resend, so a double load is unrepresentable. Only two states
permit an automatic resend — a `load` never acknowledged (nothing was forked)
and an id the container states it has no record of.

It also fixes a hazard the service path carried: `rescue_lost_loads`
re-dispatched on a `ListNodes` `Absent` verdict, but the isolated container's
`on_list_nodes` HIDES mid-construction ids, so a merely-slow composable whose
LoadNode response was lost was double-loaded once past `total_budget` — the
report's 14 lost responses are the first half of that condition. That resend is
now refused for a container of ours (where `Absent` is ambiguous) and kept for a
stock one (where it is not: a stock container cannot answer ListNodes at all
while a constructor holds its executor).

What shipped, beyond the design: protocol **v2** (`query`/`status`/`cancel`,
plus `phase` and `cpu_ms` on `constructing`); the container's pending-id SET
became a map carrying phase, pid, plugin, start time and cancellation, because
a set could answer "is it pending" and nothing else; a `queued` heartbeat that
closes the up-to-120 s silent window at the container's memory gate; and
`PLAY_LAUNCH_CONTROL_DROP`, a test-only fault injector, because "the container
never saw this load" and "the container never answers" are unreachable by any
legitimate input and are exactly the branches that decide whether to fork a
second copy of a node.

Two corrections the implementation forced, both recorded at the design:
`max_load_attempts` defaults to **2** (the draft's `1 = none` contradicted its
own "one safe automatic resend", and the budget is also what bounds a
stall-restart cycle); and a restarted load must not inherit the previous
attempt's events — the ComponentEvent topic still delivered the CANCELLED
attempt's `LOAD_FAILED`, and W1's name-and-plugin fallback matching claimed the
fresh entry with it, marking a just-restarted composable `Failed`. Measured,
then fixed by restricting that fallback to entries with no socket tracking.

## Measured on the reference stack (2026-08-22)

The reporting stack itself, in its bag-replay variant (no hardware):
`autosdv_launch logging_simulation.launch.yaml`, 44 nodes, 15 containers, 84
composables, 59 processes, on the same 12-core AGX Orin. Two arms, minutes
apart, `--container-mode isolated` in both; the second arm differs only by
`composable_node_loading.control_socket: false`.

| | control channel (default) | LoadNode service |
|---|---|---|
| `LoadNode service call timed out` | **0** | 18 |
| `LoadNode response lost — deferring to ComponentEvent` | **0** | 18 |
| `ComponentEvent LOADED not received` | **0** | 5 |
| composables that genuinely failed | 1 | 1 |

The one real failure is the same in both arms and is reported verbatim rather
than as a timeout: `cudaErrorDevicesUnavailable (46) … CUDA-capable device(s)
is/are busy or unavailable` — another workload held the GPU.

The zeros are structural, not statistical: on the socket path those five
mechanisms are not reached, so they cannot fire. What the service arm shows is
that they DO fire on this machine and this stack — the reporter's 14 and 8, at
18 and 5 here — with nothing wrong.

Two honesty notes on the arms. The service arm overlapped another agent's
launch on the same board, so its counts are an upper bound under contention;
and the socket arm was itself contended enough for the GPU to be unavailable,
so neither arm ran on a quiet machine. The comparison is about which messages
are reachable, not about a benchmark figure.

The liveness path was exercised for real, and is the second acceptance
criterion: one composable
(`/perception/object_recognition/detection/voxel_based_compare_map_filter`)
never finished constructing, and instead of a timeout or a false promotion the
log carried its own report every 30 s —

```
container:/pointcloud_container: 'composable:/…/voxel_based_compare_map_filter'
still constructing after 135s (pid 348121, alive — reported by the container)
```

— which is the same evidence for a slow constructor and for a wedged one, with
the elapsed time left for the operator to judge.

**Tests.** `tests/tests/control_channel.rs` asserts which PATH loaded, not just
that loading happened: the isolated run must contain no LoadNode service call
and none of the four false-alarm strings; `control_socket: false` must put
every load back on the service; `observable` must keep LoadNode and still
negotiate a status channel; a stock container must get no channel at all; and a
20 s constructor must produce the container's own liveness line rather than any
timeout. `src/play_launch_container/test/test_control_json.cpp` covers the
codec (escapes, `\u` pairs, a `u64` past double's exact range, malformed
input); `just test-cpp` runs it, and `just test-all` calls that.
