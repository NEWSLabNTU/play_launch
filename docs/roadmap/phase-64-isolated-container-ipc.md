# Phase 64 — a private load channel for the isolated container

Status: **proposed**, with the motivating measurements recorded below.

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
