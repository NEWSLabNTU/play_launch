# The container control channel (phase 64)

How play_launch loads a composable into a container it wrote itself, and why
that stopped going through `composition_interfaces/srv/LoadNode`.

## The report this answers

An AutoSDV Autoware stack — 54 nodes, 17 containers, 93 composables — launched
on a 12-core AGX Orin with sensors attached, under the default
`--container-mode isolated`. Every composable came up. The log said otherwise:

| symptom | count |
|---|---|
| `LoadNode service call timed out after 30s (attempt 1/3)` | 14 |
| `ComponentEvent LOADED not received after 10s; ListNodes confirms it is loaded` | 8 |
| composables that actually failed | **0** |

`/adapi/container` alone timed out on nine of its composables at once.

The container was not slow. In `isolated` mode `on_load_node` validates the
plugin, pre-assigns a `unique_id` and returns — microseconds — and the fork
happens on a worker thread. What was slow was the **rmw service layer**, while
~150 fresh processes discovered each other and tens of LoadNode round-trips
queued behind that discovery. The load path was reporting on the health of DDS,
not on the health of the load.

## The observation

`LoadNode` is the right interface for a container play_launch does not own: a
stock `rclcpp_components` container, or a user's own. It is the only interface
such a container has.

`play_launch_container` is this repository's C++. Both ends of that
conversation are ours, and nothing obliges a supervisor and its own child to
converse over the most congested channel on the machine.

## The channel

One `socketpair(2)` per container process, created before the fork.

- **Why a socketpair and not a socket file.** A unix socket path is capped at
  108 bytes, which `play_log/<timestamp>/` plus a namespaced container name can
  exceed; a path also needs a listener, a bind, a cleanup and a rendezvous
  timeout. A socketpair has none of that, and its lifetime is exactly the one
  we want: the channel exists while the process does, and EOF means the
  container is gone.
- **How the child finds it.** `PLAY_LAUNCH_CONTROL_FD` names the descriptor.
  `FD_CLOEXEC` stays SET on the supervisor's copy and is cleared only in the
  child being exec'd, from `pre_exec`. Clearing it in the parent — the way the
  interception fds are passed — would leak this container's control fd into
  every unrelated node spawned in the same window, and a leaked copy holds the
  socket open, so the supervisor would never see EOF when the container died.
- **Framing.** 4-byte little-endian length, then a JSON object. The same
  framing the io-helper protocol uses; JSON rather than bincode because the far
  end is C++ and a hand-written codec there has to stay readable by a human
  debugging a launch.
- **Definition of record.** `src/play_launch/src/ipc/container_protocol.rs`.
  The C++ side mirrors it in `control_channel.cpp`, with the version constant
  duplicated and a test on each side.

### Messages

play_launch → container: `hello`, `load`.
container → play_launch: `hello`, `accepted`, `rejected`, `loaded`,
`load_failed`, `constructing`, `unloaded`, `crashed`.

`accepted` carries the pre-assigned `unique_id` — exactly what the LoadNode
response carried, delivered without a service call. Everything after acceptance
is keyed by that id; only `accepted`/`rejected` need the request's `seq`.

`constructing` is new information, not a re-encoding: the container already
knew a child was alive and how long it had been constructing (it polls the
ready pipe in one-second slices and gives up the moment the child dies), and
until now it only wrote that to its own log.

## Who takes loads over it

The hello carries `loads_over_socket`, and the two managers answer differently:

- **`isolated` — yes.** Loading there already runs off the executor, on the
  spawn worker pool. A request arriving on the channel's reader thread runs the
  identical path a LoadNode request would have run, touching only this class's
  own mutexes.
- **`observable` — no; status only.** `ComponentManager::on_load_node` creates
  the node and adds it to the executor. Running that from the reader thread
  would race the manager's bookkeeping against the executor using it. Loads
  stay on LoadNode; `loaded`/`unloaded`/`crashed` are mirrored onto the socket
  anyway, because a lossless status path costs a few hundred bytes.
- **`stock` — no channel at all.** A stock container would not know what the fd
  is for, so none is created.

This answers open question 1 from the roadmap: yes for status, no for loading —
and the reason is thread ownership, not pacing.

## What the socket path does NOT do

- **No startup permit.** The Phase 61 governor's permit means "a LoadNode call
  is in flight"; on this path there is no call to be in flight. The container
  paces its own spawns against `MemAvailable`, which is the governor that was
  measured to matter (`composable-load-admission.md`), and the count-based cap
  was already disabled for `isolated`.
- **No `check_loading_timeouts`, no `rescue_lost_loads`.** Both sweeps exist to
  reconstruct, through `ListNodes`, facts a congested rmw layer lost. Nothing
  is lost on an ordered stream, so running them would only add DDS traffic and
  re-introduce the false alarms.
- **No unload.** `UnloadNode` stays on the ROS service. Every mechanism this
  phase removes is on the LOAD path; unload is user-initiated, one at a time,
  and never during the storm.
- **No parameter round trip.** The service path renders each parameter to a
  string and re-infers its type at the far end, which turns a `string`
  parameter whose value happens to be `"true"` into a `bool`. The socket sends
  the types the model resolved.

## Falling back

Three ways the channel does not end up carrying loads, all of them normal and
none of them an error:

1. **Version skew.** A supervisor from wheel N meeting a container from wheel
   N-1: the older binary never reads the fd and never sends a hello, so
   `await_hello` expires (`composable_node_loading.control_hello_timeout_ms`,
   default 10 s) and the LoadNode client — which was created anyway — takes
   over. A container NEWER than the supervisor announces a protocol version
   that does not match, and both ends close the channel themselves.
2. **`composable_node_loading.control_socket: false`.** No channel is created;
   the load path is byte-for-byte the pre-phase-64 one.
3. **A stock container**, as above.

Because the container publishes `ComponentEvent` as well as writing the socket,
and every supervisor-side transition is guarded on the entry still being
`Loading`, the two report paths coexist without either needing to know about
the other. Whichever arrives first wins; the second is a no-op. The log line
names the channel that won (`ComponentEvent LOADED for '…'` /
`control channel LOADED for '…'`), which is the first thing worth knowing when
a load looks wrong.

## What is still missing

Nothing on this path gives up, and nothing can be retried: a composable whose
constructor never returns stays `Loading` and is reported forever. That is the
safe direction to be wrong in, but it is not the finished story —
`composable-load-lifecycle.md` designs the coordination between failure
detection, retry, and constructors that legitimately take minutes.

## Cost

- One socketpair and two tokio tasks per container.
- One reader thread per container process.
- Roughly 300–800 bytes per composable in each direction, plus a liveness line
  every 15 s while a constructor runs.

## Where the code is

| what | where |
|---|---|
| protocol + framing (definition of record) | `src/play_launch/src/ipc/container_protocol.rs` |
| supervisor end: socketpair, negotiation, load frames | `src/play_launch/src/member_actor/container_actor/control_channel.rs` |
| socket messages → composable transitions | `.../container_actor/control_events.rs` |
| shared terminal handlers (both channels) | `.../container_actor/component_events.rs` |
| fd hand-down at spawn | `.../container_actor/process_lifecycle.rs` |
| container end: framing, dispatch, status | `src/play_launch_container/src/control_channel.cpp` |
| minimal JSON codec (no new dependency) | `src/play_launch_container/src/control_json.cpp` |
| isolated manager's socket load | `src/play_launch_container/src/clone_isolated_component_manager.cpp` |
| tests | `tests/tests/control_channel.rs`, `src/play_launch_container/test/test_control_json.cpp` |
