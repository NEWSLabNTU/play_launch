---
id: 101
title: "Nav2 with use_composition:=True does not load: tuple target_container, callable on_start, unordered loads"
status: resolved
type: defect
severity: high
---

# 0101 - Nav2 under composition does not come up

**Repo:** `play_launch` 0.41.0 (PyPI, cp312 x86_64)
**Affects:** pyexec's `pyobject_to_string` and `OnProcessStart` stand-in,
the Python dumper's handling of `OnProcessStart`, and the container
actor's LoadNode dispatch
**Found:** amr 175 (Jazzy, Ubuntu 24.04), 2026-11-20, loading
`nav2_bringup/navigation_launch.py use_composition:=True` into a
respawning container

Three defects, each with a stock-Jazzy repro, kept Nav2 from starting.

## 1. A tuple `target_container` was not performed

`navigation_launch.py` passes
`LoadComposableNodes(target_container=(namespace, '/', container_name))`.
launch_ros normalizes any list or tuple of substitutions and joins what
they perform. pyexec's `pyobject_to_string` joined a `list` but not a
`tuple`, so the tuple's repr became the container name
(`(LaunchConfiguration('namespace'), '/', ...)`) and all eleven Nav2
servers stayed `Blocked(ContainerNotFound)`.

Fixed in `pyexec/src/api/utils.rs`: a tuple is joined like a list.
Fixture `execution_semantics/target_container_tuple`, which stock resolves
to `/nav2_container`. The Python parser already performed it.

## 2. A callable `on_start` was dropped without a word

`RegisterEventHandler(OnProcessStart(target_action=X, on_start=f))` with a
function `f(event, context)` (launch accepts it and executes what it
returns) was skipped by the walk, so a launch that loads its composables
from such a handler, to reload them when a respawned container starts,
ran with an empty container and no warning. The list form was kept.

Fixed in `pyexec/src/api/event_handlers.rs`: the function is called with
a `ProcessStarted`-shaped event (the target action and its name; no
process yet, so `pid` is `None`) and the launch context, and what it
returns is walked as an `OpaqueFunction`'s result is. A function that
raises is reported and skipped. A callable `OnStateTransition` `entities`
is now warned about rather than dropped silently.

The Python dumper dropped `OnProcessStart` in BOTH forms, since nothing
emits a start event during a dump; it now walks a list in place and calls
a function the same way (`dump/visitor/action.py`), and the two parsers
produce the same model for fixture `execution_semantics/on_start_callable`.

## 3. One `LoadComposableNodes` loaded concurrently, not in order

launch_ros loads the descriptions of a `LoadComposableNodes` one after
another, waiting for each LoadNode response. The container actor sent
every LoadNode at once, and in `HashMap` order, so a component listed
later could be constructed first. Nav2's lifecycle manager, listed after
the servers it manages, then blocked a single-threaded container's
executor waiting for their services, and the container never answered
the LoadNode that would have created them (`LoadNode service call timed
out after 30s`, 3 runs of 3).

Fixed in the container actor (`container_actor/supervisor.rs`): the
auto-load pass takes composables in launch-file order (`order`, assigned
at registration, which follows the model), and LoadNode calls go one at a
time per container (`load_in_flight`), the next dispatched when the last
completes. A container that runs LoadNode on its executor (`stock`,
`observable`) serialises the calls anyway, so this costs no concurrency
there; `isolated` loads over the control socket and is unaffected. Loads
in different containers still proceed together. A completion carries its
dispatch sequence, so a call left over from before a container restart
cannot release the slot of a later one.

Verified on Humble with eight `demo_nodes_cpp` talkers in one container:
0.41.0 loaded them in a different order in each of three runs, this fix in
launch-file order in all three. Unit test
`auto_loads_come_in_launch_file_order`.
