# Admitting composable-node loads

The decision record for how many composable nodes may be coming up at once,
who decides, and why the answer is not a number.

Folds together phase 61 (the edge startup storm), issue #0019 (slow
constructors killed and reported as loaded) and the container-mode analysis in
`container-isolation.md`. Everything here was measured on a 12-core AGX Orin.

## The three containers are three different machines

`--container-mode` picks between designs that differ in the one property that
matters here — where a LoadNode request is executed:

| mode | LoadNode handled | concurrency |
|---|---|---|
| `stock` (`rclcpp_components`) | inline, **on the executor** | serialised |
| `observable` (ours) | inline, on the executor | serialised |
| `isolated` (ours, default) | answered immediately with a pre-assigned id; work runs on a spawn-worker pool | parallel |

`tests/fixtures/sequential_loading` documents the first case as intended
behaviour: *"No parallel loading: Container serializes all LoadNode
requests."* For `isolated`, `clone_isolated_component_manager.cpp` says
`// Respond immediately — node is not yet spawned`.

That difference decides everything below, and a control that ignores it will
be wrong for two of the three.

## Where the limits are, and what each is for

**1. play_launch's global load cap** —
`composable_node_loading.max_concurrent_load_node_spawn`, default 10.

Bounds LoadNode calls in flight across all containers. Its purpose is the
serialised case: flooding an executor-based container with requests it can only
handle one at a time. It was dead config until phase 61 (parsed, defaulted,
asserted in a unit test, read by nothing) — `dispatch_pending_loads` drained
its queue and spawned a task per request.

**It does not apply to `isolated`.** There the calls all return in
milliseconds, so a cap on in-flight calls bounds nothing that costs anything,
while still being able to hold back a container that had capacity. Since
phase 61 the cap is mode-aware: enforced for `stock`/`observable`, disabled for
`isolated`, and the startup line says which and why.

**2. The container's spawn admission** — `isolated` only.

This is where the cost actually is: a forked `component_node` running the
node's constructor. It used to be bounded by a fixed pool,
`kWorkerThreadCount = 4`, with each worker held for the whole constructor.

**3. play_launch's process admission** — the `MemAvailable` floor from
phase 61, governing plain-node and container process spawn. Concurrency there
is off by default because it was measured not to pay; the memory floor stays on
because it never blocks until memory is actually short.

## Why the pool count was the wrong governor

A count cannot distinguish the two reasons a component is slow:

- **Computing.** A first-run TensorRT engine build compiles ONNX inside the
  constructor: CPU-hungry, memory-hungry, and on Tegra its GPU allocations come
  out of system RAM. Throttling these is correct.
- **Waiting.** Blocked on discovery, a service, a parameter server. Costs
  almost nothing, yet holds a slot for its whole duration.

A fixed 4 throttles both identically, so it is simultaneously too strict for
the second and arbitrary for the first.

Measured, six 20 s constructors plus one fast node in one container:

    t+ 0.5s  quick
    t+20.0s  slow2, slow3, slow6
    t+20.6s  slow5
    t+40.1s  slow4, slow1      <- second wave, purely from the pool

`slow1` and `slow4` waited 20 s for a worker, for no reason related to
themselves or to the machine's state.

This is not hypothetical on a perception stack. With
`perception_preset:=camera_lidar_fusion` the golf cart model grows to 191 nodes
and **10 reference ONNX/TensorRT — six of them in one container**
(`traffic_light_recognition/traffic_light_node_container`), each ~45 s of
construction with the engine already cached.

## The decision

**Size the pool out of the way; govern admission by memory.**

`spawn_worker_count()` defaults to `clamp(nproc, 4, 32)`
(`PLAY_LAUNCH_SPAWN_WORKERS` overrides), so the pool is no longer the limit.
`await_spawn_capacity()` then holds a spawn while `MemAvailable` is below a
floor — 1 GiB by default, capped at a quarter of RAM, `0` or
`PLAY_LAUNCH_SPAWN_MIN_AVAIL_MB` to change it.

The floor is **absolute, not a share of RAM**, for the same reason play_launch's
process floor is: what it guards against is one more child allocating before
the next check, and a component needs what it needs regardless of machine size.
The quarter-of-RAM cap only stops a small board being asked to keep more free
than it has.

Two details that make it work rather than merely look right:

- **Admission is serialised on a mutex.** Without it, N workers observe the
  same `MemAvailable`, all conclude there is room, and all fork — the gate
  reads as satisfied exactly once and then admits everyone. Holding the mutex
  costs microseconds on the common path, where the first check passes and
  nothing sleeps.
- **It never blocks forever.** After 120 s the spawn proceeds with a warning
  naming the condition. A launch that will not start is worse than one that
  starts under pressure — the same rule phase 61 applies to its own gates.

Result on the same fixture: one wave instead of two.

    t+ 0.6s  quick
    t+20.0s  slow4, slow3
    t+20.1s  slow1, slow2, slow6, slow5

## What this does not fix

**Concurrency is not the same as capacity.** Six TensorRT builds now start
together, but on a 12-core Orin they contend for CPU, and on Tegra they contend
for the same RAM the memory gate is watching. Expect the gate to engage on a
real perception bring-up — that is it working, not failing. The change removes
an arbitrary limit; it does not manufacture hardware.

**The 30 s ready timeout still applies** (`PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS`,
default 30 s by deliberate choice — issue #0019). Loading six components
concurrently does not help if each is killed at 30 s; the measured ~45 s of
cached construction is already past it. Anyone enabling perception must raise
it.

**Nothing distinguishes "constructing" from "wedged" to an observer.** A
composable mid-build sits in `Loading` and surfaces as `N pending`, which looks
identical at 5 s and at 5 minutes. The container knows better — it holds a
`pidfd` and can see the child is alive — but does not report it.

## Open questions

1. **Report construction progress.** The information exists (child alive,
   elapsed); there is no channel for it. A `ComponentEvent` variant, or a field
   the existing ListNodes verify could read, would make a 10-minute engine build
   legible instead of indistinguishable from a hang. This is the most valuable
   remaining piece, and it pairs with the 30 s default: an operator who sees
   "constructing 28 s" knows to raise the timeout.
2. **Should the ready timeout default change?** It stays 30 s so nothing
   changes for anyone not asking, but the same commit's measurement (~45 s
   cached) means any perception stack fails on it. A liveness-based wait — keep
   waiting while the child is alive, fail the moment it dies — was implemented
   and discarded in favour of the explicit default; worth revisiting now that
   the failure is at least reported honestly.
3. **Is memory the only scarce resource worth gating on?** CPU was measured
   *not* to pay as a startup gate (phase 61: a runnable-task ceiling made things
   worse, because it cannot tell "busy starting" from "busy running"). GPU /
   unified memory on Tegra is not measured at all, and is plausibly the binding
   constraint for concurrent TensorRT builds.
4. **Two independent memory gates now watch the same number** — play_launch's
   process floor and the container's spawn floor. Both are "wait while short"
   so they compose safely, but neither knows about the other, and on a tight
   machine they can each be waiting for headroom the other is about to consume.
5. **`observable` still serialises on its executor**, so a slow constructor
   there blocks every other load in that container. Nothing here changes that;
   it is inherent to loading into a running executor, and it is one more reason
   `isolated` is the default.
