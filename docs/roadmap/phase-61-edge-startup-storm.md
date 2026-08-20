# Phase 61 — the edge startup storm

Status: **W1 done** (diagnosis + admission control + OOM bias + two defect
fixes). **W2 done** (staged startup: start consumers before producers).
W3 — per-node isolation granularity — not started, and it is the item that
would actually let a vehicle keep crash containment and lose the storm.

## The report

A golf cart Autoware stack (`~/repos/2026-golf-cart`) was launched on an AGX
Orin with sensors attached. play_launch started every node at once, the machine
went into a resource storm, and the **kernel OOM killer took the operator's
GNOME session**. The same stack is fine on a workstation.

## Reproduction

Bench AGX Orin, 12 cores, 64 GiB + 32 GiB swap — the same hardware as the
vehicle, confirmed with the reporter. No sensors attached.

    play_launch launch golfcart_launch golfcart.launch.yaml \
        host:=master rviz:=false imu_source:=xsens camera_model:=gscam

play_launch's own line: `Parsed: 44 node(s), 16 container(s), 84 composable
node(s)`. Under the default `--container-mode isolated` each composable is
fork+exec'd into its own process, so **144 processes** start.

Harness: [`scripts/edge-storm/`](../../scripts/edge-storm/README.md).
`memwatch.sh` samples `/proc/meminfo`, `/proc/stat` and `/proc/vmstat` at
100 ms; `cpuwatch.py` samples every `/proc/<pid>/stat` for per-process
attribution; `oom_guard.sh` kills the launch process group if `MemAvailable`
falls below a floor, so the experiment could not repeat the vehicle's outcome on
the machine running it; `analyze_repro.py` summarises a trace.

### What reproduced, and what did not

| | measured |
|---|---|
| peak runnable tasks | **484** on 12 cores |
| peak load1 | **203** |
| CPU during startup | **85% of all 12 cores for 49 s** |
| MemAvailable consumed | 4.0 GiB |
| OOM kills | none |

The **memory** storm did not reproduce; the **CPU** storm did, hard. 4 GiB
against 64 GiB is nothing, and that gap is informative rather than a failed
repro: on the vehicle the sensors keep publishing into nodes that cannot get
CPU for ~50 s, and DDS and callback queues absorb the backlog. Here there is
nothing to queue. The CPU storm is the cause; the OOM is its consequence.

**This is the one part of the diagnosis that is inferred rather than measured**,
because it needs sensors. Confirming it means re-running on the vehicle with
`memwatch.sh` attached.

### Attribution

Per-process CPU over the 49 s startup window (`cpuwatch.py`, weighted by real
sample intervals — the sampler was itself starved and its nominal 500 ms
interval stretched to 2.5 s, which invalidated a first naive pass):

| process | cpu-s | share |
|---|---|---|
| `component_node` (84 procs) | 252.9 | **50.5%** |
| `component_container` (19 procs) | 61.4 | 12.3% |
| everything else | ~186 | 37% |
| **total** | **500.8** | 85% of 588 available |

Nearly two thirds of the storm is the composable-node machinery: the 84
`component_node` processes that `--container-mode isolated` forks. They exist
for a reason (see "The lever that does work" below) — this is what fault
isolation costs, not a defect.

## Findings in the code

1. **No admission control existed at all.** `builder.rs:215` spawns one tokio
   task per node immediately; `regular_node_actor.rs` `handle_pending` called
   `command.spawn()` with no gate.
2. **`max_concurrent_load_node_spawn` was dead config** — parsed, defaulted,
   asserted in a unit test, read by nothing. `dispatch_pending_loads` drained
   its queue and spawned a task per request, so all 84 composables went into
   LoadNode at once. `delay_load_node_millis` was dead the same way.
3. **`startup_complete` was `composable_pending == 0`**, which is true at t=0:
   a composable whose container has not started yet is `Blocked`, counting as
   neither pending nor loaded nor failed. It worked only by a race — every
   container used to spawn within 100 ms — and pacing made the race lose every
   time, printing `Startup complete: all nodes ready (nodes 12/44, containers
   0/16, composable 0/84)` two tenths of a second in, then falling silent
   because the progress task exits on completion. Pre-existing; would misreport
   on any slow-starting container.
4. **play_launch 0.9.0 could not parse the launch file at all.** `<choice>` is
   a child entity of `<arg>` in real ROS 2
   (`declare_launch_argument.py:176` reads it as
   `get_attr('choice', data_type=List[Entity])`), but `attr_spec.rs` gave `arg`
   no children, so the YAML frontend saw an unknown attribute. A regression
   against the 0.5.1 the project runs today. Fixed; the choice LIST is still
   not enforced, which is laxness rather than a parse divergence.

## The fix that did not work

The obvious inference is that the storm is contention and that pacing the
spawns will fix it. It was implemented (`execution/startup_governor.rs`) and
**measured to be wrong**. Same binary, same launch:

| | pacing off | 12 at a time |
|---|---|---|
| spawn → startup complete | 10.6 s | 23.8 s |
| peak runnable tasks | 525 | 471 |
| peak load1 | 181 | 184 |
| MemAvailable consumed | 3.8 GiB | 3.6 GiB |

Doubled the startup for about ten percent. The storm is not contention over a
fixed amount of work — it *is* the work; those processes cost what they cost by
existing. Serialising the spawns stretches the same CPU over a longer window.

An earlier variant was worse still: a runnable-task ceiling of `2 * ncpu`
cannot distinguish "busy because we are starting things" from "busy because the
things we started are running", so nodes already up held `procs_running` above
the ceiling and nothing further could be admitted until the 30 s bypass fired
each time — startup unfinished at 190 s with peak load1 **217 against the
unpaced 183**, i.e. worse than doing nothing.

Both are now off by default, with the measurements recorded at the fields they
govern so the defaults are not re-derived from first principles by the next
reader.

## The lever that does work

Both arms below are the phase-61 binary with its shipped defaults, so the
completion figures are comparable (the pre-fix detector could not report
completion at all — finding 3).

| | `isolated` (default) | `observable` |
|---|---|---|
| processes | 144 | 60 |
| spawn → startup complete | 10.7 s | **8.4 s** |
| composables loaded | 84/84 | 84/84 |
| peak load1 | 190 | **45** |
| peak runnable tasks | 506 | **128** |
| MemAvailable consumed | 3.5 GiB | **1.4 GiB** |
| CPU during startup¹ | 10.2 of 12 cores | 3.9 cores |
| CPU in steady state¹ | 4.0 cores | 2.8 cores |

¹ from the earlier pre-fix pair, which is where per-process attribution was
sampled; the ratio is what matters and the load/memory columns above confirm it
on the shipped binary.

Same system up, **2.6x less CPU, 4x less peak load, 2.5x less memory — and
faster**.

**But this is not a recommendation, and the first draft of this document was
wrong to read it as one.** The cost is the documented price of fault isolation
(`docs/design/container-isolation.md`), not waste:

- **A SIGSEGV cannot be contained inside a process.** In a shared container one
  segfaulting composable takes down every node loaded beside it — 3–10 in an
  Autoware container — and cascades from there. That document surveys the
  alternatives (signal handler + `pthread_exit`, `siglongjmp`, seccomp-bpf) and
  concludes every production system that isolates native-code crashes uses
  processes.
- **`oom_score_adj` is process-only** — so the per-node OOM bias W1 added is
  per-*node* under `isolated` and per-*container* under `observable`. The
  cheaper mode is the one that makes this phase's own OOM protection coarser.
- **`memory` cgroups, `kill(pid)` and per-node restart are process-only too.**
- `observable` additionally gives up intra-process **zero-copy IPC**, put at
  ~1–5 ms per pipeline stage for LiDAR/camera paths.

So the mode is a **safety decision, not a performance one**: `observable` on a
bench, `isolated` on a vehicle. play_launch reports the cost and names what the
alternative forfeits; it does not downgrade anything.

The real lever for a vehicle that cannot afford the full cost is to reduce *what
has to be isolated* — per-node rather than per-run granularity — which
play_launch does not support today. That is the W2 item below, and it is the
one that would let a vehicle keep containment and lose the storm.

## What shipped (W1)

- `execution/startup_governor.rs` — admission control with three independent
  gates, each with a bypass so a launch can never fail to start:
  - concurrency limit — **off by default**, see above;
  - runnable-task ceiling — **off by default**, see above;
  - `MemAvailable` floor — **on**, an ABSOLUTE 1 GiB capped at a quarter of
    RAM. Not a throughput control: it never blocks while memory is plentiful
    and only serialises admissions once memory falls through the floor, which
    is the condition that ended with a dead desktop. (This said "10% of RAM
    clamped to [512 MiB, 4 GiB]" until W3 — the percentage scheme was replaced
    during W1 because 10% gives a 64 GiB box 4 GiB it does not need and a 4 GiB
    board 512 MiB, less than two of the processes it is protecting against.
    The code changed and three descriptions of it did not: here,
    `cli/config.rs`'s `min_available_mb` doc, and the same field in layer 2's
    duplicate config struct.)
  - a separate semaphore for composable loads, because a load slot is held
    until LoadNode returns (minutes, under the existing retry budgets) and
    sharing one semaphore would let a stuck container stall every plain node.
- `oom_score_adj = +300` on every spawned child (`node_cmdline.rs` `pre_exec`,
  `PLAY_LAUNCH_OOM_SCORE_ADJ` to override). The kernel chose GNOME because it
  was the largest thing on the box and nothing said the 144 new processes were
  expendable. Raising the score needs no privilege; only lowering does, which
  is why play_launch cannot protect the desktop directly.
- `max_concurrent_load_node_spawn` made real; `delay_load_node_millis` removed
  (`post_service_ready_warmup_ms` already does what it described).
- `HealthSummary::startup_complete()` replacing the broken test.
- The `isolated`-mode advisory, fired when composables exceed `4 * ncpu`.
- `attr_spec.rs`: `<arg>` accepts `<choice>`.

## Open (W2)

- **Confirm the OOM mechanism on the vehicle.** Everything above is measured;
  the link from CPU storm to OOM is the one inference, and it needs sensors.
- **Per-node isolation granularity.** `--container-mode` is all-or-nothing for
  a whole run, so an integrator who needs containment for one perception node
  pays for it on all 84. Being able to isolate a named set — and load the rest
  as threads — is what would let a vehicle keep crash containment where it
  matters and drop the storm. This is the single most valuable follow-up.
- ~~**Start order**~~ — done, see W2 below.
- `delay_load_node_millis` still exists in layer 2's copy of the config
  (`src/ros-launch-resolve/cli/src/config.rs`), which is a duplicate of this
  struct and a place for the two to drift.

## W2 — staged startup

`execution/startup_order.rs`. Members are assigned a stage; stage 0 starts
immediately and each later stage waits until every earlier member is up. The
point is to start sensor drivers LAST, so nothing publishes into subscribers
that do not exist yet.

**Correction to the W1 write-up:** it claimed "the SystemModel already carries
the chain information needed to derive an order". It does not, for a launch like
this one. `structure.topics` is built from authored manifests
(`model_builder.rs` walks `index.topics`), not from the launch tree, and the
golf cart resolve reports **0 topics**. So automatic derivation is inert here
and the working path is explicit configuration. Both ship:

- `startup.order` — glob groups against node FQNs, in start order. Works on any
  launch.
- `startup.defer_sources` — derives a final group from the topic graph
  (publishes and never subscribes → last). Inert without manifests. Explicit
  groups win.

Off by default: reordering startup is a semantic change, and a system where
something waits on a driver being up early would break.

### A stage waits for the ROS graph, not for `spawn()`

`spawn()` returns long before a node has constructed its subscriptions, so a
spawn-counting gate would order the `fork()` calls and nothing that matters. A
member is accounted for when it appears in the graph, is `Loaded` (composable),
or is dead — that last one because a driver whose hardware is absent never
appears and must not hold up the system. The timeout names whatever is still
missing.

### The naming gap this surfaced

The first staged run sat on its full 25 s timeout reporting 16 healthy nodes as
missing. Dumping the live `ros2 node list` against the model explained it: **19
of 144 model FQNs never appear in the graph**. For a node the launch file did
not name, the model keys it by its EXECUTABLE while the node registers its
compiled-in name —

    model:  /localization/pose_twist_fusion_filter/autoware_ekf_localizer_node
    graph:  /localization/pose_twist_fusion_filter/ekf_localizer

— and the emitted cmdline confirms play_launch passes `__ns` but **no `__node`
remap**, so it never imposes the model's name. (CLAUDE.md says `exec_name` is
used as a `__node` remap; it is used as the FQN map *key*. Worth an issue —
changing node naming would move params and remaps, so it is not a side effect to
take on here.)

`NodeInstance.node_name.is_none()` is an exact discriminator for the case: 17
nodes, plus the 2 lidar drivers legitimately absent without hardware, accounts
for all 19. The gate now requires a graph match only where the model knows the
name and falls back to process-running otherwise — weaker for those, but the
strongest signal available and no worse than not staging at all.

### Measured

| | unstaged | staged |
|---|---|---|
| stage 0 up | — | 24.7 s |
| peak load1 | 201 | 212 |
| peak runnable tasks | 473 | 462 |
| MemAvailable consumed | 3.8 GiB | 3.7 GiB |

No resource benefit, as expected: **the benefit needs sensors**, and there are
none on the bench. What is measured is that the ordering works and what it
costs — the drivers start ~14 s later, because stage 0 completing means the
consumers are genuinely in the graph rather than merely spawned. On the vehicle
that is 14 s of LiDAR frames not published into nothing.

Confirming the benefit is the same open vehicle run as W1's.

## W3 — measuring the governor

W1 shipped three gates — a `MemAvailable` floor ON, a concurrency limit and a
runnable-task ceiling both OFF — and the floor's justification was that it
"never blocks until memory is actually short". None of it was tested against a
launch. W3 tests all three, on the same 12-core / 61.4 GiB AGX Orin.

Short version: **the two gates that ship off both work and both cost what W1
said they cost. The one that ships on did not fire at all** on the storm this
phase exists for. The container's own admission, measured separately, paces
exactly as designed in both modes.

Fixture: `tests/fixtures/governor_stress/`. Two nodes — `mem_hog`, which makes
a configurable amount of memory *resident* (touched, not merely allocated: a
`malloc` of 1 GiB barely moves `MemAvailable`, so a hog that only allocates
leaves the gate it is testing permanently open), and `cpu_burn`, which burns
CPU **in its constructor**, where admission can see it.

### The result: the gate does not fire on a self-inflicted storm

24 nodes, 1 GiB resident each. Three arms, and the first three rows are the
point:

| arm | floor | spawn spread | gate fired |
|---|---|---|---|
| `gate_off` | — | 0.69 s | n/a |
| `gate_on` | 40000 MiB | 0.70 s | **no** |
| `gate_auto` | 50177 MiB (available − 6 GiB) | 0.32 s | **no** |
| `gate_auto` + 10 GiB preload | 50029 MiB (**above** available) | held 30 s | **yes, all 24** |

The hogs worked: `MemAvailable` fell 55.0 → 30.6 GiB, well through every floor
tried. The gate still admitted everything.

**Because admission is far faster than allocation.** All 24 processes were
admitted in **0.32–0.70 s**; each took **1421 ms on average** to become
resident. Every admission happened before any allocation landed. The gate
behaved exactly as written — it read a `MemAvailable` that had not moved yet.

### The mechanism itself is sound

With pressure that *pre-existed* the launch — an external 10 GiB hog, floor set
above the resulting `MemAvailable` — the gate held **all 24 admissions** for the
full `max_gate_wait_secs`, then bypassed, naming the condition and the fix:

```text
startup: admitting NODE 'hog_13' despite MemAvailable 45996 MiB below floor
50029 MiB — waited 30s (raise the limit or lower the load; see startup config)
```

So the floor is not broken. Its protection is narrower than W1 claimed.

### What this means for the vehicle

**The floor protects against pre-existing pressure, not against pressure the
launch creates.** The incident this phase exists for is the second kind: 144
processes, all started by one launch, collectively exhausting the box. For the
floor to help there, admission has to span long enough that earlier nodes'
allocations register before later nodes are admitted — plausible with 144 real
nodes over ~10 s, where here 24 trivial binaries were admitted in under a
second. **Not demonstrated either way**, and it is the thing to measure on the
real stack rather than argue about.

Second, smaller: after the bypass, every held admission releases **at once**.
The protection is "delay the storm by up to `max_gate_wait_secs`", not
"stagger it". A gate that released admissions gradually as memory recovered
would be a different and probably better design; that is a proposal, not a
measurement.

### On the test being wrong first

The first three arms are recorded above rather than deleted because the null
result is the finding, and because the mistake is worth keeping: the fixture
*asserted* a condition (memory would be short during admission) instead of
*arranging* for it. The same shape as the `Built: <path>` echo in the release
build — a check that reports success without establishing the thing it claims.
What settled it was making the pressure pre-exist, which is one line of setup
and turns a null into a demonstration.

### Safety, since this is a fixture that can brick a board

Five independent guards, each verified firing rather than assumed:

1. **In-node reserve** — `mem_hog` stops short of `safety_reserve_mb` (8 GiB
   default), re-read every 1 MiB chunk. Verified: asked for 200 GiB against a
   50 GiB reserve, stopped at 6384 MiB and said why.
2. **In-node deadline** — `max_lifetime_secs` releases and exits an orphan.
   Verified at 8 s.
3. **cgroup `MemoryMax`** — a `systemd-run --user --scope` at half of RAM with
   swap disabled. The only guard the kernel enforces *at allocation time*; the
   other four poll a number that can move faster than they sample. Verified by
   disabling guard 1 under a 512M cap: the kernel killed the process, the host
   was untouched. The harness **refuses to run** if this is unavailable.
4. **External OOM guard** — `scripts/edge-storm/oom_guard.sh` on the process
   group at a 6 GiB floor, outside play_launch so it cannot alter what is
   measured.
5. **`timeout` + EXIT trap + `just abort`** — bounded wall clock, group killed
   however the recipe ends. Verified with `RUN_TIMEOUT=25`.

Across every run the box never fell below 30.6 GiB free, no hog reached its
reserve, and memory returned to 54.9 GiB with no survivors.

### The CPU gates: both work, both cost what W1 said they cost

The two gates that ship OFF were re-measured on the same box. 32 `cpu_burn`
nodes, each burning 8 s in its constructor then idling — ~2.7x oversubscription
on 12 cores.

| arm | spawn spread | all settled | peak runnable | bypasses |
|---|---|---|---|---|
| `gate_off` | 0.66 s | **8.66 s** | 82 | 0 |
| `concurrency_on` (12 = nproc) | 17.76 s | **25.76 s** | 50 | 0 |
| `runnable_on` (2x ncpu) | 9.40 s | **17.40 s** | 57 | 0 |

`max_concurrent` costs **3.0x startup** for 39% off peak runnable;
`max_runnable_factor` costs **2.0x** for 30%. No bypass fired in either arm, so
both paced by real admission control rather than by hitting their escape hatch.

The cost direction reproduces W1 and is the measurement behind both defaults.

### Two caveats that matter more than the table

**This fixture flatters the gates.** W1 measured ~10% off peak runnable on
Autoware; these arms show 30-39%. `cpu_burn` is pure CPU with nothing else, so
holding it back is maximally effective. A real node spends much of its startup
blocked on discovery, parameter loading and I/O, where delaying it saves
nothing and costs the full delay. Read 30-39% as an upper bound that a real
stack will not reach — not as a contradiction of W1's 10%.

**The `runnable_on` pathology did NOT reproduce, and its absence is explained.**
W1 measured the runnable ceiling as *worse than nothing* — peak load1 217 with
it against 183 without — because it cannot tell "busy starting things" from
"busy running the things we started", so already-running nodes held the gate
shut until the bypass fired each time. Here it is simply the cheaper of the two
gates. The difference is the workload: these burners settle after 8 s and go
idle, so the runnable count falls back on its own and the ceiling never gets
stuck. **Reproducing the pathology needs nodes that stay busy after startup**,
which `cpu_burn` deliberately is not.

So these arms confirm the gates function and confirm their cost. They can
neither confirm nor refute W1's finding that the runnable ceiling is actively
harmful — that needs a sustained-load variant of the fixture, or the real
stack.

### The container arm: the two modes are governed differently, measurably

16 composables, each sleeping 20 s in its constructor, in one container.

| mode | loads | span | pacing |
|---|---|---|---|
| `isolated` (default) | 16/16 | **20.5 s** | 12 at t+0.0 s, 4 at t+20.4 s |
| `observable` | 6/16 in 150 s | — | one per **20.0 s**, exactly serial |

`isolated` forks a `component_node` per composable and paces them with its own
spawn-worker pool: **12 = `clamp(nproc, 4, 32)`** on a 12-core box, so 16
components fall into two waves and the whole set is up in the time of two
constructors. `observable` loads them on the container's executor, and the
measured interval is 20.0 s — the constructor duration, to the tenth. At that
rate the set would take 320 s.

This is the sequential-loading behaviour `tests/fixtures/sequential_loading`
asserts as intended, now with a number against it, and it is why the global
load cap is enforced for `stock`/`observable` and disabled for `isolated`:
in the serial modes an in-flight cap bounds something real, and in `isolated`
it would bound calls that return in milliseconds.

One nuance worth recording against the design note. `composable-load-admission.md`
says the spawn pool is "sized out of the way" so it is not the limit — here it
*was* the limit, because 16 components exceed a 12-worker pool on a 12-core
board. Memory never came near gating (53 GiB free throughout). "Out of the way"
holds for pools versus machine size, not for a component count above `nproc`.

Two W1/#0019 fixes are visible in the same run, doing what they were written to
do:

```text
container:   4 composable(s) still constructing; longest 'composable:/slow_06' at 30s
play_launch: composable 12/16 loaded (4 pending)
play_launch: Startup complete: all nodes ready (composable 16/16)
```

Progress reported during a long construction rather than silence; the
intermediate count honest at 12/16; and `Startup complete` printed only after
all 16 were ListNodes-confirmed, not on the LoadNode responses that would have
claimed 16/16 immediately.

### A fixture bug the parser caught

`container_wave.launch.xml` failed to parse on its first run:

```text
XML parsing error: comment at 2:1 contains '--'
```

The fixture was wrong, not the parser — an XML comment may not contain `--`,
and the comment described `--container-mode`. Recorded because the reflex on
seeing a parser reject your file is to suspect the parser, and here the right
move was to check the XML spec and fix the fixture. All three launch files are
now validated with a real XML parser rather than by whether they happened to
load.

### Open

- **Does the floor bind on a real 144-node launch?** The one question W3 could
  not answer with synthetic nodes.
- **Gradual release after the bypass**, instead of releasing every held
  admission simultaneously.
- The `preload` arm exists only as a hand-run sequence; making it a recipe
  would make the working case reproducible.
- **A sustained-load variant of `cpu_burn`** — one that stays busy after
  construction — is what would let the runnable-ceiling pathology be
  reproduced or refuted on the bench instead of only on a vehicle.
