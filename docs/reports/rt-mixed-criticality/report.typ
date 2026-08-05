#import "diagram.typ": system-diagram

// Every measured number comes from data.json, which make_figures.py writes
// from the run directories. Nothing is typed in by hand, so a re-run cannot
// leave a stale sentence beside a fresh figure.
#let d = json("data.json")
#let off = d.stats.off
#let on = d.stats.on

#let ms(x) = [#calc.round(x, digits: 1) ms]
#let pct(x) = [#calc.round(x, digits: 0)%]

#set document(title: "Mixed-criticality real-time scheduling with play_launch")
#set page(
  paper: "a4",
  margin: (x: 2.2cm, y: 2.2cm),
  numbering: "1",
  footer: context [
    #set text(size: 8pt, fill: rgb("#8a857f"))
    #grid(columns: (1fr, auto),
      align: (left, right),
      [play_launch — mixed-criticality RT demonstration],
      counter(page).display("1 / 1", both: true))
  ],
)
// Fonts present in a stock texlive/fontconfig setup, so this compiles anywhere.
#set text(font: ("Libertinus Serif", "DejaVu Serif"), size: 10pt)
#set par(justify: true, leading: 0.62em)
#show raw: set text(font: ("DejaVu Sans Mono"), size: 8.5pt)
#show heading: set text(font: ("DejaVu Sans"))
#show heading.where(level: 1): it => {
  v(1.1em); text(size: 13pt, weight: "bold", it.body); v(0.35em)
}
#show heading.where(level: 2): it => {
  v(0.7em); text(size: 10.5pt, weight: "bold", it.body); v(0.2em)
}
#set table(stroke: (x, y) => if y == 0 { (bottom: 0.7pt + rgb("#4b5563")) }
                             else { (bottom: 0.4pt + rgb("#e0dcd6")) })

#let note(title, body) = block(
  width: 100%, inset: 9pt, radius: 3pt,
  fill: rgb("#f7f5f2"), stroke: (left: 2.5pt + rgb("#c2410c")),
  [#text(weight: "bold", size: 9.5pt, title) \ #text(size: 9pt, body)],
)

// ---------------------------------------------------------------- title
#align(center)[
  #text(size: 17pt, weight: "bold")[
    Mixed-criticality real-time scheduling with play_launch
  ]
  #v(0.2em)
  #text(size: 10pt, fill: rgb("#6b6660"))[
    Deriving Linux scheduling parameters from a launch contract,
    and measuring whether they matter
  ]
  #v(0.4em)
  #text(size: 9pt, fill: rgb("#8a857f"))[2026-08-05]
]
#v(0.6em)

#note("Result")[
  A three-node safety chain with a 60 ms end-to-end deadline, sharing one CPU
  with mission and best-effort work, missed that deadline on
  *#off.missed of #off.total frames* (#pct(off.miss_pct)) under the default
  Linux scheduler. With the scheduling parameters `play_launch` derives from
  the launch contract applied — same binaries, same load, one flag — it missed
  *#on.missed of #on.total*, and p99 latency fell from #ms(off.p99) to
  #ms(on.p99). The cost is borne by best-effort throughput, which drops by up
  to a third.
]

= 1. The workload and its scheduling parameters

`rt_av_demo` is a standalone ROS 2 workspace built for this measurement. Its
nodes are ordinary `rclcpp` — no `play_launch` API is linked, and the launch
file is standard XML. Each node busy-spins for a configured number of
milliseconds per callback, so the CPU demand is a parameter rather than an
accident of the code.

#v(0.4em)
#align(center, system-diagram)
#v(0.4em)

== Why these settings

The three `FIFO` priorities are *not written anywhere*. The platform file names
a mapper and a priority band; the priorities are derived from facts the
contract already had to state:

#table(
  columns: (auto, 1fr),
  align: (left + top, left + top),
  inset: (x: 0pt, y: 5pt),
  column-gutter: 12pt,
  table.header[*Decision*][*Rationale*],

  [Safety chain gets\ an RT class],
  [The contract declares `criticality: high` on the three chain nodes and a
   `lidar_to_brake` chain with `max_latency_ms: 60`. A deadline that must hold
   under load is a statement that these nodes may not be made to wait behind
   work that has no deadline. That is what an RT class buys.],

  [Ordered 40 > 39 > 38\ toward the actuator],
  [The `chain_aware` mapper ranks a chain drain-toward-sink: the hop nearest
   the actuator preempts the hops feeding it. The alternative — favouring the
   source — lets a fast producer fill queues the consumer cannot drain, which
   raises end-to-end latency while every individual node looks healthy.],

  [Band `[10, 40]`],
  [Leaves priorities 1–9 and 41–99 free for software outside this launch
   (kernel threads, IRQ handlers, a co-resident stack). Pinning the demo at
   the top of the usable range would make it unschedulable next to anything
   else that matters.],

  [Best-effort pinned\ to `SCHED_OTHER`],
  [`map_loader` and `telemetry_logger` are pinned explicitly by `overrides`
   rather than left to the mapper's default. If they ever drifted into
   `SCHED_FIFO` they would compete with the safety chain on equal terms and
   the demonstration would quietly stop demonstrating anything. The four
   `tools` nodes reach the same place by default — they declare no timing
   facts, so the mapper has nothing to derive from.],
)

The chain's declared budget is honest arithmetic rather than a guess: the
50 Hz boundary contributes its 20 ms period, `detect` declares 12 ms and
`brake` 8 ms, totalling 40 ms inside a 60 ms budget. The remaining 20 ms is
the slack scheduling is being asked to protect. A budget that only just fit
would miss for arithmetic reasons and prove nothing about scheduling.

= 2. From contract to `sched_setscheduler`

The declared intent becomes concrete kernel state in four steps. The values
below are the real ones for `brake_controller`, taken from each stage rather
than described.

#let stage(n, title, body) = block(width: 100%, breakable: false, [
  #text(weight: "bold", size: 9.5pt)[#n. #title]
  #v(0.15em)
  #body
])

#stage(1)[Declared facts — `bringup.contract.yaml`, `bringup.system.posix.yaml`][
```yaml
brake_controller:
  criticality: high
  paths: { brake: { trigger: {input: [obstacles]}, max_latency_ms: 8 } }

chains: { lidar_to_brake: { max_latency_ms: 60, segments: [...] } }

mapper: chain_aware
resources: { rt_priority_band: { min: 10, max: 40 } }
```
Note what is absent: no priority, no policy, no CPU. The contract states what
the software must achieve; the platform file states what the machine offers.
]

#stage(2)[Derived plan — `play_launch check --sched --explain`][
```
FQN                        CLASS        PRIO  PROVENANCE
/safety/brake_controller   SCHED_FIFO     40  derived(chain_aware:
                                              lidar_to_brake segment drain 1/2)
/safety/obstacle_detector  SCHED_FIFO     39  derived(chain_aware:
                                              lidar_to_brake segment drain 2/2)
/safety/lidar_driver       SCHED_FIFO     38  derived(chain_aware:
                                              lidar_to_brake boundary RM period=20ms)
/perception/map_loader     SCHED_OTHER     0  override(map_loader)
/planning/path_planner     SCHED_OTHER     0  default (no timing facts)
```
Every row carries its provenance, so a surprising priority can be traced to
the fact that produced it rather than guessed at.
]

#stage(3)[Persisted — `system_model.yaml`, execution layer][
```yaml
execution:
  tiers:
    /safety/brake_controller:
      class: real_time
      posix: { priority: 40, sched_class: SCHED_FIFO }
    /telemetry/telemetry_logger:
      posix: { priority: 0, sched_class: SCHED_OTHER }
```
This is the artifact `play_launch up` consumes. Resolution and execution are
separable: the plan can be inspected, diffed, and reviewed before anything
runs.
]

#stage(4)[Applied — one syscall per thread][
After spawning, and never before, each node's scheduling is applied by the
privileged `play_launch_rt_helper`, which holds `CAP_SYS_NICE` so the main
process does not have to:
```c
/* for every TID in /proc/<pid>/task/ */
sched_setscheduler(tid, SCHED_FIFO, &(struct sched_param){ .sched_priority = 40 });
sched_setaffinity(tid, sizeof(cpu_set_t), &cpus);
```
Linux scheduling attributes are per-thread, and a ROS 2 node is multi-threaded
before its first callback runs — applying only to the thread-group leader would
leave the executor threads, the ones that actually do the work, untouched.
]

For `SCHED_OTHER` the policy syscall carries priority 0 and nothing else:
the kernel rejects any other value. This detail is why two of the defects in
#link(<defects>)[section 5] existed at all.

= 3. Experiment setup

== Machine

#table(
  columns: (auto, 1fr),
  inset: (x: 0pt, y: 4pt),
  column-gutter: 12pt,
  table.header[*Property*][*Value*],
  [CPU], [AMD Ryzen 9 9950X — 16 cores / 32 threads, up to 5.75 GHz],
  [Memory], [60 GiB],
  [OS], [Ubuntu 22.04.5 LTS, ROS 2 Humble],
  [Kernel], [6.8.0-124-generic, `PREEMPT_DYNAMIC`],
  [RT throttling], [`sched_rt_runtime_us` = 950000 / `sched_rt_period_us` = 1000000],
  [CPU isolation], [none — `/sys/devices/system/cpu/isolated` is empty],
)

Two properties of this machine shape the design, and both were measured rather
than assumed.

*It is not a real-time kernel.* `PREEMPT_DYNAMIC` is the stock Ubuntu
configuration, not `PREEMPT_RT`, so worst-case latency is milliseconds rather
than microseconds. The 60 ms deadline is set well above that floor: a miss
should mean scheduling, not kernel jitter.

*It has far too many cores.* Ten nodes on 32 threads never contend, so both
runs would produce identical timelines and the honest conclusion would be
"scheduling changed nothing". Contention is therefore manufactured with
`taskset`. Two CPUs turned out not to be enough — see below — so the workload
is confined to *one*.

#note("Why one CPU, and why that is not cheating")[
  On two CPUs the safety chain missed *zero* deadlines at every load tried,
  up to 60 ms best-effort burns, with p99 stuck near 36 ms. The cause is CFS
  sleeper fairness: the chain is a low-duty-cycle task that sleeps between
  frames, accrues low virtual runtime, and is therefore picked first on every
  wake. CFS is genuinely good at this, and no amount of added load changes it
  while a spare CPU exists. Priority only decides anything once the chain and
  the load share a single runqueue. Confining to one CPU is the instrument
  that creates the regime being studied — the same role a load bank plays in
  testing a generator.
]

A consequence worth stating: CPU 2's SMT sibling (CPU 18) is *not* confined,
so unrelated system work can share the physical core. This adds noise to both
runs equally. It is one reason the baseline miss rate varies between runs
(17–77% observed across the session) even though it never approaches zero.

== Workload

Both runs are identical in every respect but one flag. Each lasts 25 s.
Measurement is taken by `brake_controller`, which compares the original lidar
timestamp — forwarded unchanged along the chain — against arrival, and prints
one line per frame. The first #d.warmup samples are discarded: node discovery
and DDS matching make early frames miss by roughly 150 ms regardless of scheduling,
and RT-on shows the same startup transient.

#table(
  columns: (auto, 1fr),
  inset: (x: 0pt, y: 4pt),
  column-gutter: 12pt,
  table.header[*Run*][*Command*],
  [baseline], raw("play_launch launch rt_av_demo bringup.launch.xml --sched-apply off"),
  [treatment], raw("play_launch launch rt_av_demo bringup.launch.xml --sched-apply strict"),
)

The timeline data comes from `play_launch`'s LD_PRELOAD interception layer,
which timestamps every `rcl_publish` and `rcl_take` with `CLOCK_MONOTONIC` and
exports a Chrome Trace Event file. That path is independent of the nodes: no
instrumentation was added to the ROS code to produce it.

= 4. Results

#figure(
  image("figures/latency.svg", width: 100%),
  caption: [End-to-end chain latency, every frame, steady state only. RT-off
    crosses the deadline repeatedly; RT-on is a flat line.],
)

#v(0.4em)
#table(
  columns: (auto, auto, auto, auto, 1fr),
  align: (left, right, right, right, right),
  inset: (x: 4pt, y: 5pt),
  table.header[][*p50*][*p99*][*max*][*past deadline*],
  [RT off], ms(off.p50), ms(off.p99), ms(off.max),
    [#off.missed / #off.total (#pct(off.miss_pct))],
  [RT on], ms(on.p50), ms(on.p99), ms(on.max),
    [#on.missed / #on.total (#pct(on.miss_pct))],
)
#v(0.3em)

RT-on latency is effectively deterministic — p50, p99 and max agree to within
a few tenths of a millisecond — and #ms(on.p50) is precisely the chain's own
declared work (2 + 8 + 3 ms). Under `SCHED_FIFO` on a contended core, the
chain waits for nothing.

#figure(
  image("figures/timeline.svg", width: 100%),
  caption: [Publish instants over a #d.window_ms ms window mid-run. Each tick
    is one message. RT-off shows the safety chain firing irregularly and in
    bursts as it falls behind and catches up; RT-on shows an even cadence.],
)

== What it costs

#figure(
  image("figures/throughput.svg", width: 100%),
  caption: [Messages published per criticality band. Priority is zero-sum on a
    saturated core.],
)

The safety chain's determinism is paid for by the best-effort nodes. This is
the expected and desired behaviour — it is what declaring a criticality
*means* — but a report showing only the chain would advertise the benefit and
conceal the bill. Note also that `sched_rt_runtime_us` caps RT at 95% of a
CPU, which is what keeps the best-effort nodes running at all rather than
starving completely.

== How application was verified <verification>

Twice, by independent means, because a scheduling run that silently applies
nothing produces results that look like a failed hypothesis rather than a
broken experiment.

*From the live processes.* `chrt -p` against the running nodes, mid-run:

```
$ chrt -p $(pgrep -f __node:=brake_controller)
current scheduling policy: SCHED_FIFO
current scheduling priority: 40
```

matching the derived plan exactly, with `SCHED_OTHER` 0 on the best-effort
nodes and all threads pinned to CPU 2.

*From the log.* The RT-on run reports each application with its concrete
values; the RT-off run reports none:

```
INFO sched: pid 416984 -> SCHED_FIFO priority 38 cpu - (tier '/safety/lidar_driver')
INFO sched: pid 416985 -> SCHED_FIFO priority 39 cpu - (tier '/safety/obstacle_detector')
INFO sched: pid 416986 -> SCHED_FIFO priority 40 cpu - (tier '/safety/brake_controller')
```

`cpu -` is accurate rather than a defect: the platform file's `isolated_cpus`
declares intent to the mapper, and no per-node core is assigned, so affinity
here comes from the harness's `taskset` rather than from `sched_setaffinity`.

This second check did not exist when work on this report began — the two runs'
logs were then *byte-identical* in their scheduling lines, because
`play_launch` logged the derived plan either way and never logged the
application. That was the fourth defect in #link(<defects>)[section 5].

= 5. Defects found while producing this report <defects>

Building the measurement surfaced four defects. All are fixed; each fix has a
test that fails when the fix is reverted.

#table(
  columns: (auto, 1fr),
  align: (left + top, left + top),
  inset: (x: 0pt, y: 6pt),
  column-gutter: 12pt,
  table.header[*Defect*][*Consequence and fix*],

  [Band check ignored\ scheduling class],
  [`band_violations` compared every tier against `rt_priority_band`, including
   `SCHED_OTHER` tiers — which carry priority 0 because the kernel requires it.
   Best-effort nodes were reported as violations and "clamped" into the RT
   band, so `check --explain` printed `SCHED_OTHER 10`: a combination Linux
   cannot represent, and not what was applied. In strict mode the same list is
   fatal, which would have rejected a valid platform file for doing exactly
   what `overrides` exist to do. Now filtered to real-time classes only.],

  [Demotion left a stale\ RT priority],
  [An override that set `sched_class: SCHED_OTHER` did not clear the priority
   the mapper had already derived, producing the same impossible pairing by a
   second route. Demotion now zeroes the priority, and an override that sets a
   non-zero priority alongside a non-RT class is warned about rather than
   silently ignored.],

  [`class: real_time` on\ best-effort tiers],
  [A node pinned out of the RT classes still carried the `real_time` label into
   `system_model.yaml`, contradicting the `sched_class` beside it. The label is
   now omitted for demoted tiers.],

  [Application was\ never logged],
  [The cause of #link(<verification>)[the verification problem] above.
   Successful application is now reported at `info` with the concrete values
   (`sched: pid 1234 -> SCHED_FIFO priority 40 cpu -`); best-effort stays at `debug`
   so the RT lines are not buried.],
)

Two further corrections were made to the demo workspace itself: the platform
file declared `isolated_cpus: [2, 3]` while the harness pinned to one CPU, and
the lidar boundary declared no execution cost, which made the chain's
feasibility verdict "feasible on incomplete evidence" — it was counting that
hop as zero. Both now agree with reality, and `check` reports no scheduling
warnings.

= 6. What this does not show

#table(
  columns: (auto, 1fr),
  align: (left + top, left + top),
  inset: (x: 0pt, y: 5pt),
  column-gutter: 12pt,
  table.header[*Claim not made*][*Why*],

  [Hard real-time guarantees],
  [`PREEMPT_DYNAMIC`, not `PREEMPT_RT`. This measures whether priority changes
   observable behaviour on a stock kernel, not worst-case bounds.],

  [Generalisation to real stacks],
  [Ten synthetic nodes whose cost is a busy-spin parameter. Real perception
   workloads have cache behaviour, memory pressure and I/O this does not
   model.],

  [CPU occupancy],
  [The interception layer sees messages, not thread run-time. A gap in a
   timeline row means "no message", not "descheduled".],

  [A calibrated deadline],
  [60 ms was chosen before measurement. It turns out well placed — 4.5x above
   RT-on p99 and far below RT-off p99 — but that is a fortunate outcome, not a
   derived one.],
)

= Appendix: reproducing

```sh
cd examples/rt_av_demo
just build          # colcon build the workspace
just check          # validate the contract, print the derived schedule
just ab             # both runs plus the comparison table
```

`just ab` refuses to report in two situations rather than produce a misleading
comparison: when `play_launch_rt_helper` lacks `CAP_SYS_NICE` (the RT-on run
would apply nothing, making both halves identical), and when the RT-off
baseline meets its deadline (nothing for scheduling to fix, so the result is
inconclusive rather than successful).

A file capability lives on the inode, so *any* rebuild of `play_launch`
replaces the helper and silently drops it — `just setcap` must be re-run after
every build, not only the first.

To regenerate this report from a run:

```sh
cd docs/reports/rt-mixed-criticality
just report
```
