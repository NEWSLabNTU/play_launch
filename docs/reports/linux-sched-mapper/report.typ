#import "@preview/fletcher:0.5.1" as fletcher: diagram, node, edge

#set page(paper: "a4", margin: (x: 2.0cm, y: 2.0cm), numbering: "1")
#set text(font: "DejaVu Sans", size: 9.5pt)
#set par(justify: true, leading: 0.62em)
#show heading.where(level: 1): it => block(above: 1.4em, below: 0.7em)[
  #set text(size: 14pt, weight: "bold"); #it.body
]
#show heading.where(level: 2): it => block(above: 1.1em, below: 0.5em)[
  #set text(size: 11pt, weight: "bold"); #it.body
]
#show raw.where(block: false): it => box(
  fill: luma(240), inset: (x: 3pt, y: 0pt), outset: (y: 3pt), radius: 2pt, it,
)
#let key(t) = text(weight: "bold", t)
#let note(title, body) = block(
  width: 100%, fill: luma(246), stroke: (left: 2pt + luma(150)),
  inset: 8pt, radius: 2pt,
)[#key(title) #h(0.3em) #body]

#align(center)[
  #text(size: 18pt, weight: "bold")[From launch file to `sched_setattr`]
  #v(-0.4em)
  #text(size: 11pt)[How the Linux scheduling mapper turns three declarations into a running schedule]
  #v(0.2em)
  #text(size: 9pt, fill: luma(90))[play_launch · phase 60 · 2026-08-12]
]

#v(0.6em)

The mapper answers one question: #emph[given a launch file that says what to run,
and a contract that says what timing it needs, what scheduling parameters should
each process get on this machine?] ROS 2 has no answer: `ros2_control` hardcodes
`SCHED_FIFO` priority 50 for its controller manager, `ros2-realtime-examples`
takes `--priority` per process, and everything else is left to whoever deploys
it. Nothing derives a priority from a stated requirement.

= 1. The three inputs

Each input answers a different question, and each is owned by a different person.
That separation is the design.

#table(
  columns: (auto, 1fr, auto),
  stroke: 0.4pt + luma(200),
  inset: 6pt,
  table.header(
    [#key("Input")], [#key("Answers")], [#key("Owner")],
  ),
  [`bringup.launch.xml`\ #text(size: 8pt, fill: luma(90))[stock ROS 2, unmodified]],
  [What processes exist, and how they are wired.],
  [package author],

  [`bringup.contract.yaml`\ #text(size: 8pt, fill: luma(90))[platform-agnostic]],
  [Timing #emph[requirements]: rates, end-to-end deadlines, which topics form a
   causal chain. True on any machine.],
  [system integrator],

  [`bringup.system.posix.yaml`\ #text(size: 8pt, fill: luma(90))[per-target]],
  [Platform #emph[facts] and policy: which priority band exists, which CPUs are
   isolated, what each node #emph[costs] here.],
  [whoever owns the box],
)

#v(0.3em)

#note("The split that matters:")[
  a deadline is a #emph[requirement] and survives a change of machine, so it lives
  in the contract. A cost is a property of (code, hardware) and does not, so it
  lives in the platform file. Conflating them is the defect that left this system
  with no execution-time figure anywhere in its model — and therefore no
  response-time analysis, no reservation sizing, no feasibility. See §5.
]

= 2. Pipeline

#align(center)[
#diagram(
  spacing: (9mm, 9mm),
  node-stroke: 0.5pt,
  node-inset: 6pt,
  edge-stroke: 0.9pt,
  node((0, 0), [launch file], fill: luma(245), name: <l>),
  node((0, 1), [contract], fill: luma(245), name: <c>),
  node((0, 2), [platform file], fill: luma(245), name: <p>),

  node((1.0, 1), align(center)[#key("resolve")\ #text(size: 8pt)[derive → override → validate]],
       fill: rgb("#e8eef7"), name: <r>),
  node((2.0, 1), align(center)[`system_model.yaml`\ #text(size: 8pt)[the one artifact]],
       fill: luma(245), name: <m>),
  node((3.0, 1), align(center)[#key("up") / #key("launch")\ #text(size: 8pt)[spawn + apply]],
       fill: rgb("#e8eef7"), name: <a>),
  node((4.0, 1), align(center)[`sched_setattr`\ #text(size: 8pt)[per thread]],
       fill: rgb("#f3e8e8"), name: <s>),

  edge(<l>, <r>, "->"), edge(<c>, <r>, "->"), edge(<p>, <r>, "->"),
  edge(<r>, <m>, "->"), edge(<m>, <a>, "->"), edge(<a>, <s>, "->"),
)
]

Two properties of this shape are load-bearing:

- #key("The model is the contract between halves.") `up` reads
  `system_model.yaml` and never the platform file. Anything the apply layer
  needs must survive into the model — a lesson learned by shipping a model that
  said `SCHED_DEADLINE` with no runtime or period, which the apply layer then
  (correctly) refused.
- #key("Derivation is machine-independent; application is not.") The same model
  applied on an unprovisioned host degrades loudly rather than silently.

= 3. Deriving the schedule

== 3.1 Ranking: rate-monotonic, extended

Classical rate-monotonic says: shorter period ⇒ higher priority. With #emph[implicit
deadlines] ($D_i = T_i$, i.e. a task must finish before its next release) this is
optimal among fixed-priority policies #emph[on one CPU]. Deadline-monotonic
generalises it to $D_i <= T_i$.

The mapper sorts on one ascending quantity, so both rules coexist:

$ "rank-key"(p) = cases(
  T_p & "timer-triggered: the period" \
  D_p & "input-triggered: the declared end-to-end deadline" \
  bot & "no timing fact — not ranked at all"
) $

#text(size: 8.5pt, fill: luma(90))[$bot$ ("bottom") means #emph[absent], not zero — the
distinction matters everywhere in this design.]

A node with no timing fact is not ranked and stays best-effort. That is a
deliberate refusal to invent a number.

#note("Watch the word \"budget\".")[
  the codebase uses it for #emph[both] the ranking key above (a #emph[deadline], in
  `path_budget_ms`) and the declared execution cost of §4 (`budget_us`). They are
  different quantities, and conflating them is exactly the bug in §5. This report
  says #emph[rank key] for the first and #emph[cost] for the second.
]

== 3.2 Why rate alone is not enough

Autoware's `vehicle_cmd_gate` — which emits the actual vehicle command — and a
pure pipeline filter both declare #key("10 Hz"). Rate cannot separate them. The
contract therefore declares a #key("chain"):

```yaml
chains:
  lidar_to_brake:
    semantics: reaction
    max_latency_ms: 60          # end-to-end, sensor to actuator
    segments:
      - { scope: /, path: sample }
      - { via: /safety/scan }   # the connecting topic, named explicitly
      - { scope: /, path: detect }
      - { via: /safety/obstacles }
      - { scope: /, path: brake }
```

The `chain_aware` mapper ranks chain members #key("drain-toward-sink"): the hop
nearest the actuator preempts the ones feeding it, so work already in flight
leaves the system rather than queueing behind new work.

== 3.3 Band compression

Derived ranks are dense integers; the platform offers a band, e.g.
$[10, 40]$. Compression maps ranks onto the band #emph[order-preservingly],
collapsing adjacent ranks when the band is too narrow — never inverting, and
never merging across a criticality boundary.

Compression #emph[creates ties]. A tie under `SCHED_FIFO` means one node can
starve another, which is exactly what `SCHED_RR` exists to fix — but only if the
slice is short enough to matter:

$ "derive" "SCHED_RR" <==> Q_"RR" < min_(i in "tied") T_i $

where $Q_"RR"$ is the host's #emph[global] round-robin slice
(`/proc/sys/kernel/sched_rr_timeslice_ms`, default #key("100 ms")). At that
default, two tied 10 Hz nodes get a slice as long as their whole period, so RR
degenerates to FIFO while #emph[looking] like it fixed starvation. The mapper
declines and says so.

= 4. Reservations: `SCHED_DEADLINE`

Fixed priority #emph[orders] tasks; it does not #emph[bound] them. A reservation does.
Linux implements the Constant Bandwidth Server: a task gets runtime $Q$ every
period $P$, to be completed within relative deadline $D$.

$ U_i = Q_i / P_i, quad
  sum_i U_i < M dot ("sched_rt_runtime_us") / ("sched_rt_period_us") $

$M$ is the CPU count in the #emph[root domain]. The right-hand side is #key("95%") by
default, and it is #emph[shared with every] `SCHED_FIFO` #emph[thread in that domain].
Exceed it and `sched_setattr` returns `EBUSY` — admission control refusing, which
the tool reports as arithmetic rather than as "resource busy".

Derivation, with every term traceable to a declaration:

$ Q <- "budget_us (declared cost)", quad
  P <- 1 / f_"min", quad
  D <- D_"declared" "else" P $

#note("Absent is not zero.")[
  No declared budget ⇒ no reservation ⇒ the node stays `SCHED_FIFO`, and
  `check --explain` prints #emph[budget absent]. The alternative — substituting
  something plausible — is what §5 is about.
]

== 4.1 Three constraints the kernel imposes

#table(
  columns: (auto, 1fr),
  stroke: 0.4pt + luma(200),
  inset: 6pt,
  [#key("Class order")],
  [`stop > deadline > rt > fair > idle`. A reserved task preempts #emph[every]
   `SCHED_FIFO` thread regardless of priority, so a band holding both loses the
   order the mapper computed. Hence: all-or-nothing within a band.],

  [#key("Affinity")],
  [A deadline thread's CPU mask may not be narrower than its #emph[root domain] —
   the set of CPUs the scheduler balances over. So `taskset` cannot confine one;
   only an exclusive cpuset partition can, because it restricts the root domain
   itself.],

  [#key("fork(2)")],
  [A `SCHED_DEADLINE` thread cannot `fork()` (returns `EAGAIN`) unless
   reset-on-fork is set — and our default container mode forks once per
   composable node.],
)

== 4.2 The per-thread trap

Linux scheduling attributes are #key("per thread"), and a ROS node reaches about 11
threads within half a second of exec (DDS spawns most of them). Applying a
reservation to every thread multiplies it:

$ (11 times 8 "ms")/(100 "ms") = #key("88%") " of one CPU," quad "for a declared " 8% $

Admission control would either refuse outright, or grant an order of magnitude
more bandwidth than was declared. So: the thread-group leader is reserved (on a
single-threaded executor that #emph[is] the callback thread) and its siblings take
`SCHED_FIFO` beneath it.

#align(center)[
#diagram(
  spacing: (18mm, 7mm),
  node-stroke: 0.5pt,
  node-inset: 5pt,
  edge-stroke: 0.9pt,
  node((0, 0.5), align(center)[node process\ #text(size: 8pt)[11 threads]], fill: luma(245), name: <n>),
  node((1.5, 0), align(center)[leader TID\ #text(size: 8pt)[`SCHED_DEADLINE` $(Q,P,D)$]], fill: rgb("#f3e8e8"), name: <ld>),
  node((1.5, 1), align(center)[10 × DDS TIDs\ #text(size: 8pt)[`SCHED_FIFO` at derived prio]], fill: rgb("#e8eef7"), name: <sib>),
  edge(<n>, <ld>, "->"), edge(<n>, <sib>, "->"),
)
]

= 5. What the design catches

Every row below was a real defect, found by building or measuring — not a
hypothetical.

#table(
  columns: (auto, 1fr),
  stroke: 0.4pt + luma(200),
  inset: 6pt,
  table.header([#key("Failure")], [#key("What the design does now")]),

  [A #emph[deadline] used as a #emph[cost]],
  [Chain resolution filled each hop's execution time with the path's declared
   `max_latency_ms`. No true execution time existed anywhere in the model, so
   response-time analysis, reservation sizing and feasibility were all
   unreachable — while feasibility #emph[reported success]. Now cost comes only
   from a declared budget; absent one, the verdict says
   #emph[feasible on incomplete evidence]. A test fails if a deadline is ever read
   as a cost again.],

  [`SCHED_FIF0` (typo)],
  [Used to become `SCHED_OTHER` silently, dropping a node out of real-time with
   no diagnostic. Now a parse error naming the six legal values.],

  [`SCHED_OTHER` at priority 10],
  [A state Linux cannot represent — the kernel requires priority 0 for CFS
   policies. Now unrepresentable in the type: each policy variant carries only
   the parameters it has.],

  [`RESET_ON_FORK` on `SCHED_FIFO`],
  [Looks like hygiene. The kernel resets scheduling in `sched_fork()`, which runs
   for #emph[thread] creation too — so the flag stops threads created after the
   apply sweep from inheriting the policy, leaving an arbitrary subset of a node
   at `SCHED_OTHER`. Measured, then locked by a test. Set only for
   `SCHED_DEADLINE`, which requires it.],

  [`uclamp_min` on an RT task],
  [A no-op: RT tasks already default to the maximum performance point. Warned
   about rather than silently ignored.],

  [An #emph[invalid] cpuset partition],
  [Writing `partition = root` can #emph[succeed] and read back `root invalid`. A
   task there runs on the full root domain with no isolation, and
   `SCHED_DEADLINE` still succeeds — so the system looks reserved and is not.
   The readback is checked; anything unrecognised is treated as invalid.],
)

= 6. Applying it

```
resolve →  execution.tiers:
             /safety/brake_controller:
               class: real_time
               budget_us: 3000   deadline_us: 8000   period_us: 20000
               posix: { sched_class: SCHED_DEADLINE, priority: 40 }
```

```
apply  →  sched: pid 47 -> SCHED_DEADLINE runtime 3000us / deadline 8000us
                            / period 20000us overrun=false
```

The `priority: 40` on a reserved node is not the reservation's — a reservation
has none. It is what the #emph[sibling threads] get.

== Provisioning, and why the tool creates nothing

A partition must be a #key("top-level") cgroup: nested under a systemd-managed
slice it always reads back `root invalid`, because that slice holds no CPUs
exclusively. And cgroup v2 requires write access to the #emph[common ancestor] of
source and destination to migrate a process — which for a top-level slice is the
cgroup root. #key("So nothing can migrate in; a process must be started inside.")

Consequently `play_launch` creates no cgroup. It detects whether it is already
inside a valid partition and refuses clearly if not. Provisioning is one root
step, out of band.

= 7. Does it work?

Three arms, same workload, one CPU, 25 s each, warm-up discarded. 60 ms deadline.

#table(
  columns: (auto, auto, auto, auto, auto),
  stroke: 0.4pt + luma(200),
  inset: 6pt,
  align: (left, right, right, right, right),
  table.header([#key("arm")], [#key("p50")], [#key("p99")], [#key("missed")], [#key("best-effort")]),
  [RT off], [43.0 ms], [155.0 ms], [217 / 1013], [—],
  [`SCHED_FIFO`], [13.1 ms], [19.9 ms], [#key("9 / 1030")], [#key("−16%")],
  [`SCHED_DEADLINE`], [31.6 ms], [148.6 ms], [42 / 1038], [#key("−5%")],
)

#key("Fixed priority wins.") Reservations return most of the throughput fixed
priority costs, and give up most of the determinism.

The flattering explanation — budgets too small, so the server throttles — was
tested and #emph[rejected]: raising them to the largest values the declared
deadlines admit made it #emph[worse] (59 misses). What remains is a model mismatch.
CBS assumes a #key("sporadic release model") — a task wakes, consumes up to $Q$,
is replenished each $P$. An `rclcpp::spin()` loop is an event loop whose releases
are message arrivals, with no phase relationship to replenishment. A message
arriving after $Q$ is spent waits for the next #emph[period], not for a CPU. Hence
p50 improves while p99 barely moves.

#note("Why this is worth presenting:")[
  the result is negative and it is the point. Reservations are the right
  mechanism at the wrong granularity — per process, when the schedulable entity
  is the callback. Getting the benefit needs executor cooperation: designs that
  give each callback its own thread and reserve #emph[that] (ROSRT, RTSS'25, is
  the clearest example) report the opposite result. No amount of work
  #emph[outside] the process substitutes for it.
]

= 8. Try it

```bash
# derive and explain, no privilege, no processes spawned
play_launch check <pkg> <launch.xml> --sched <platform.yaml> --explain

# the two-arm experiment (RT off vs SCHED_FIFO)
cd examples/rt_av_demo && just ab

# three arms: adds SCHED_DEADLINE. Needs root — a reservation must run inside
# an exclusive cpuset partition, and only root can create one.
sudo -E just ab3
```

`check --explain` is the useful one for reading a system you did not write: it
prints the final policy for every node with #emph[provenance] — derived, overridden,
or defaulted — so a surprising priority can be traced to the declaration that
caused it.

#v(0.4em)

= Symbols

#table(
  columns: (auto, 1fr, auto, 1fr),
  stroke: none,
  inset: (x: 4pt, y: 3pt),
  [$T_i$], [period], [$Q$], [runtime (reservation budget)],
  [$D_i$], [relative deadline], [$P$], [reservation period],
  [$U_i$], [utilisation, $Q\/P$], [$M$], [CPUs in the root domain],
  [$f_"min"$], [minimum declared rate], [$bot$], [absent (≠ zero)],
)

#v(0.3em)
#text(size: 8.5pt, fill: luma(90))[
#key("root domain") — the set of CPUs the kernel balances real-time tasks over.
#key("CBS") — Constant Bandwidth Server, the algorithm Linux's deadline scheduler implements.
#key("cpuset partition") — a cgroup v2 cgroup that owns its CPUs exclusively,
creating a root domain of its own.
#key("drain-toward-sink") — ranking a chain so the hop nearest the output has the
highest priority.
]
