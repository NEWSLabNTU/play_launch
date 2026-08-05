// Node graph for the rt_av_demo workload, annotated with the scheduling
// parameters play_launch derives for each node.
//
// The values here are NOT hand-written: they are what `play_launch check
// --sched --explain` prints for bringup.system.posix.yaml. If the mapper's
// output ever changes, this diagram is wrong and the report's own
// `check --explain` listing will disagree with it.

#import "@preview/cetz:0.4.2"

#let c-safety = rgb("#c2410c")
#let c-mission = rgb("#a16207")
#let c-best = rgb("#6b7280")
#let c-mute = rgb("#8a857f")
#let c-line = rgb("#4b5563")

// One node box: title, scheduling class + priority, and what it costs.
#let node-box(pos, name, sched, work, accent, w: 3.2, h: 1.25) = {
  import cetz.draw: *
  rect(
    (pos.at(0) - w / 2, pos.at(1) - h / 2),
    (pos.at(0) + w / 2, pos.at(1) + h / 2),
    stroke: accent + 0.9pt,
    fill: accent.lighten(92%),
    radius: 0.08,
    name: name,
  )
  content(
    (pos.at(0), pos.at(1) + 0.30),
    text(size: 8.5pt, weight: "bold", fill: black, name),
  )
  content(
    (pos.at(0), pos.at(1) + 0.02),
    text(size: 8pt, fill: accent, weight: "bold", raw(sched)),
  )
  content(
    (pos.at(0), pos.at(1) - 0.28),
    text(size: 7.5pt, fill: c-mute, work),
  )
}

#let system-diagram = cetz.canvas(length: 1cm, {
  import cetz.draw: *

  // ---- safety chain -------------------------------------------------
  content((-6.2, 4.15), anchor: "west",
    text(size: 9pt, weight: "bold", fill: c-safety, [SAFETY]))
  content((-6.2, 3.8), anchor: "west",
    text(size: 7.5pt, fill: c-mute, [criticality: high]))

  node-box((-1.3, 4.0), "lidar_driver", "FIFO 38", [50 Hz · 2 ms], c-safety)
  node-box((3.6, 4.0), "obstacle_detector", "FIFO 39", [on msg · 8 ms], c-safety)
  node-box((8.5, 4.0), "brake_controller", "FIFO 40", [on msg · 3 ms], c-safety)

  line("lidar_driver.east", "obstacle_detector.west",
    mark: (end: ">", size: 0.18), stroke: c-line + 0.8pt)
  content((1.15, 4.88), text(size: 6.5pt, fill: c-mute, raw("scan")))

  line("obstacle_detector.east", "brake_controller.west",
    mark: (end: ">", size: 0.18), stroke: c-line + 0.8pt)
  content((6.05, 4.88), text(size: 6.5pt, fill: c-mute, raw("obstacles")))

  // the deadline the whole report is about
  line((-1.3, 3.15), (8.5, 3.15), stroke: (paint: c-safety, dash: "dashed",
    thickness: 0.8pt), mark: (start: "|", end: "|", size: 0.12))
  content((3.6, 2.92),
    text(size: 8pt, fill: c-safety, weight: "bold",
      [chain `lidar_to_brake` — 60 ms end-to-end deadline]))

  // ---- mission ------------------------------------------------------
  content((-6.2, 1.75), anchor: "west",
    text(size: 9pt, weight: "bold", fill: c-mission, [MISSION]))
  content((-6.2, 1.4), anchor: "west",
    text(size: 7.5pt, fill: c-mute, [no timing facts]))
  node-box((-1.3, 1.6), "path_planner", "OTHER 0", [10 Hz · 40 ms], c-mission)

  // ---- best effort --------------------------------------------------
  content((-6.2, -0.35), anchor: "west",
    text(size: 9pt, weight: "bold", fill: c-best, [BEST EFFORT]))
  content((-6.2, -0.7), anchor: "west",
    text(size: 7.5pt, fill: c-mute, [2 pinned, 4 by default]))

  node-box((-1.3, -0.5), "map_loader", "OTHER 0", [2 Hz · 120 ms], c-best)
  node-box((3.6, -0.5), "telemetry_logger", "OTHER 0", [30 Hz · 25 ms], c-best)
  node-box((8.5, -0.5), "4 × tool nodes", "OTHER 0", [30 Hz · 5 ms each], c-best)

  // ---- the shared resource ------------------------------------------
  rect((-6.4, -1.85), (10.2, -2.55), stroke: c-line + 0.7pt,
    fill: rgb("#f3f4f6"), radius: 0.08)
  content((1.75, -2.2),
    text(size: 8.5pt, [*one CPU* — every node above is confined to CPU 2 by
      `taskset`, so all ten compete on a single runqueue]))
})
