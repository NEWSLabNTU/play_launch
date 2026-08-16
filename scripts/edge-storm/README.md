# Startup-storm measurement harness

Four small tools for answering "what is this launch doing to the machine while
it comes up?" They read only `/proc`, so they keep working while the box is
thrashing and they do not need play_launch to be healthy — which matters,
because the interesting runs are the ones where it is not.

Written for phase-61 ([the edge startup
storm](../../docs/roadmap/phase-61-edge-startup-storm.md)) and kept because the
one part of that diagnosis which is *inferred* rather than measured — the path
from CPU storm to OOM kill — needs a run on a vehicle with sensors attached to
confirm.

play_launch's own `system_stats.csv` samples at 1 s by default, which is coarser
than the transient being measured; these sample at 100 ms.

## `memwatch.sh <out.csv> [interval]`

System pressure at 100 ms: `MemAvailable`, swap, dirty pages, `Committed_AS`,
loadavg, `procs_running`/`procs_blocked`, process count, and the cumulative
`pgmajfault`/`pswpin`/`pswpout` counters.

```sh
scripts/edge-storm/memwatch.sh run.csv 0.1 &
MEMWATCH=$!
# ... launch ...
kill $MEMWATCH
```

## `cpuwatch.py <out.csv> [interval]`

Per-process attribution: one row per (sample, pid) with CPU-time delta, thread
count, RSS and state, for every process above 1% CPU or growing by more than
1 MiB.

**Read the timestamps, not the nominal interval.** Under a real storm this
sampler is starved too — a requested 500 ms stretched to 2.5 s in the phase-61
runs — so weight each row by the actual gap to the next sample. Assuming the
nominal interval undercounted total CPU by a factor of five on the first pass.

## `oom_guard.sh <pgid> <floor_kb> <log>`

Kills a process group when `MemAvailable` falls below a floor. The point of a
storm experiment is to watch memory drain; the point of this is to stop it
finishing the job, since the kernel's choice of victim on the machine running
the experiment is usually the session running the experiment.

External to play_launch on purpose: it does not change what is being measured,
it only ends the run early at a recorded moment.

## `analyze_repro.py <dir> [<dir>...]`

Summarises one or more `memwatch.csv` traces: how far `MemAvailable` fell, how
fast at its steepest sustained (1 s) rate, when the trough was, peak load and
runnable tasks, swap, and major faults. Prints a table rather than a plot,
because the numbers are what get compared between runs and a plot cannot be
diffed.

Accepts either a directory containing `memwatch.csv` or the CSV itself.

## Putting it together

```sh
scripts/edge-storm/memwatch.sh out/memwatch.csv 0.1 &
python3 scripts/edge-storm/cpuwatch.py out/cpuwatch.csv 0.5 &

setsid play_launch launch <pkg> <launch_file> >out/launch.log 2>&1 &
LAUNCH=$!
scripts/edge-storm/oom_guard.sh "$LAUNCH" $((6*1024*1024)) out/launch.log &

sleep 180
kill -TERM -"$LAUNCH"     # the process group, never individual pids

python3 scripts/edge-storm/analyze_repro.py out
```
