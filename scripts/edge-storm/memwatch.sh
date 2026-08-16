#!/usr/bin/env bash
# High-rate system pressure sampler, independent of play_launch's own monitoring.
#
# Samples every 100 ms so the startup transient is visible; play_launch's
# system_stats.csv defaults to 1 s, which is coarser than the storm itself.
# Reads only /proc, so it stays cheap and keeps working while the machine is
# thrashing.
#
# Usage: memwatch.sh <out.csv> [interval_seconds]
set -uo pipefail

OUT="${1:?usage: memwatch.sh <out.csv> [interval]}"
INTERVAL="${2:-0.1}"

echo "unix_ms,mem_total_kb,mem_available_kb,mem_free_kb,swap_total_kb,swap_free_kb,dirty_kb,committed_kb,loadavg1,procs_running,procs_blocked,nr_procs,pgmajfault,pswpin,pswpout" > "$OUT"

# pgmajfault / pswpin / pswpout are cumulative counters; the deltas are what
# show the thrash, so record them raw and difference at analysis time.
while true; do
    now_ms=$(date +%s%3N)

    # One read of each file, parsed in awk: a per-field grep would be 8 reads
    # of /proc/meminfo per sample and the sampler itself would show up in the
    # numbers it is recording.
    read -r mt ma mf st sf dirty committed < <(
        awk '
            /^MemTotal:/     {mt=$2}
            /^MemAvailable:/ {ma=$2}
            /^MemFree:/      {mf=$2}
            /^SwapTotal:/    {st=$2}
            /^SwapFree:/     {sf=$2}
            /^Dirty:/        {d=$2}
            /^Committed_AS:/ {c=$2}
            END {print mt, ma, mf, st, sf, d, c}
        ' /proc/meminfo
    )

    read -r load1 _ _ _ _ < /proc/loadavg

    read -r running blocked < <(
        awk '/^procs_running/ {r=$2} /^procs_blocked/ {b=$2} END {print r, b}' /proc/stat
    )

    read -r majflt swpin swpout < <(
        awk '/^pgmajfault /  {m=$2}
             /^pswpin /      {i=$2}
             /^pswpout /     {o=$2}
             END {print m+0, i+0, o+0}' /proc/vmstat
    )

    nr_procs=$(ls -d /proc/[0-9]* 2>/dev/null | wc -l)

    echo "$now_ms,$mt,$ma,$mf,$st,$sf,$dirty,$committed,$load1,$running,$blocked,$nr_procs,$majflt,$swpin,$swpout" >> "$OUT"

    sleep "$INTERVAL"
done
