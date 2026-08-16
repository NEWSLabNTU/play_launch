#!/usr/bin/env python3
"""Summarise one repro run's pressure trace.

Reads tmp/repro/<label>/memwatch.csv and reports the startup transient: how
far MemAvailable fell, how fast, when the trough was relative to launch, and
whether the machine was swapping or major-faulting while it fell.

Prints a table, not a plot: the numbers are what decide whether the fix works,
and a plot cannot be diffed between runs.
"""

import csv
import sys
from pathlib import Path

KB = 1024


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: (float(v) if v not in ("", None) else 0.0) for k, v in r.items()})
    return rows


def summarize(label, rows):
    if not rows:
        print(f"{label}: empty trace")
        return

    t0 = rows[0]["unix_ms"]
    total = rows[0]["mem_total_kb"]

    avail = [r["mem_available_kb"] for r in rows]
    start_avail = avail[0]
    trough = min(avail)
    trough_i = avail.index(trough)
    trough_t = (rows[trough_i]["unix_ms"] - t0) / 1000.0

    # Steepest sustained drain over a 1 s window (10 samples at 100 ms). A
    # single-sample delta is noise; the sustained rate is what outruns reclaim.
    win = 10
    worst_rate = 0.0
    worst_at = 0.0
    for i in range(len(rows) - win):
        dt = (rows[i + win]["unix_ms"] - rows[i]["unix_ms"]) / 1000.0
        if dt <= 0:
            continue
        rate = (avail[i] - avail[i + win]) / dt  # kB/s drained
        if rate > worst_rate:
            worst_rate = rate
            worst_at = (rows[i]["unix_ms"] - t0) / 1000.0

    swap_used = [r["swap_total_kb"] - r["swap_free_kb"] for r in rows]
    majflt_delta = rows[-1]["pgmajfault"] - rows[0]["pgmajfault"]
    swpout_delta = rows[-1]["pswpout"] - rows[0]["pswpout"]
    swpin_delta = rows[-1]["pswpin"] - rows[0]["pswpin"]

    print(f"=== {label} ===")
    print(f"  duration              {(rows[-1]['unix_ms'] - t0)/1000.0:8.1f} s  ({len(rows)} samples)")
    print(f"  MemTotal              {total/KB/KB:8.1f} GiB")
    print(f"  MemAvailable start    {start_avail/KB/KB:8.1f} GiB")
    print(f"  MemAvailable trough   {trough/KB/KB:8.1f} GiB   at t+{trough_t:.1f} s")
    print(f"  consumed at trough    {(start_avail-trough)/KB/KB:8.1f} GiB")
    print(f"  steepest 1 s drain    {worst_rate/KB:8.1f} MiB/s at t+{worst_at:.1f} s")
    print(f"  peak swap used        {max(swap_used)/KB/KB:8.1f} GiB")
    print(f"  peak load1            {max(r['loadavg1'] for r in rows):8.2f}")
    print(f"  peak procs_running    {max(r['procs_running'] for r in rows):8.0f}")
    print(f"  peak procs_blocked    {max(r['procs_blocked'] for r in rows):8.0f}")
    print(f"  peak process count    {max(r['nr_procs'] for r in rows):8.0f}"
          f"   (start {rows[0]['nr_procs']:.0f})")
    print(f"  major faults          {majflt_delta:8.0f}")
    print(f"  pages swapped out/in  {swpout_delta:8.0f} / {swpin_delta:.0f}")
    print()


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_repro.py <repro-dir> [<repro-dir>...]")
        return 1
    for d in sys.argv[1:]:
        p = Path(d)
        csv_path = p / "memwatch.csv" if p.is_dir() else p
        summarize(p.name, load(csv_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
