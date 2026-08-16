#!/usr/bin/env python3
"""Attribute the startup CPU storm to processes.

memwatch.sh answers "how bad" (load, runnable tasks). This answers "who": it
samples /proc/<pid>/stat for every process every interval and writes one row
per (sample, pid) with the CPU-time deltas, thread count and RSS.

Sampling all of /proc is a few ms even with 800 processes, and it costs one
`open` per pid — cheap enough not to distort the thing being measured, which a
`top`/`ps` fork per sample would not be.

Usage: cpuwatch.py <out.csv> [interval_seconds]
"""

import os
import sys
import time

HZ = os.sysconf("SC_CLK_TCK")
PAGE = os.sysconf("SC_PAGE_SIZE")


def snapshot():
    """pid -> (comm, utime+stime ticks, num_threads, rss_bytes, state)."""
    out = {}
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/stat") as f:
                raw = f.read()
        except (OSError, IOError):
            continue
        # comm is field 2 and may contain spaces and parentheses, so split on
        # the LAST ')' rather than on whitespace.
        try:
            lp = raw.index("(")
            rp = raw.rindex(")")
        except ValueError:
            continue
        comm = raw[lp + 1:rp]
        rest = raw[rp + 2:].split()
        # rest[0] is field 3 (state); field N is rest[N-3].
        try:
            state = rest[0]
            utime = int(rest[11])   # field 14
            stime = int(rest[12])   # field 15
            threads = int(rest[17])  # field 20
            rss_pages = int(rest[21])  # field 24
        except (IndexError, ValueError):
            continue
        out[int(name)] = (comm, utime + stime, threads, rss_pages * PAGE, state)
    return out


def main():
    if len(sys.argv) < 2:
        print("usage: cpuwatch.py <out.csv> [interval]")
        return 1
    path = sys.argv[1]
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    prev = snapshot()
    prev_t = time.time()

    with open(path, "w") as f:
        f.write("unix_ms,pid,comm,cpu_percent,threads,rss_bytes,state\n")
        f.flush()
        while True:
            time.sleep(interval)
            now = snapshot()
            now_t = time.time()
            dt = now_t - prev_t
            ms = int(now_t * 1000)

            for pid, (comm, ticks, threads, rss, state) in now.items():
                old = prev.get(pid)
                # A pid that appeared this interval has no baseline; charge it
                # from zero rather than dropping it, or the storm's newest and
                # busiest processes would be exactly the ones missing.
                old_ticks = old[1] if old else 0
                cpu = (ticks - old_ticks) / HZ / dt * 100.0
                # Idle and not growing — not worth a row. The 1 MiB RSS
                # threshold keeps a process that is merely touching a page
                # here and there out of the trace while still catching the
                # ones actually allocating during startup.
                grew = old is not None and abs(rss - old[3]) > (1 << 20)
                if cpu < 1.0 and not grew and old is not None:
                    continue
                f.write(f"{ms},{pid},{comm},{cpu:.1f},{threads},{rss},{state}\n")
            f.flush()

            prev, prev_t = now, now_t

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
