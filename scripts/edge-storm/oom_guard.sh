#!/usr/bin/env bash
# Kill a process group if the machine's free memory falls below a floor.
#
# The point of the repro is to watch the startup storm drain memory. The point
# of this guard is that we do not want it to finish the job: on the vehicle the
# kernel OOM killer picked GNOME, and the same thing here would take the session
# running the experiment with it, losing the trace we came for.
#
# It is deliberately EXTERNAL to play_launch and reads only /proc, so it does
# not change the behaviour under test — it just ends the run early, at a
# recorded moment, instead of letting the kernel choose a victim.
#
# Usage: oom_guard.sh <pgid> <floor_kb> <logfile>
set -uo pipefail

PGID="${1:?usage: oom_guard.sh <pgid> <floor_kb> <log>}"
FLOOR_KB="${2:?}"
LOG="${3:?}"

while true; do
    avail=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)
    if [ -z "$avail" ]; then
        sleep 0.2
        continue
    fi
    if [ "$avail" -lt "$FLOOR_KB" ]; then
        echo "=== OOM GUARD TRIPPED $(date -Is): MemAvailable ${avail} kB < floor ${FLOOR_KB} kB — killing pgid ${PGID} ===" >> "$LOG"
        kill -TERM "-$PGID" 2>/dev/null
        sleep 3
        kill -9 "-$PGID" 2>/dev/null
        exit 10
    fi
    # 200 ms: fast enough to get ahead of the drain rates seen during startup,
    # slow enough that the guard itself is not a load.
    sleep 0.2
done
