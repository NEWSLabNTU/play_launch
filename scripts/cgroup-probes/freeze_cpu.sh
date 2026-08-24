#!/usr/bin/env bash
# Does freezing a CONSTRUCTED node actually return CPU?
#
# The whole "freeze helps a massive spawn" argument rests on this, and it is
# not obvious either way. A ROS node that has finished constructing and is
# spinning with little data spends most of its life blocked in `rcl_wait` — and
# a blocked task is already off the runqueue, so freezing it would save
# nothing. If instead a large graph keeps every participant busy with discovery
# and liveliness traffic, freezing removes real work.
#
# Run under: systemd-run --user --scope -p Delegate=yes bash tmp/freeze_cpu.sh <launch>
set -u
LAUNCH=${1:?launch file}
PL=${PL:?path to play_launch}
LOG=$(mktemp)
trap 'rm -f "$LOG"' EXIT

"$PL" launch --disable-all --container-mode isolated "$LAUNCH" >"$LOG" 2>&1 &
PLPID=$!

node_cpu() {
  local total=0 t
  for p in $(pgrep -x component_node 2>/dev/null); do
    t=$(awk '{print $14 + $15}' /proc/$p/stat 2>/dev/null || echo 0)
    total=$((total + ${t:-0}))
  done
  echo "${total:-0}"
}
runnable() { awk '/procs_running/{print $2}' /proc/stat; }

# CPU ticks consumed over a window, and the mean runnable count. Ticks are the
# honest unit here: they count work actually done, where a runnable sample only
# catches whatever happened to be on the queue at that instant.
sample() {
  local label=$1 secs=${2:-10} a b r=0 i
  a=$(node_cpu)
  for i in $(seq 1 "$secs"); do r=$((r + $(runnable))); sleep 1; done
  b=$(node_cpu)
  printf '  %-24s %6s ticks/%ss   mean_runnable %s\n' \
    "$label" "$((b - a))" "$secs" "$(awk -v x=$r -v n=$secs 'BEGIN{printf "%.1f", x/n}')"
}

# Wait for the composables to exist and settle.
for _ in $(seq 1 60); do
  n=$(pgrep -xc component_node 2>/dev/null || true); [ "${n:-0}" -gt 0 ] && break
  sleep 1
done
sleep 12
echo "  composables: $(pgrep -xc component_node 2>/dev/null || true)"

C=$(awk -F: '{print $3}' /proc/self/cgroup)
R=/sys/fs/cgroup$C
G=$(find "$R" -maxdepth 3 -type d -path '*/container/*' 2>/dev/null | head -1)
if [ -z "$G" ]; then
  echo "  no container cgroup — play_launch could not create one here"
  kill -9 $PLPID 2>/dev/null; exit 1
fi

sample "settled (running)"
echo 1 > "$G/cgroup.freeze" 2>/dev/null
sample "frozen"
echo 0 > "$G/cgroup.freeze" 2>/dev/null
sample "thawed"

kill -TERM $PLPID 2>/dev/null
sleep 3
pkill -x component_node 2>/dev/null
exit 0
