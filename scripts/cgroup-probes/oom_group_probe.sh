#!/bin/bash
# The one bit that chooses between container semantics and fault isolation.
#
# `memory.oom.group=1` makes the kernel kill EVERY task in the cgroup when any
# member triggers OOM — a container dying as a unit. `=0` kills only the
# offending process and leaves its neighbours running — fault isolation. Same
# grouping, same limits; one bit decides which failure model applies.
#
# Run under: systemd-run --user --scope -p Delegate=yes --quiet bash <this>
set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup)
R=/sys/fs/cgroup$C
mkdir -p "$R/sup" 2>/dev/null
echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null

run_arm() {
  local oomgroup=$1 name=$2
  local G="$R/$name"
  mkdir -p "$G" 2>/dev/null
  echo 64M > "$G/memory.max"
  echo 0    > "$G/memory.swap.max" 2>/dev/null   # force OOM rather than swap
  echo "$oomgroup" > "$G/memory.oom.group"

  # Fork both members INSIDE the group: play_launch would do the same by
  # stepping into the cgroup before fork, so no migration and no EPERM.
  ( echo $BASHPID > "$G/cgroup.procs" 2>/dev/null; exec sleep 30 ) &
  local BYSTANDER=$!
  sleep 0.4
  ( echo $BASHPID > "$G/cgroup.procs" 2>/dev/null
    # allocate and TOUCH, so pages are resident and the limit actually binds
    exec python3 -c "b=[]
while True: b.append(bytearray(4*1024*1024))" ) &
  local HOG=$!

  sleep 4
  local b="dead" h="dead"
  kill -0 $BYSTANDER 2>/dev/null && b="ALIVE"
  kill -0 $HOG 2>/dev/null && h="ALIVE"
  printf '  oom.group=%s  bystander=%-5s  hog=%-5s  oom_kill=%s\n' \
    "$oomgroup" "$b" "$h" "$(awk '/oom_kill /{print $2}' "$G/memory.events" 2>/dev/null)"

  kill -9 $BYSTANDER $HOG 2>/dev/null
  wait $BYSTANDER $HOG 2>/dev/null
  sleep 0.3
  echo 1 > "$G/cgroup.kill" 2>/dev/null
  rmdir "$G" 2>/dev/null
}

echo "== memory.oom.group: one bit, two failure models =="
run_arm 1 grp_unit      # container semantics: the group is the fault unit
run_arm 0 grp_isolated  # fault isolation: only the offender dies

echo
echo "== cgroup.kill: atomic teardown of a whole container =="
G="$R/tear"; mkdir -p "$G" 2>/dev/null
for i in 1 2 3; do ( echo $BASHPID > "$G/cgroup.procs" 2>/dev/null; exec sleep 30 ) & done
sleep 0.5
printf '  members before: %s\n' "$(wc -l < "$G/cgroup.procs")"
echo 1 > "$G/cgroup.kill"
sleep 0.5
printf '  members after cgroup.kill: %s\n' "$(wc -l < "$G/cgroup.procs")"
rmdir "$G" 2>/dev/null

echo $$ > "$R/cgroup.procs" 2>/dev/null
rmdir "$R/sup" 2>/dev/null
echo "  done"
