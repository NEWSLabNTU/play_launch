#!/bin/bash
# Probe which cgroup v2 features a per-logical-container design could rely on,
# unprivileged, inside a delegated scope. Run under:
#   systemd-run --user --scope -p Delegate=yes --quiet bash tmp/cgroup_probe.sh
#
# Every claim printed here is a measurement, not a reading of the docs: the
# interesting failures (migration EPERM, `partition root invalid`) are ones
# that succeed at the write and fail at the readback.
set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup)
R=/sys/fs/cgroup$C
say() { printf '  %-42s %s\n' "$1" "$2"; }

echo "== delegated scope =="
say "cgroup" "$C"
say "controllers" "$(cat "$R/cgroup.controllers" 2>/dev/null)"

# cgroup v2 forbids a cgroup that holds processes from enabling controllers for
# its children, so the supervisor has to step down into a leaf of its own first.
mkdir -p "$R/supervisor" "$R/ctr_a" "$R/ctr_b" 2>/dev/null
echo $$ > "$R/supervisor/cgroup.procs" 2>/dev/null \
  && say "supervisor steps into leaf" "ok" || say "supervisor steps into leaf" "FAILED"
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null
say "subtree_control" "$(cat "$R/cgroup.subtree_control" 2>/dev/null)"

echo
echo "== per-container files that exist =="
for f in cgroup.kill cgroup.freeze cgroup.type cgroup.events \
         memory.max memory.high memory.min memory.oom.group memory.events \
         memory.pressure cpu.pressure io.pressure pids.max pids.current; do
  if [ -e "$R/ctr_a/$f" ]; then say "$f" "present"; else say "$f" "ABSENT"; fi
done

echo
echo "== writability (the part that decides the design) =="
for f in memory.max memory.high memory.oom.group pids.max cgroup.freeze; do
  case $f in
    memory.max|memory.high) v=64M ;;
    memory.oom.group|cgroup.freeze) v=1 ;;
    pids.max) v=64 ;;
  esac
  if echo "$v" > "$R/ctr_a/$f" 2>/dev/null; then
    say "write $f=$v" "ok, reads back '$(cat "$R/ctr_a/$f" 2>/dev/null)'"
  else
    say "write $f=$v" "EPERM/EINVAL"
  fi
done
echo 0 > "$R/ctr_a/cgroup.freeze" 2>/dev/null   # thaw before using it

echo
echo "== PSI: stall, not free bytes =="
say "memory.pressure" "$(head -1 "$R/ctr_a/memory.pressure" 2>/dev/null)"
say "cpu.pressure" "$(head -1 "$R/ctr_a/cpu.pressure" 2>/dev/null)"

echo
echo "== freeze/thaw a live child =="
nohup sleep 60 >/dev/null 2>&1 &
P=$!
echo $P > "$R/ctr_b/cgroup.procs" 2>/dev/null \
  && say "child into ctr_b" "ok" || say "child into ctr_b" "FAILED"
echo 1 > "$R/ctr_b/cgroup.freeze" 2>/dev/null
sleep 0.3
say "state after freeze" "$(awk '/State/{print $2}' /proc/$P/status 2>/dev/null) (D=frozen)"
echo 0 > "$R/ctr_b/cgroup.freeze" 2>/dev/null
sleep 0.3
say "state after thaw" "$(awk '/State/{print $2}' /proc/$P/status 2>/dev/null)"
kill -9 $P 2>/dev/null

echo
echo "== does a child inherit the group it was forked in? =="
# The design needs this: play_launch moves itself into a container's cgroup,
# forks, and the child is already there — no migration, so no EPERM.
mkdir -p "$R/ctr_c" 2>/dev/null
(
  echo $BASHPID > "$R/ctr_c/cgroup.procs" 2>/dev/null
  nohup sleep 30 >/dev/null 2>&1 &
  GC=$!
  sleep 0.2
  own=$(awk -F: '{print $3}' /proc/$GC/cgroup 2>/dev/null)
  printf '  %-42s %s\n' "grandchild cgroup" "$own"
  kill -9 $GC 2>/dev/null
)

echo
echo "== cleanup =="
echo $$ > "$R/cgroup.procs" 2>/dev/null
for d in ctr_a ctr_b ctr_c supervisor; do rmdir "$R/$d" 2>/dev/null; done
say "removed test cgroups" "done"
