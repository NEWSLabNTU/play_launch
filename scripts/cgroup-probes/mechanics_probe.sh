set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup); R=/sys/fs/cgroup$C
mkdir -p "$R/sup" "$R/ctr" 2>/dev/null; echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null

echo "--- 1. can a process join by writing \"0\" (signal-safe: no pid formatting)?"
( cd /; echo 0 > "$R/ctr/cgroup.procs" 2>/dev/null && echo "  write 0: ok" || echo "  write 0: FAILED"
  own=$(awk -F: '{print $3}' /proc/self/cgroup); echo "  now in: ${own##*/}"
  # 2. a fork child of a member: does it inherit, and does cgroup.kill reach it?
  nohup sleep 40 >/dev/null 2>&1 & GC=$!
  sleep 0.3
  echo "  grandchild in: $(awk -F: '{print $3}' /proc/$GC/cgroup 2>/dev/null | sed 's#.*/##')"
  echo "$GC" > /tmp/pl_gc
  exec sleep 40 ) &
MEMBER=$!
sleep 1
echo "  ctr members: $(wc -l < "$R/ctr/cgroup.procs")"
GC=$(cat /tmp/pl_gc 2>/dev/null)

echo "--- 2. cgroup.kill reaches grandchildren (the composable case)"
echo 1 > "$R/ctr/cgroup.kill" 2>/dev/null
sleep 0.5
kill -0 "$MEMBER" 2>/dev/null && echo "  member: ALIVE" || echo "  member: killed"
kill -0 "$GC" 2>/dev/null && echo "  grandchild: ALIVE" || echo "  grandchild: killed"
echo "  members after: $(wc -l < "$R/ctr/cgroup.procs")"

echo $$ > "$R/cgroup.procs" 2>/dev/null; rmdir "$R/ctr" "$R/sup" 2>/dev/null; rm -f /tmp/pl_gc
