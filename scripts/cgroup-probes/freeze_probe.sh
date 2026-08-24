set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup); R=/sys/fs/cgroup$C
mkdir -p "$R/sup" "$R/fz" 2>/dev/null; echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
( echo $BASHPID > "$R/fz/cgroup.procs" 2>/dev/null
  exec bash -c 'i=0; while :; do i=$((i+1)); echo $i > /tmp/pl_freeze_tick; done' ) &
P=$!
sleep 1; a=$(cat /tmp/pl_freeze_tick 2>/dev/null)
echo 1 > "$R/fz/cgroup.freeze" 2>/dev/null
sleep 1
echo "  cgroup.events: $(tr '\n' ' ' < "$R/fz/cgroup.events" 2>/dev/null)"
b=$(cat /tmp/pl_freeze_tick 2>/dev/null); sleep 1; c=$(cat /tmp/pl_freeze_tick 2>/dev/null)
echo "  ticks while frozen: $b -> $c  (equal = actually stopped)"
echo 0 > "$R/fz/cgroup.freeze" 2>/dev/null
sleep 1; d=$(cat /tmp/pl_freeze_tick 2>/dev/null)
echo "  ticks after thaw:   $c -> $d  (advancing = resumed)"
echo 1 > "$R/fz/cgroup.kill" 2>/dev/null; sleep 0.3
echo $$ > "$R/cgroup.procs" 2>/dev/null; rmdir "$R/fz" "$R/sup" 2>/dev/null; rm -f /tmp/pl_freeze_tick
