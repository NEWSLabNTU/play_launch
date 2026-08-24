set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup); R=/sys/fs/cgroup$C
mkdir -p "$R/sup" "$R/acct" 2>/dev/null; echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null
pids=""
for i in 1 2 3 4 5 6; do
  ( echo $BASHPID > "$R/acct/cgroup.procs" 2>/dev/null
    exec python3 -c "import time; time.sleep(20)" ) &
  pids="$pids $!"
done
sleep 3
sum=0
for p in $pids; do r=$(awk '/VmRSS/{print $2}' /proc/$p/status 2>/dev/null); sum=$((sum + ${r:-0})); done
cur=$(cat "$R/acct/memory.current" 2>/dev/null)
printf '  sum of per-process VmRSS : %s kB\n' "$sum"
printf '  cgroup memory.current    : %s kB\n' "$((cur/1024))"
printf '  difference               : %s kB (shared pages counted once by the cgroup)\n' "$((sum - cur/1024))"
printf '  pids.current             : %s\n' "$(cat "$R/acct/pids.current" 2>/dev/null)"
echo 1 > "$R/acct/cgroup.kill" 2>/dev/null; sleep 0.5
echo $$ > "$R/cgroup.procs" 2>/dev/null; rmdir "$R/acct" "$R/sup" 2>/dev/null
