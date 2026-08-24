set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup); R=/sys/fs/cgroup$C
mkdir -p "$R/sup" "$R/probe" 2>/dev/null; echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null
echo "--- every interface file in a leaf cgroup ---"
ls "$R/probe" | sort | tr '\n' ' '; echo
echo "--- root-level controllers on this host ---"
cat /sys/fs/cgroup/cgroup.controllers
echo "--- cgroup.stat ---"; cat "$R/probe/cgroup.stat" 2>/dev/null | tr '\n' ' '; echo
echo "--- memory.events keys ---"; cat "$R/probe/memory.events" 2>/dev/null | tr '\n' ' '; echo
echo $$ > "$R/cgroup.procs" 2>/dev/null; rmdir "$R/probe" "$R/sup" 2>/dev/null
