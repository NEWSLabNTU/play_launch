set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup); R=/sys/fs/cgroup$C
mkdir -p "$R/sup" 2>/dev/null; echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null
mkdir -p "$R/thr" 2>/dev/null
echo threaded > "$R/thr/cgroup.type" 2>/dev/null && echo "  cgroup.type=threaded: ok ($(cat $R/thr/cgroup.type))" || echo "  threaded: FAILED"
echo "  controllers in a threaded cgroup: '$(cat $R/thr/cgroup.controllers 2>/dev/null)'"
echo "  has memory.max: $(test -e $R/thr/memory.max && echo yes || echo NO)"
echo "  has cpu.weight: $(test -e $R/thr/cpu.weight && echo yes || echo NO)"
echo "  has pids.max:   $(test -e $R/thr/pids.max && echo yes || echo NO)"
echo $$ > "$R/cgroup.procs" 2>/dev/null; rmdir "$R/thr" "$R/sup" 2>/dev/null
