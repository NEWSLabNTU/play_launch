#!/bin/bash
# Provision an exclusive cgroup v2 cpuset partition for SCHED_DEADLINE
# reservations, and launch a command inside it.
#
# WHY THIS EXISTS, AND WHY IT IS A SCRIPT RATHER THAN A play_launch VERB
#
# SCHED_DEADLINE refuses an affinity mask narrower than the root domain the
# thread was created on, so confining a reserved node to a CPU subset needs a
# restricted root domain — an exclusive cpuset partition. Two measured facts
# then fix the shape of everything:
#
#   1. A partition only validates when its parent owns those CPUs exclusively.
#      Nested under a systemd-managed slice (whose cpuset.cpus is blank, so it
#      holds nothing exclusively) it always reads back "root invalid". Only a
#      TOP-LEVEL cgroup — parent = the cgroup root — works.
#
#   2. cgroup v2 migration requires write access to the COMMON ANCESTOR of
#      source and destination. A top-level slice's common ancestor with a login
#      session is the cgroup root, which no unprivileged user can write, and
#      clone3(CLONE_INTO_CGROUP) inherits the same rule.
#
# Together: nothing can move itself in. A process must be STARTED inside. That
# is why this is a launcher, and why play_launch itself contains no privileged
# code — it only detects whether it is already inside a valid partition and
# refuses clearly when it is not.
#
# USAGE (as root):
#   scripts/provision_rt_cpuset.sh --cpus 2,3 --user "$SUDO_USER" -- \
#       play_launch up system_model.yaml
#
# WARNING: an exclusive partition REMOVES those CPUs from every other cgroup on
# the machine for as long as it exists. On a shared host, pick CPUs nobody else
# is relying on, and prefer testing inside a container.
set -euo pipefail

SLICE=/sys/fs/cgroup/play_launch.slice
CPUS=""
RUN_AS=""

usage() {
    sed -n '2,40p' "$0" | sed 's/^# \?//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --cpus) CPUS="$2"; shift 2 ;;
        --user) RUN_AS="$2"; shift 2 ;;
        --slice) SLICE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        --) shift; break ;;
        *) echo "unknown argument: $1" >&2; usage 1 ;;
    esac
done

[ -n "$CPUS" ] || { echo "error: --cpus is required (e.g. --cpus 2,3)" >&2; exit 1; }
[ $# -gt 0 ] || { echo "error: no command given after --" >&2; exit 1; }
[ "$(id -u)" -eq 0 ] || { echo "error: must run as root — creating a partition needs it" >&2; exit 1; }

# --- preflight, so a failure names the precondition rather than an errno ---
[ "$(stat -fc %T /sys/fs/cgroup)" = "cgroup2fs" ] || {
    echo "error: /sys/fs/cgroup is not a unified cgroup v2 hierarchy" >&2; exit 1; }
grep -qw cpuset /sys/fs/cgroup/cgroup.subtree_control || {
    echo "error: the cgroup root does not delegate 'cpuset' to its children." >&2
    echo "       enable it with: echo +cpuset > /sys/fs/cgroup/cgroup.subtree_control" >&2
    exit 1; }

# Teardown FIRST, so a re-run after a crash is clean rather than EBUSY.
if [ -d "$SLICE" ]; then
    echo "member" > "$SLICE/cpuset.cpus.partition" 2>/dev/null || true
    rmdir "$SLICE" 2>/dev/null || {
        echo "error: $SLICE exists and still holds processes:" >&2
        cat "$SLICE/cgroup.procs" >&2
        exit 1; }
fi

cleanup() {
    # Order matters, and both steps were learned by getting them wrong.
    #
    # 1. Release the CPUs FIRST. Leaving the partition active keeps those CPUs
    #    away from every other cgroup on the machine; leaving an empty
    #    directory behind is harmless by comparison. So the damaging half must
    #    succeed even if the rest fails.
    # 2. Move OURSELVES out before the rmdir. A cgroup holding any process
    #    cannot be removed (EBUSY), and this script put its own shell inside.
    echo "member" > "$SLICE/cpuset.cpus.partition" 2>/dev/null || true
    echo $$ > /sys/fs/cgroup/cgroup.procs 2>/dev/null || true
    rmdir "$SLICE" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mkdir -p "$SLICE"
echo "$CPUS" > "$SLICE/cpuset.cpus"
echo "$CPUS" > "$SLICE/cpuset.cpus.exclusive"
echo "root"  > "$SLICE/cpuset.cpus.partition"

# The readback is load-bearing, not defensive. The kernel reports an invalid
# partition by CHANGING WHAT THIS FILE SAYS, not by failing the write — and a
# task in an invalid partition runs on the full root domain, so SCHED_DEADLINE
# would succeed while delivering no isolation at all. Measured.
STATE=$(cat "$SLICE/cpuset.cpus.partition")
case "$STATE" in
    root|isolated) ;;
    *) echo "error: partition did not take effect — cpuset.cpus.partition reads '$STATE'" >&2
       echo "       a task here would run on the FULL root domain with no isolation." >&2
       exit 1 ;;
esac

echo "partition ready: $SLICE owns CPU(s) $(cat "$SLICE/cpuset.cpus.effective")"

# Enter the partition, then drop privilege. Children inherit the cgroup, so
# every node play_launch spawns is already inside — which is the entire point.
echo $$ > "$SLICE/cgroup.procs"

# Deliberately NOT `exec`. `exec` replaces this shell, which destroys the EXIT
# trap — and then nothing releases the partition, so those CPUs stay removed
# from the rest of the machine until somebody notices. (Measured: a first cut
# of this script used `exec` and left CPU 31 exclusive on a shared host.)
#
# So: run the command as a child, wait for it, and let the trap clean up.
if [ -n "$RUN_AS" ]; then
    setpriv --reuid "$RUN_AS" --regid "$RUN_AS" --init-groups --inh-caps=-all "$@" &
else
    "$@" &
fi
CHILD=$!

# Forward the signals a launcher is expected to relay, so Ctrl-C reaches the
# workload rather than orphaning it inside a cgroup we are about to tear down.
trap 'kill -TERM "$CHILD" 2>/dev/null || true' INT TERM

set +e
wait "$CHILD"
STATUS=$?
set -e
exit "$STATUS"
