#!/usr/bin/env bash
# Run the CLONE_VM probe once per available RMW backend and print a verdict
# table. Discovers the backends from the ament index rather than hardcoding a
# list, so a newly installed rmw (rmw_zenoh_cpp, say) is picked up with no edit.
#
#   ./run_matrix.sh [path-to-probe-binary]
set -u

PROBE="${1:-}"
if [ -z "$PROBE" ]; then
  PROBE="$(dirname "$0")/../../install/clone_vm_rmw_probe/lib/clone_vm_rmw_probe/clone_vm_rmw_probe"
fi
[ -x "$PROBE" ] || { echo "probe not found or not executable: $PROBE" >&2; exit 1; }

# Every prefix on the path, not just the first: with an overlay workspace
# sourced, the first entry is that overlay and carries no rmw at all.
BACKENDS=$(
  # %s\n, not %s: without the trailing newline `read` drops the last entry,
  # and /opt/ros/humble is exactly that entry once an overlay is sourced.
  printf '%s\n' "${AMENT_PREFIX_PATH:-/opt/ros/humble}" | tr ':' '\n' |
  while read -r prefix; do
    ls "$prefix/share/ament_index/resource_index/rmw_typesupport" 2>/dev/null
  done | grep -v '^rmw_implementation$' | sort -u
)
[ -n "$BACKENDS" ] || { echo "no rmw backends found on AMENT_PREFIX_PATH" >&2; exit 1; }

# A CycloneDDS profile from the environment changes what is being measured.
unset CYCLONEDDS_URI

printf '%-26s %-10s %-22s %s\n' BACKEND VERDICT "CHILD REACHED" DETAIL
printf '%.0s─' {1..90}; echo

for rmw in $BACKENDS; do
  out=$(RMW_IMPLEMENTATION="$rmw" timeout 60 "$PROBE" 2>&1)
  rc=$?
  # The stage strings contain spaces ("entered spin()"), so anchor on the
  # delimiters rather than stopping at the first blank.
  stage=$(printf '%s\n' "$out" | sed -n "s/.*child reached '\(.*\)'.*/\1/p" | tail -1)
  [ -n "$stage" ] || stage=$(printf '%s\n' "$out" | sed -n 's/.*child_stage=\(.*\)  *->.*/\1/p' | tail -1)

  case $rc in
    0)   verdict=PASS;    detail="clean through shutdown" ;;
    124) verdict=HANG;    detail="no progress in 60 s" ;;
    # A bare SEGV prints no symbol; naming the frame needs the gdb line below.
    139) verdict=SEGV;    detail="crashed - rerun under gdb for the frame" ;;
    *)   verdict="EXIT $rc"; detail=$(printf '%s\n' "$out" | tail -1) ;;
  esac
  printf '%-26s %-10s %-22s %s\n' "$rmw" "$verdict" "${stage:-—}" "${detail:-—}"
  printf '%s\n' "$out" > "/tmp/clone_vm_probe_${rmw}.log"
done

echo
echo "full output per backend: /tmp/clone_vm_probe_<backend>.log"
echo "for a SEGV, the frame is only named if it reached the process; run under"
echo "gdb for the stack:  RMW_IMPLEMENTATION=<b> gdb -q -batch \\"
echo "    -ex 'set follow-fork-mode child' -ex run -ex 'bt 25' --args $PROBE"
