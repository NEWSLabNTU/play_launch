#!/bin/bash
# Can this process own a cgroup subtree? The answer must be a MEASUREMENT, not
# a guess from the path name: `systemd-run --user --scope` without
# `-p Delegate=yes` produces a cgroup that looks identical and is not writable.
#
# Same shape as cpuset.rs's `partition root` / `root invalid` readback lesson:
# the write is not the test, the effect is.
set -u
C=$(awk -F: '{print $3}' /proc/self/cgroup)
R=/sys/fs/cgroup$C
printf 'cgroup:       %s\n' "$C"
printf 'controllers:  %s\n' "$(cat "$R/cgroup.controllers" 2>/dev/null)"

probe=$R/.play_launch_probe
if mkdir "$probe" 2>/dev/null; then
  printf 'mkdir child:  ok\n'
  if echo $$ > "$probe/cgroup.procs" 2>/dev/null; then
    printf 'move self:    ok (delegation usable)\n'
    echo $$ > "$R/cgroup.procs" 2>/dev/null
  else
    printf 'move self:    EPERM (cgroup exists but is not ours)\n'
  fi
  rmdir "$probe" 2>/dev/null
else
  printf 'mkdir child:  EPERM (no delegation — degrade to no grouping)\n'
fi
