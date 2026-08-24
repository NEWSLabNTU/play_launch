#!/usr/bin/env bash
# Phase 66 W3's blocking measurement: what does `cgroup.freeze` cost a node's
# DDS discovery?
#
# The claim W3 rests on is that freezing is STRICTLY STRONGER than phase 61
# W2's graph wait — a frozen driver cannot publish, where a spawned-but-
# unsubscribed one merely shouldn't. That holds only if a frozen participant
# survives being frozen. A frozen process runs no threads, so it sends no SPDP
# announcements; past the peers' lease duration they purge it. If the thaw does
# not restore the subscription, W3 would replace a wait that costs nothing with
# a hold that breaks the thing it was protecting.
#
# So the question is not "does freeze work" (W1 measured that: `frozen 1`, no
# progress until thawed). It is:
#
#   1. At what freeze duration do peers drop the participant?
#   2. After thaw, does the subscription re-match — and how long does it take?
#   3. Are messages lost across the freeze beyond the frozen interval itself?
#
# (1) is the threshold that decides whether staged startup can use freeze at
# all; a stage that holds a driver for 30 s is useless if peers purge at 10 s.
# (2) is the one that decides whether it is USABLE: a drop that recovers in
# 200 ms is a non-event, one that needs a fresh handshake per subscription is
# the failure mode.
#
# Run under: systemd-run --user --scope -p Delegate=yes bash tmp/freeze_discovery.sh
set -u
DUR=${1:-"2 5 10 15 30"}
C=$(awk -F: '{print $3}' /proc/self/cgroup)
R=/sys/fs/cgroup$C
mkdir -p "$R/sup" 2>/dev/null
echo $$ > "$R/sup/cgroup.procs" 2>/dev/null
echo "+memory +pids" > "$R/cgroup.subtree_control" 2>/dev/null

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-88}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

printf '%-8s %-16s %-16s %-18s %-14s %s\n' \
  "freeze" "peer_during" "peer_after" "thaw_to_first" "rcvd_frozen" "rcvd_10s_after"

for T in $DUR; do
  G="$R/talker_$T"
  mkdir -p "$G" 2>/dev/null

  # Publisher, in its own freezable cgroup.
  ( echo $BASHPID > "$G/cgroup.procs" 2>/dev/null
    exec ros2 run demo_nodes_cpp talker ) >"$WORK/talk_$T.log" 2>&1 &
  TALKER=$!
  # Subscriber, NOT frozen — it is the peer whose view we care about.
  ros2 run demo_nodes_cpp listener >"$WORK/listen_$T.log" 2>&1 &
  LISTENER=$!

  sleep 8   # let discovery settle and a few messages flow

  before=$(grep -c 'I heard' "$WORK/listen_$T.log" 2>/dev/null || true)
  echo 1 > "$G/cgroup.freeze" 2>/dev/null
  sleep "$T"

  # Does an observer still see the frozen node? This is the participant-lease
  # question: a frozen process sends no SPDP, so past the lease its peers purge
  # it from the graph.
  during=$(timeout 5 ros2 node list 2>/dev/null | grep -c '^/talker$' || true)

  echo 0 > "$G/cgroup.freeze" 2>/dev/null
  thaw_ns=$(date +%s%N)
  at_thaw=$(grep -c 'I heard' "$WORK/listen_$T.log" 2>/dev/null || true)

  # Time until the subscriber receives again. The number that decides whether a
  # staged start can thaw a driver and have it work.
  first=""
  for _ in $(seq 1 200); do
    now=$(grep -c 'I heard' "$WORK/listen_$T.log" 2>/dev/null || true)
    if [ "$now" -gt "$at_thaw" ]; then
      first=$(( ($(date +%s%N) - thaw_ns) / 1000000 ))
      break
    fi
    sleep 0.1
  done
  [ -z "$first" ] && first=">20000"

  after=$(timeout 5 ros2 node list 2>/dev/null | grep -c '^/talker$' || true)

  # Recovery quality: a 1 Hz talker should deliver ~10 in the next 10 s. Fewer
  # means rediscovery cost more than the freeze itself.
  base=$(grep -c 'I heard' "$WORK/listen_$T.log" 2>/dev/null || true)
  sleep 10
  recovered=$(( $(grep -c 'I heard' "$WORK/listen_$T.log" 2>/dev/null || true) - base ))

  # The talker publishes at 1 Hz, so ~T messages are legitimately missed while
  # frozen. Anything beyond that is loss the freeze caused rather than deferred.
  # Messages received while frozen. A frozen publisher runs no threads, so this
  # should be 0 — anything else would mean the freeze is not what it claims.
  during_freeze=$(( at_thaw - before ))

  printf '%-8s %-16s %-16s %-18s %-14s %s\n' \
    "${T}s" "$during" "$after" "${first}ms" "$during_freeze" "$recovered/10"

  kill -9 $TALKER $LISTENER 2>/dev/null
  wait $TALKER $LISTENER 2>/dev/null
  echo 1 > "$G/cgroup.kill" 2>/dev/null
  # Wait for the graph to actually empty; a leftover talker from the previous
  # arm shows up as `peer_during=2` and silently invalidates the next row.
  for _ in $(seq 1 30); do
    n=$(timeout 5 ros2 node list 2>/dev/null | grep -c '^/talker$' || true)
    [ "${n:-0}" -eq 0 ] && break
    sleep 1
  done
  rmdir "$G" 2>/dev/null
done

echo $$ > "$R/cgroup.procs" 2>/dev/null
rmdir "$R/sup" 2>/dev/null
echo
echo "peer_sees_*: 1 = the frozen node is in \`ros2 node list\`, 0 = purged"
