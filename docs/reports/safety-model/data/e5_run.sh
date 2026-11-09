#!/bin/bash
# Phase 102 E5: play_launch's own cost on Autoware's planning simulator, with
# the overlay contracts' `commands` fields read (arm "fields") and without
# (arm "nofields"). Each run: 120 s, then SIGINT; the bundle's
# play_launch.log carries the per-subsystem totals.
set +u
ROOT=/home/aeon/repos/play_launch
source /opt/ros/humble/setup.bash
source $ROOT/install/setup.bash
source /opt/autoware/1.5.0/setup.bash
export PLAY_LAUNCH_INTERCEPTION_SO=$ROOT/src/play_launch_interception/target/release/libplay_launch_interception.so
PL=$ROOT/install/play_launch/lib/play_launch/play_launch
cd $ROOT/tmp/e5
for rep in 1 2; do
  for arm in fields nofields; do
    c=$ROOT/tmp/e5/contracts_$arm
    d=$ROOT/tmp/e5/$arm-$rep; rm -rf $d; mkdir -p $d; cd $d
    if [ $arm = fields ]; then export ROS_DOMAIN_ID=$((150 + rep * 2)); else export ROS_DOMAIN_ID=$((151 + rep * 2)); fi
    setsid $PL launch --contracts $c --interception on --disable-web-ui \
      autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning > run.log 2>&1 &
    pid=$!
    sleep 120
    PGID=$(ps -o pgid= -p $pid | tr -d ' ')
    kill -INT $pid; sleep 20; kill -TERM -$PGID 2>/dev/null; sleep 3; kill -9 -$PGID 2>/dev/null
    wait $pid 2>/dev/null
    grep -h "own CPU over the run" play_log/latest/play_launch.log | tail -1
    cd $ROOT/tmp/e5
  done
done
