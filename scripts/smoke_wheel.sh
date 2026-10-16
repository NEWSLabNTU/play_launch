#!/usr/bin/env bash
# Smoke-test an installed play_launch wheel against one ROS distribution
# (phase 92). Run where /opt/ros/<distro> is installed, with
# ros-<distro>-demo-nodes-cpp and ros-<distro>-composition, after
# `pip install <wheel>`.
#
#   scripts/smoke_wheel.sh <distro> [workdir]
#
# Checks, in order: the payload selected for the sourced distribution and
# with nothing sourced (the only /opt/ros/<d>); `check` on a plain launch
# file; and a real launch of a Python launch file with a container and two
# composables (composition's demo), stopped with SIGINT, which must exit 0 after startup
# completed. That last one exercises the payload's pyexec, its container,
# the play_launch_msgs typesupport the container loads, and the
# interception library's absence or presence alike.
set -euo pipefail

DISTRO=${1:?usage: smoke_wheel.sh <distro> [workdir]}
WORK=${2:-$(mktemp -d)}
STARTUP_WAIT=${SMOKE_STARTUP_WAIT:-180}
mkdir -p "$WORK"
cd "$WORK"

say() { printf '\n== %s\n' "$*"; }

say "payload, nothing sourced"
env -i HOME="$HOME" PATH="$PATH" play_launch --version | tee version.txt
grep -q "ROS $DISTRO" version.txt

# setup.bash reads unset variables
set +u
# shellcheck disable=SC1090
source "/opt/ros/$DISTRO/setup.bash"
set -u

say "payload, $DISTRO sourced"
python3 -m play_launch.payload | tee payload.txt
grep -q "^selected:  $DISTRO " payload.txt
play_launch --version | grep "ROS $DISTRO"

say "check"
cat > talker_listener.launch.xml <<'EOF'
<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
  <node pkg="demo_nodes_cpp" exec="listener" name="listener"/>
</launch>
EOF
play_launch check talker_listener.launch.xml

# Humble installs composition_demo.launch.py; Jazzy and later renamed it
# composition_demo_launch.py.
demo=$(cd "/opt/ros/$DISTRO/share/composition/launch" && ls composition_demo*.py | head -1)
say "launch $demo"
play_launch launch composition "$demo" --disable-web-ui \
  > launch.out 2>&1 &
pid=$!
ok=
early=
for _ in $(seq "$STARTUP_WAIT"); do
  if grep -q 'Startup complete' launch.out; then ok=1; break; fi
  if ! kill -0 "$pid" 2>/dev/null; then early=1; break; fi
  sleep 1
done
kill -INT "$pid" 2>/dev/null || true
rc=0
wait "$pid" || rc=$?
cat launch.out
if [[ -n "$early" ]]; then
  echo "FAIL: play_launch exited ($rc) before startup completed" >&2
  exit 1
fi
if [[ -z "$ok" ]]; then
  echo "FAIL: no 'Startup complete' within ${STARTUP_WAIT}s" >&2
  exit 1
fi
grep 'Startup complete' launch.out | grep -q 'composable 2/2' \
  || { echo "FAIL: composables did not all load" >&2; exit 1; }
if [[ $rc -ne 0 ]]; then
  echo "FAIL: play_launch exited $rc after SIGINT" >&2
  exit 1
fi
say "smoke test passed for $DISTRO"
