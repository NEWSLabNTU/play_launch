#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."
cd "$SCRIPT_DIR"

MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"

source ../../install/setup.bash
source autoware/install/setup.bash
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"

play_launch launch \
    --web-ui \
    --web-ui-addr 0.0.0.0 \
    --web-ui-port 8080 \
    --enable-monitoring \
    autoware_launch planning_simulator.launch.xml \
    map_path:="$MAP_PATH"
