#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."
cd "$SCRIPT_DIR"

MAP_PATH="${MAP_PATH:-$HOME/autoware_map/sample-map-planning}"

# Get Autoware path
AUTOWARE_PATH=$(readlink -f autoware)

# Source ROS and Autoware setup
source "$AUTOWARE_PATH/install/setup.bash"
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"

echo "[Demo] Starting Autoware planning simulator..."

# Source play_launch setup
source "$SCRIPT_DIR/../../install/setup.bash"

# Start Autoware in the background
play_launch launch autoware_launch planning_simulator.launch.xml map_path:="$MAP_PATH" &
SIM_PID=$!

# Function to cleanup on exit
cleanup() {
    echo "[Demo] Shutting down..."
    if kill -0 $SIM_PID 2>/dev/null; then
        kill $SIM_PID
        wait $SIM_PID 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[Demo] Waiting 60s for system initialization..."
sleep 60

echo "[Demo] Running autonomous driving test..."
python3 scripts/test_autonomous_drive.py
TEST_STATUS=$?

if [ $TEST_STATUS -eq 0 ]; then
    echo "[Demo] Test completed successfully"
    echo "[Demo] Simulator will continue running. Stop the service to terminate."
else
    echo "[Demo] Test failed with exit code $TEST_STATUS"
    exit $TEST_STATUS
fi

# Keep simulator running
echo "[Demo] Waiting for simulator to exit..."
wait $SIM_PID
