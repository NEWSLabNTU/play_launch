#!/usr/bin/env bash
# Script to kill orphan ROS nodes by finding their processes
# This kills all ROS-related processes except the ROS2 daemon

set -e

echo "Finding orphan ROS node processes..."

# Get the current script's PID and parent PIDs to avoid killing ourselves
SCRIPT_PID=$$
PARENT_PIDS=$(pstree -p $SCRIPT_PID | grep -o '([0-9]\+)' | grep -o '[0-9]\+' || true)

# Function to check if a PID should be skipped
should_skip_pid() {
    local pid=$1
    # Skip daemon
    if ps -p $pid -o cmd= 2>/dev/null | grep -q "ros2.*daemon"; then
        return 0
    fi
    # Skip our own script
    if echo "$PARENT_PIDS" | grep -q "^${pid}$"; then
        return 0
    fi
    return 1
}

echo "Killing component containers..."
pkill -9 -f "component_container" 2>/dev/null || true

echo "Killing standalone ROS executables..."
pkill -9 -f "controller_node_exe" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "simple_planning_simulator" 2>/dev/null || true
pkill -9 -f "dummy_perception_publisher" 2>/dev/null || true
pkill -9 -f "dummy_diag_publisher" 2>/dev/null || true
pkill -9 -f "map_hash_generator" 2>/dev/null || true
pkill -9 -f "map_projection_loader" 2>/dev/null || true
pkill -9 -f "multi_object_tracker" 2>/dev/null || true
pkill -9 -f "map_based_prediction" 2>/dev/null || true
pkill -9 -f "shape_estimation" 2>/dev/null || true
pkill -9 -f "planning_validator" 2>/dev/null || true
pkill -9 -f "planning_evaluator" 2>/dev/null || true
pkill -9 -f "control_evaluator" 2>/dev/null || true

echo "Killing Python ROS nodes..."
pgrep -f "python3.*ros" | while read pid; do
    if ! should_skip_pid $pid; then
        cmdline=$(ps -p $pid -o cmd= 2>/dev/null || true)
        echo "  Killing Python node PID $pid: $(echo $cmdline | cut -c1-80)..."
        kill -9 $pid 2>/dev/null || true
    fi
done

echo "Killing ROS CLI processes..."
pkill -9 -f "_ros2cli_node" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true

echo "Finding and killing any other ROS processes by executable name..."
# Kill common ROS executable patterns
for pattern in \
    "web_server" \
    "pose_initializer" \
    "mrm_handler" \
    "aggregator" \
    "duplicated_node_checker" \
    "hazard_status_converter" \
    "processing_time_checker" \
    "service_log_checker" \
    "topic_state_monitor" \
    "component_state_diagnostics" \
    "initial_pose_adaptor" \
    "routing_adaptor" \
    "goal_pose_visualizer" \
    "scenario_selector" \
    "velocity_limit_selector" \
    "remaining_distance_time" \
    "traffic_light_" \
    "detected_object_feature_remover"
do
    pkill -9 -f "$pattern" 2>/dev/null || true
done

echo "Waiting for processes to terminate..."
sleep 2

echo "Restarting ROS2 daemon to clear stale registrations..."
ros2 daemon stop 2>/dev/null || true
sleep 2
ros2 daemon start

echo "Waiting for daemon to stabilize..."
sleep 3

echo ""
echo "Current node count:"
NODE_COUNT=$(ros2 node list 2>&1 | grep -v "WARNING" | wc -l)
echo "$NODE_COUNT nodes remaining"

if [ $NODE_COUNT -gt 0 ]; then
    echo ""
    echo "Remaining nodes:"
    ros2 node list 2>&1 | grep -v "WARNING" | head -20
fi

echo ""
echo "Done!"
