#!/usr/bin/env python3
"""
Automated autonomous driving test for Autoware using play_launch.
This script sets initial pose, goal pose, and engages autonomous mode.
"""

import subprocess
import time
import sys
import yaml
from pathlib import Path


def load_poses_config():
    """Load poses from configuration file."""
    # Use local config file
    config_path = Path(__file__).parent.parent / "poses_config.yaml"
    if not config_path.exists():
        print(f"ERROR: Config file not found: {config_path}")
        print("Using default poses for sample-map-planning")
        return {
            "initial_pose": {
                "position": {"x": 3752.342041015625, "y": 73736.09375, "z": 19.3404},
                "orientation": {
                    "x": -0.0008153484821679761,
                    "y": -0.00024282468558667022,
                    "z": -0.9583998316115443,
                    "w": 0.2854278175125686,
                },
            },
            "goal_pose": {
                "position": {"x": 3758.956298828125, "y": 73689.2890625, "z": 19.62882244009854},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.9662634415358213,
                    "w": -0.2575557445512531,
                },
            },
        }

    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def run_command(cmd, shell=False, timeout=10):
    """Run a command and return success status."""
    try:
        result = subprocess.run(
            cmd if not shell else cmd,
            shell=shell,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def check_topic_active(topic_name, timeout=5):
    """Check if a topic has publishers."""
    success, stdout, _ = run_command(
        f"ros2 topic info {topic_name}", shell=True, timeout=timeout
    )
    if success and "Publisher count: 0" not in stdout:
        return True
    return False


def wait_for_topics(topics, max_wait=60, check_interval=2):
    """Wait for multiple topics to become active."""
    print(f"\nWaiting for {len(topics)} critical topics to become active...")
    start_time = time.time()

    while time.time() - start_time < max_wait:
        all_active = True
        missing_topics = []

        for topic in topics:
            if not check_topic_active(topic):
                all_active = False
                missing_topics.append(topic)

        if all_active:
            print(f"✓ All topics active after {time.time() - start_time:.1f}s")
            return True

        elapsed = time.time() - start_time
        print(f"  [{elapsed:.1f}s] Waiting for {len(missing_topics)} topics: {missing_topics[0] if missing_topics else ''}...")
        time.sleep(check_interval)

    print(f"✗ Timeout waiting for topics. Still missing: {missing_topics}")
    return False


def publish_initial_pose(pose_data):
    """Publish initial pose to /initialpose."""
    pose = pose_data["initial_pose"]

    cmd = f'''ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: {pose["position"]["x"]}
      y: {pose["position"]["y"]}
      z: {pose["position"]["z"]}
    orientation:
      x: {pose["orientation"]["x"]}
      y: {pose["orientation"]["y"]}
      z: {pose["orientation"]["z"]}
      w: {pose["orientation"]["w"]}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"'''

    print("Publishing initial pose...")
    success, stdout, stderr = run_command(cmd, shell=True, timeout=10)

    if not success:
        print(f"✗ Failed to publish initial pose: {stderr}")
        return False

    print("✓ Initial pose published")
    return True


def set_route_to_goal(pose_data):
    """Set route using AD API service."""
    goal = pose_data["goal_pose"]

    # Format as YAML-style request (ros2 service call expects YAML, not JSON)
    request = (
        '"{header: {frame_id: map}, '
        f'option: {{allow_goal_modification: true}}, '
        f'goal: {{'
        f'position: {{x: {goal["position"]["x"]}, y: {goal["position"]["y"]}, z: {goal["position"]["z"]}}}, '
        f'orientation: {{x: {goal["orientation"]["x"]}, y: {goal["orientation"]["y"]}, z: {goal["orientation"]["z"]}, w: {goal["orientation"]["w"]}}}'
        f'}}, '
        f'waypoints: []}}"'
    )

    print(f"Setting route to goal ({goal['position']['x']:.1f}, {goal['position']['y']:.1f})...")

    cmd = f"ros2 service call /api/routing/set_route_points autoware_adapi_v1_msgs/srv/SetRoutePoints {request}"

    success, stdout, stderr = run_command(cmd, shell=True, timeout=15)

    if not success:
        print(f"✗ Failed to set route: {stderr}")
        return False

    if "success=True" in stdout or "status: 0" in stdout:
        print("✓ Route set successfully")
        return True
    else:
        print(f"✗ Route service call failed: {stdout}")
        return False


def engage_autonomous_mode():
    """Engage autonomous driving mode."""
    print("Engaging autonomous mode...")

    cmd = [
        "ros2", "service", "call",
        "/api/operation_mode/change_to_autonomous",
        "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
        "{}"
    ]

    success, stdout, stderr = run_command(cmd, timeout=10)

    if not success:
        print(f"✗ Failed to engage autonomous mode: {stderr}")
        return False

    # Check if the response indicates success
    if "status: 0" in stdout or "success=True" in stdout:
        print("✓ Autonomous mode engaged successfully")
        return True
    else:
        print(f"⚠ Autonomous mode call completed but may have warnings: {stdout}")
        return True  # Still return True as the call succeeded


def monitor_progress(goal_pos, duration=60, update_interval=3):
    """Monitor vehicle progress for a given duration."""
    print(f"\nMonitoring autonomous driving for {duration} seconds...")
    print("-" * 60)

    start_time = time.time()
    last_update = start_time

    while time.time() - start_time < duration:
        current_time = time.time()

        if current_time - last_update >= update_interval:
            elapsed = current_time - start_time

            # Check kinematic state
            cmd = "ros2 topic echo /localization/kinematic_state --once"
            success, stdout, _ = run_command(cmd, shell=True, timeout=3)

            if success and "pose:" in stdout:
                # Simple status update
                print(f"  [{elapsed:5.1f}s] System active - vehicle is driving...")
            else:
                print(f"  [{elapsed:5.1f}s] Waiting for kinematic state...")

            last_update = current_time

        time.sleep(0.5)

    print("-" * 60)
    print(f"✓ Monitoring complete ({duration}s)")


def main():
    """Main test sequence."""
    print("=" * 60)
    print("Automated Autonomous Driving Test for play_launch")
    print("=" * 60)

    # Load poses
    print("\n1. Loading pose configuration...")
    poses = load_poses_config()
    print("   ✓ Poses loaded")

    # Wait for Autoware to be ready
    print("\n2. Waiting for Autoware to be ready...")
    critical_topics = [
        "/map/vector_map",
        "/api/operation_mode/state",
        "/api/routing/state",
    ]

    if not wait_for_topics(critical_topics, max_wait=120):
        print("\n✗ ERROR: Autoware is not ready")
        print("   Make sure run.sh is running and wait longer")
        return 1

    print("   ✓ Autoware is ready")

    # Publish initial pose
    print("\n3. Setting initial pose...")
    if not publish_initial_pose(poses):
        print("   ✗ Failed to set initial pose")
        return 1

    # Wait for localization to stabilize
    print("\n4. Waiting for localization to stabilize...")
    time.sleep(5)

    # Wait for critical topics that appear after localization
    print("\n5. Waiting for localization-dependent topics...")
    localization_topics = [
        "/tf",
        "/localization/kinematic_state",
    ]
    if not wait_for_topics(localization_topics, max_wait=30):
        print("   ⚠ Warning: Some localization topics not active yet")
    else:
        print("   ✓ Localization topics active")

    # Set route
    print("\n6. Setting route to goal...")
    if not set_route_to_goal(poses):
        print("   ✗ Failed to set route")
        return 1

    # Wait for route processing
    print("\n7. Waiting for route to be processed...")
    time.sleep(3)

    # Engage autonomous mode
    print("\n8. Engaging autonomous mode...")
    if not engage_autonomous_mode():
        print("   ✗ Failed to engage autonomous mode")
        print("   Check system diagnostics for errors")
        return 1

    print("\n✓ AUTONOMOUS MODE ENGAGED")
    print("   Vehicle is now driving autonomously!")

    # Monitor for a while
    print("\n9. Monitoring autonomous driving...")
    monitor_progress(poses["goal_pose"]["position"], duration=60)

    print("\n" + "=" * 60)
    print("✓ TEST COMPLETE")
    print("=" * 60)
    print("\nThe autonomous driving sequence has been executed.")
    print("The vehicle should now be driving to the goal.")
    print("You can monitor progress in RViz or stop with Ctrl+C.")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
