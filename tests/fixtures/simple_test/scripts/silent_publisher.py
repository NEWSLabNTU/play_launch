#!/usr/bin/env python3
"""A node that CREATES a publisher and never publishes on it.

The fixture for issue #0047. The message type used to reach disk only through
`stats_summary.json` / `frontier_summary.json`, which are keyed by traffic: a
topic appears there once a message crossed it. So an endpoint like this one --
created, advertised, discoverable by `ros2 topic info`, and silent -- had no
type anywhere, and `play_launch contract capture` could not emit it at all.
Phase 77 measured how common the shape is: 982 endpoints created, 63 carrying
a message, on one Autoware run.

It also subscribes, silently, so both init hooks are exercised.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32


def main():
    rclpy.init()
    node = Node("silent_pub")
    # Deliberately kept alive and deliberately never used: an endpoint is
    # created at init, which is the whole point.
    node.create_publisher(String, "/silent_topic", 10)
    node.create_subscription(Int32, "/silent_input", lambda _msg: None, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
