#!/usr/bin/env python3
"""Graph-versus-run fixture for the Health page (W6).

`pub`: node `nm_pub` publishes /near_miss/points at 5 Hz.
`sub`: node `nm_sub` subscribes /near_miss/pointz, a one-letter typo of it,
so nothing ever publishes what it listens to.
`stranger`: node `stranger`, started by the test OUTSIDE the launch on the
same ROS_DOMAIN_ID: a node the run did not start.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    role = sys.argv[1]
    rclpy.init()
    node = Node({"pub": "nm_pub", "sub": "nm_sub"}.get(role, role))
    if role == "pub":
        pub = node.create_publisher(String, "/near_miss/points", 10)
        node.create_timer(0.2, lambda: pub.publish(String(data="x")))
    elif role == "sub":
        node.create_subscription(String, "/near_miss/pointz", lambda _m: None, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
