#!/usr/bin/env python3
"""Delivery latency fixture for the Entities page and Health (W5).

`pub`: publishes std_msgs/Header at 20 Hz, each with a fresh stamp.
`fast`: takes every message and returns at once.
`slow`: sleeps 150 ms per message, so it takes one in three and its queue
(depth 10) holds messages up to ~0.5 s old: its publish-to-take latency is
far above both 100 ms and twice the 50 ms period, while `fast` stays low.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


def main():
    role = sys.argv[1]
    rclpy.init()
    node = Node(f"latency_{role}")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    if role == "pub":
        pub = node.create_publisher(Header, "latency_probe", qos)

        def tick():
            msg = Header()
            msg.stamp = node.get_clock().now().to_msg()
            msg.frame_id = "probe"
            pub.publish(msg)

        node.create_timer(0.05, tick)
    elif role == "slow":
        node.create_subscription(Header, "latency_probe", lambda _m: time.sleep(0.15), qos)
    else:
        node.create_subscription(Header, "latency_probe", lambda _m: None, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
