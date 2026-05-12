#!/usr/bin/env python3
"""Subscriber with RELIABLE reliability — pairs with qos_mismatch_pub.py.

When the publisher advertises BEST_EFFORT, DDS discovery fires
`REQUESTED_QOS_INCOMPATIBLE` on this side. rclpy installs a default
incompatible-QoS callback so `rmw_take_event` is polled automatically.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("qos_mismatch_sub")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, "qos_test", lambda _msg: None, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
