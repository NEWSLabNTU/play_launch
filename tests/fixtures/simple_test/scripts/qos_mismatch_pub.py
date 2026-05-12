#!/usr/bin/env python3
"""Publisher with BEST_EFFORT reliability — pairs with qos_mismatch_sub.py.

Used by the runtime-enforcement smoke test
`qos_match_runtime_fires_on_dds_incompatibility` to trigger an
`OFFERED_QOS_INCOMPATIBLE` event when the listener requests RELIABLE.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("qos_mismatch_pub")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(String, "qos_test", qos)

    def tick():
        msg = String()
        msg.data = "hello"
        pub.publish(msg)

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
