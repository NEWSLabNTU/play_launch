#!/usr/bin/env python3
"""Large-message fixture (phase 89 D2).

`pub`: publishes a 200 KiB std_msgs/String on /big/blob at 5 Hz.
`sub`: takes it in another process. Over UDP each message is larger than a
datagram, so play_launch reports the topic with the kernel's IP reassembly
settings; it changes none of them.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    role = sys.argv[1]
    rclpy.init()
    node = Node(f"big_{role}")
    if role == "pub":
        pub = node.create_publisher(String, "/big/blob", 10)
        msg = String(data="x" * (200 * 1024))
        node.create_timer(0.2, lambda: pub.publish(msg))
    else:
        node.create_subscription(String, "/big/blob", lambda _m: None, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
