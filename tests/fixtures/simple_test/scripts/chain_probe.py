#!/usr/bin/env python3
"""Carried-edge fixture (phase 88 C1).

`source`: publishes std_msgs/Header on /chain/raw at 20 Hz, fresh stamps.
`filter`: re-publishes each /chain/raw stamp on /chain/filtered.
`control`: re-publishes each /chain/filtered stamp on /chain/cmd, so
    raw -> filtered -> cmd is one carried chain of three topics.
`restamp`: takes /chain/raw and publishes /chain/restamped with now(): no
    carried edge, the stamp is born there.
`once`: publishes one message on /chain/mode (transient local), then stays.
`hold`: keeps the /chain/mode stamp it took and re-sends it on
    /chain/mode_out at 10 Hz, so the stamp it publishes grows old: a stale
    stamp once it is past a second.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


def main():
    role = sys.argv[1]
    rclpy.init()
    node = Node(f"chain_{role}")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def relay(src, dst, restamp=False):
        pub = node.create_publisher(Header, dst, qos)

        def on(msg):
            if restamp:
                msg.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)

        node.create_subscription(Header, src, on, qos)

    if role == "source":
        pub = node.create_publisher(Header, "/chain/raw", qos)

        def tick():
            msg = Header()
            msg.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)

        node.create_timer(0.05, tick)
    elif role == "filter":
        relay("/chain/raw", "/chain/filtered")
    elif role == "control":
        relay("/chain/filtered", "/chain/cmd")
    elif role == "restamp":
        relay("/chain/raw", "/chain/restamped", restamp=True)
    elif role == "once":
        pub = node.create_publisher(Header, "/chain/mode", latched)
        msg = Header()
        msg.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
    elif role == "hold":
        pub = node.create_publisher(Header, "/chain/mode_out", qos)
        held = []
        node.create_subscription(Header, "/chain/mode", held.append, latched)

        def tick():
            if held:
                pub.publish(held[-1])

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
