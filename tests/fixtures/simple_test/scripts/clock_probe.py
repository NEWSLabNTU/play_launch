"""A /clock publisher for the Health page's time checks (I): 50 Hz from
100 s, jumps back 10 s after 3 s of wall time, and stops publishing after 6 s
while staying alive."""
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main():
    rclpy.init()
    node = Node("clock_probe")
    pub = node.create_publisher(Clock, "/clock", 10)
    start = time.monotonic()
    state = {"t": 100.0, "jumped": False}

    def tick():
        wall = time.monotonic() - start
        if wall > 6.0:
            return
        if wall > 3.0 and not state["jumped"]:
            state["jumped"] = True
            state["t"] -= 10.0
        state["t"] += 0.02
        msg = Clock()
        msg.clock.sec = int(state["t"])
        msg.clock.nanosec = int((state["t"] % 1) * 1e9)
        pub.publish(msg)

    node.create_timer(0.02, tick)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
