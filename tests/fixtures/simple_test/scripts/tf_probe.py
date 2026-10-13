"""A /tf publisher for the Health page's TF checks (H): base_link -> lidar
at 20 Hz with a moving value, for 3 s, then silent (alive)."""
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = Node("tf_probe")
    pub = node.create_publisher(TFMessage, "/tf", 10)
    start = time.monotonic()

    def tick():
        wall = time.monotonic() - start
        if wall > 3.0:
            return
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "lidar"
        t.transform.translation.x = wall
        t.transform.rotation.w = 1.0
        pub.publish(TFMessage(transforms=[t]))

    node.create_timer(0.05, tick)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
