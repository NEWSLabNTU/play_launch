"""Drives /lc_talker through configure and, a second later, activate: an
external lifecycle manager, as nav2's is, for the startup timeline (Health
page). Exits when done."""
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("lifecycle_driver")
    client = node.create_client(ChangeState, "/lc_talker/change_state")
    while not client.wait_for_service(timeout_sec=0.5):
        pass
    for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        req = ChangeState.Request()
        req.transition.id = transition
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
