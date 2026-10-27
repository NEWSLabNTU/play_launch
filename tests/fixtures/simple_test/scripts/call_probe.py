#!/usr/bin/env python3
"""Service calls for the Health page's `call_unanswered` (phase 95).

`server <name> answer`: answers every std_srvs/Trigger request at once.
`server <name> silent`: takes a request and never answers it (its callback
blocks, so the executor never sends the response): a server that is alive
and deaf.
`server <name> die`: takes the first request and exits holding it: a server
that dies mid-call. Nothing restarts it, so later requests find no server.
`client <name>`: waits for the service, then sends a request every 0.5 s
without waiting for the response (`call_async`), as a send-and-poll client
does, and takes whatever responses arrive.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    role, name = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Node(f"{name}_{role}", namespace="/calls")
    if role == "server":
        mode = sys.argv[3]

        def handle(_request, response):
            if mode == "silent":
                while True:
                    time.sleep(3600)
            if mode == "die":
                os._exit(3)
            response.success = True
            return response

        node.create_service(Trigger, name, handle)
    else:
        client = node.create_client(Trigger, name)
        client.wait_for_service(timeout_sec=10.0)
        node.create_timer(0.5, lambda: client.call_async(Trigger.Request()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
