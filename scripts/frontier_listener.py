#!/usr/bin/env python3
"""Read and print FrontierEvent datagrams from a Unix socket.

Usage:
    python3 scripts/frontier_listener.py [SOCKET_PATH]

Default socket path: /tmp/frontier.sock

Each datagram is 17 bytes (packed):
    topic_hash:    u64  (8 bytes, little-endian)
    stamp_sec:     i32  (4 bytes, little-endian)
    stamp_nanosec: u32  (4 bytes, little-endian)
    event_type:    u8   (1 byte: 0=publish, 1=take)
"""

import os
import socket
import struct
import sys

EVENT_SIZE = 17
EVENT_FMT = "<QiIB"  # u64, i32, u32, u8
EVENT_NAMES = {0: "PUB", 1: "TAKE"}


def main():
    sock_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/frontier.sock"

    # Remove stale socket file if it exists.
    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(sock_path)
    print(f"Listening on {sock_path} ...", flush=True)

    count = 0
    try:
        while True:
            data = sock.recv(256)
            if len(data) < EVENT_SIZE:
                print(f"  [short datagram: {len(data)} bytes]")
                continue

            topic_hash, sec, nsec, etype = struct.unpack(EVENT_FMT, data[:EVENT_SIZE])
            kind = EVENT_NAMES.get(etype, f"?{etype}")
            count += 1
            print(
                f"  [{count:>6}] {kind:<4}  topic=0x{topic_hash:016x}  "
                f"stamp={sec}.{nsec:09d}",
                flush=True,
            )
    except KeyboardInterrupt:
        print(f"\n{count} events received.")
    finally:
        sock.close()
        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
