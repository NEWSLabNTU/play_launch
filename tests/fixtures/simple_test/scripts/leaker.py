"""Grows steadily: 2 MiB of memory and one open file every 0.4 s, touched
so the pages count. For the Health page's resource trends (K), run with a
short PLAY_LAUNCH_RESOURCE_TREND_WINDOW_S."""
import os
import time

held = []
files = []
# Files go in the working directory, the test's own temporary one.
d = os.getcwd()
i = 0
while True:
    b = bytearray(2 << 20)
    b[::4096] = b"x" * len(b[::4096])
    held.append(b)
    files.append(open(os.path.join(d, f"f{i}"), "w"))
    i += 1
    time.sleep(0.4)
