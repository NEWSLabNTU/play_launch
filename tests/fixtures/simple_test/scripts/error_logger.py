"""Write rcutils-formatted log lines to stderr: an ERROR five times a second,
each with a different counter, plus INFO chatter.

For the Health page's log watcher: the errors are one signature with a count,
and more than ten a minute is an error burst. No ROS needed — the watcher
reads the node's err file, whatever wrote it."""
import sys
import time

n = 0
while True:
    n += 1
    stamp = f"{time.time():.9f}"
    print(f"[ERROR] [{stamp}] [error_logger]: lost connection to sensor {n}", file=sys.stderr, flush=True)
    print(f"[INFO] [{stamp}] [error_logger]: retrying in {n % 7} ms", file=sys.stderr, flush=True)
    time.sleep(0.2)
