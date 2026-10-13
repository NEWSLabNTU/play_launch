"""Die by a signal after a short run: SIGNAL env names it (SEGV, ABRT).

For the Health page's exit tracking: a node that segfaults over and over is a
crash loop, one that aborts with respawn off is down. Runs no ROS at all — a
process ending by a signal is the whole fixture."""
import os
import signal
import time

time.sleep(float(os.environ.get("AFTER_S", "1.0")))
os.kill(os.getpid(), getattr(signal, "SIG" + os.environ.get("SIGNAL", "SEGV")))
