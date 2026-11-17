"""A process that writes down the signals it receives and ignores them.

`python3 probe.py <file>`: one line per event, `<EVENT> <CLOCK_MONOTONIC>`,
`START` first. It survives SIGINT and SIGTERM, so only a SIGKILL ends it,
which makes the launcher's whole shutdown ladder observable (#0084).
"""

import signal
import sys
import time

log = open(sys.argv[1], "a", buffering=1)


def note(sig, _frame):
    log.write(f"{signal.Signals(sig).name} {time.monotonic()}\n")


signal.signal(signal.SIGINT, note)
signal.signal(signal.SIGTERM, note)
log.write(f"START {time.monotonic()}\n")
while True:
    time.sleep(1)
