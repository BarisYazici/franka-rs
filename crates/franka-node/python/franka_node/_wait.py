"""The one condition wait of the client."""

import time

# Longest single wait, s, so Ctrl-C is seen.
SLICE = 0.1


def wait_for(condition, poll, timeout):
    """`poll()` under `condition` until it returns something other than `None`, which is
    returned; `None` after `timeout` s (`None`: no timeout). `poll` may raise."""
    deadline = None if timeout is None else time.monotonic() + timeout
    with condition:
        while True:
            value = poll()
            if value is not None:
                return value
            remaining = SLICE if deadline is None else min(SLICE, deadline - time.monotonic())
            if remaining <= 0.0:
                return None
            condition.wait(remaining)
