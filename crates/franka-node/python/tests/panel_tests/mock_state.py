"""`StateMsg` bytes for the mock owner and the tests, written through the client's own layout
(`franka_node._wire.STATE`); the panel itself only decodes."""

import numpy as np

from franka_node import _wire


def encode_state(**fields):
    """The given fields (the node's names and codes), version 1, everything else zero."""
    msg = np.zeros((), _wire.STATE)
    msg["version"] = _wire.VERSION
    for name, value in fields.items():
        msg[name] = value
    return msg.tobytes()
