"""The client's exceptions; plain Python, no dependency on `franka`."""


class NodeError(Exception):
    """Base of every error the client raises."""


class Refused(NodeError):
    """The node answered a verb with an error; `reason` is its text verbatim."""

    def __init__(self, verb, reason):
        super().__init__(f"{verb} refused: {reason}")
        self.verb = verb
        self.reason = reason


class NodeTimeout(NodeError):
    """No reply, no state, or a `wait` that did not finish in time."""


class SessionEnded(NodeError):
    """The session is over: stopped, or ended on the node's side (watchdog, lease lost, fault,
    someone's `stop`)."""

    def __init__(self, phase, robot_mode, has_errors):
        super().__init__(
            f"session ended: phase {phase}, robot mode {robot_mode}, errors {has_errors}"
        )
        self.phase = phase
        self.robot_mode = robot_mode
        self.has_errors = has_errors


class ProtocolError(NodeError):
    """A message of another length or wire version: the node is newer or older than the client."""
