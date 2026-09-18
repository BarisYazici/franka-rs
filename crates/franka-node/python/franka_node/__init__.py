"""Python client for franka-node: drive a Franka arm served over Zenoh.

    import franka_node

    node = franka_node.Node("tcp/<node-host>:7447")
    with node.arm("fr3") as arm:                     # lease token + acquire, release on exit
        with arm.cartesian_targets() as t:           # enable; stop on exit
            t.move_by([0.0, 0.0, 0.05])              # +5 cm z, relative to the goal
            state = t.wait(timeout=5.0)              # until the arm is there
        arm.gripper.move(0.04)

The lease, the keepalive, sequence numbers, timestamps and the byte layout are the client's.
"""

from importlib.metadata import PackageNotFoundError, version

from ._arm import Arm, Subscription
from ._errors import NodeError, NodeTimeout, ProtocolError, Refused, SessionEnded
from ._gripper import Gripper
from ._session import Node
from ._targets import CartesianTargets, JointTargets, rotated
from ._wire import ArmState, GripperState

try:
    __version__ = version("franka-node-client")
except PackageNotFoundError:
    __version__ = "unknown"

__all__ = [
    "Node",
    "Arm",
    "Subscription",
    "CartesianTargets",
    "JointTargets",
    "Gripper",
    "ArmState",
    "GripperState",
    "NodeError",
    "Refused",
    "NodeTimeout",
    "SessionEnded",
    "ProtocolError",
    "rotated",
    "__version__",
]
