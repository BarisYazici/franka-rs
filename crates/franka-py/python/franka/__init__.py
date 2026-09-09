"""Python bindings for `franka-rs <https://github.com/BarisYazici/franka-rs>`_.

Drive a Franka FR3 (FCI v10) or Franka Emika Robot / Panda (FCI v5) from Python at
whatever rate your program runs -- a policy at 5 to 30 Hz, a script, a keyboard -- while
the 1 kHz control loop runs on a Rust thread that never touches the GIL::

    import franka

    robot = franka.Robot("192.168.0.1")       # FRANKA_REALTIME=ignore for franka-sim

    with robot.cartesian_targets(max_velocity=0.3) as arm:
        obs = arm.state().flat()              # 1-D float64, order: franka.FLAT_LAYOUT
        arm.move_by([0.01, 0.0, 0.0])         # +1 cm in x, relative to the current target
        arm.move_to([0.4, 0.0, 0.3])          # absolute, base frame, metres
        arm.follow(chunk, dt=0.02)            # (N, 3) or (N, 7) rows, one every 20 ms

Every error is a :class:`FrankaError`; a motion the robot aborted is a
:class:`ControlException` (a subclass) whose ``reason`` names the reflex.
"""

import numpy as _np

from ._franka import (
    FLAT_LAYOUT,
    FLAT_LEN,
    CartesianTargets,
    ControlException,
    FrankaError,
    Gripper,
    GripperState,
    JointTargets,
    Model,
    Robot,
    RobotState,
    __version__,
)


def rotated(quaternion, rotation_vector):
    """`quaternion` (x, y, z, w) turned by `rotation_vector` (rad, base frame): the
    composition `CartesianTargets.move_by` applies, for building `move_to` targets."""
    q = _np.asarray(quaternion, dtype=float)
    r = _np.asarray(rotation_vector, dtype=float)
    angle = _np.linalg.norm(r)
    axis = r / angle if angle > 0.0 else r
    a = _np.r_[_np.sin(angle / 2) * axis, _np.cos(angle / 2)]
    return _np.r_[a[3] * q[:3] + q[3] * a[:3] + _np.cross(a[:3], q[:3]), a[3] * q[3] - a[:3] @ q[:3]]


__all__ = [
    "rotated",
    "Robot",
    "RobotState",
    "CartesianTargets",
    "JointTargets",
    "Model",
    "Gripper",
    "GripperState",
    "FrankaError",
    "ControlException",
    "FLAT_LAYOUT",
    "FLAT_LEN",
    "__version__",
]
