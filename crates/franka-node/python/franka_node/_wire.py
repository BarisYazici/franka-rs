"""The only module that knows bytes: the node's wire layouts as numpy dtypes, the codes, and the
encoders and decoders. `crates/franka-node/tests/wire.json` pins all of it to the Rust types.

For readers coming from the docs, the `struct` strings: `<BBHIQQ7d` target,
`<BBBBHIQQQQd7d7d7d16d6d7dQQQ` state, `<BBHIQQdd` gripper command, `<BBHIQddd` gripper state.
"""

import json
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ._errors import ProtocolError

VERSION = 1
TARGET_ANCHOR = 1
FLAG_HOLDING = 1
FLAG_JOINTS = 2
GRIPPER_CALIBRATED = 1
GRIPPER_GRASPED = 2
GRIPPER_MOVING = 4
GRIPPER_FAULT = 8
KINDS = {"cartesian": 1, "joints": 2}
GRIPPER_KINDS = {"width": 1, "grasp": 2}
PHASES = ("idle", "acquired", "active", "stopping", "faulted", "homing")
ROBOT_MODES = (
    "other", "idle", "move", "guiding", "reflex", "user_stopped", "automatic_error_recovery",
)

_TYPES = {"u8": "u1", "u16": "<u2", "u32": "<u4", "u64": "<u8", "f64": "<f8"}


def _dtype(size, fields):
    """A structured dtype from `(name, type, offset, count)` rows, the layout of `wire.json`."""
    return np.dtype({
        "names": [name for name, _, _, _ in fields],
        "formats": [(_TYPES[ty], (n,)) if n > 1 else _TYPES[ty] for _, ty, _, n in fields],
        "offsets": [offset for _, _, offset, _ in fields],
        "itemsize": size,
    })


_HEADER = [("version", "u8", 0, 1), ("kind", "u8", 1, 1)]
TARGET = _dtype(80, _HEADER + [
    ("flags", "u16", 2, 1), ("client_id", "u32", 4, 1), ("seq", "u64", 8, 1),
    ("t_send_ns", "u64", 16, 1), ("data", "f64", 24, 7),
])
STATE = _dtype(474, [
    ("version", "u8", 0, 1), ("phase", "u8", 1, 1), ("robot_mode", "u8", 2, 1),
    ("has_errors", "u8", 3, 1), ("flags", "u16", 4, 1), ("client_id", "u32", 6, 1),
    ("seq_accepted", "u64", 10, 1), ("t_send_ns_accepted", "u64", 18, 1),
    ("t_node_ns", "u64", 26, 1), ("robot_time_ms", "u64", 34, 1), ("success_rate", "f64", 42, 1),
    ("q", "f64", 50, 7), ("dq", "f64", 106, 7), ("tau_ext", "f64", 162, 7),
    ("o_t_ee", "f64", 218, 16), ("o_f_ext_k", "f64", 346, 6), ("target", "f64", 394, 7),
    ("accepted", "u64", 450, 1), ("refused", "u64", 458, 1), ("dropped", "u64", 466, 1),
])
GRIPPER = _dtype(40, _HEADER + [
    ("_pad", "u16", 2, 1), ("client_id", "u32", 4, 1), ("seq", "u64", 8, 1),
    ("t_send_ns", "u64", 16, 1), ("width", "f64", 24, 1), ("force", "f64", 32, 1),
])
GRIPPER_STATE = _dtype(40, [
    ("version", "u8", 0, 1), ("flags", "u8", 1, 1), ("_pad", "u16", 2, 1),
    ("client_id", "u32", 4, 1), ("t_node_ns", "u64", 8, 1), ("width", "f64", 16, 1),
    ("commanded", "f64", 24, 1), ("max_width", "f64", 32, 1),
])


@dataclass(frozen=True, eq=False)
class ArmState:
    """One `franka/<arm>/state` sample, decoded; arrays are read-only."""

    phase: str
    robot_mode: str
    has_errors: bool
    holding: bool
    joints: bool
    holder: int
    q: np.ndarray
    dq: np.ndarray
    tau_ext_hat_filtered: np.ndarray
    O_T_EE: np.ndarray
    O_F_ext_hat_K: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray
    target: np.ndarray
    control_command_success_rate: float
    seq_accepted: int
    accepted: int
    refused: int
    dropped: int
    time: float
    t_node_ns: int
    received_ns: int
    round_trip: Optional[float]


@dataclass(frozen=True)
class GripperState:
    """One `franka/<arm>/gripper/state` sample, decoded; widths in m."""

    width: float
    max_width: float
    commanded: float
    is_grasped: bool
    calibrated: bool
    moving: bool
    fault: bool
    holder: int
    t_node_ns: int
    received_ns: int


def _record(buf, dtype, name):
    buf = bytes(buf)
    if len(buf) != dtype.itemsize:
        raise ProtocolError(f"{name}: {len(buf)} bytes, expected {dtype.itemsize}")
    if buf[0] != VERSION:
        raise ProtocolError(f"{name}: wire version {buf[0]}, expected {VERSION}")
    return np.frombuffer(buf, dtype)[0]


def _readonly(array):
    array = np.array(array, dtype=float)
    array.setflags(write=False)
    return array


def _name(table, code):
    return table[code] if code < len(table) else "unknown"


def quaternion_from_matrix(rotation):
    """Unit quaternion `(x, y, z, w)`, `w >= 0`, of a rotation matrix."""
    r = rotation
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q = [(r[2, 1] - r[1, 2]) * s, (r[0, 2] - r[2, 0]) * s, (r[1, 0] - r[0, 1]) * s, 0.25 / s]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.asarray(q) / np.linalg.norm(q)
    return -q if q[3] < 0.0 else q


def decode_state(buf, client_id=0, received_ns=0):
    """`ArmState` of a state sample; `round_trip` is set when `client_id` holds the arm and a
    target of it was accepted."""
    rec = _record(buf, STATE, "state")
    pose = rec["o_t_ee"].reshape(4, 4, order="F")
    holder = int(rec["client_id"])
    sent = int(rec["t_send_ns_accepted"])
    mine = client_id != 0 and holder == client_id and int(rec["seq_accepted"]) > 0 and sent > 0
    return ArmState(
        phase=_name(PHASES, int(rec["phase"])),
        robot_mode=_name(ROBOT_MODES, int(rec["robot_mode"])),
        has_errors=bool(rec["has_errors"]),
        holding=bool(rec["flags"] & FLAG_HOLDING),
        joints=bool(rec["flags"] & FLAG_JOINTS),
        holder=holder,
        q=_readonly(rec["q"]),
        dq=_readonly(rec["dq"]),
        tau_ext_hat_filtered=_readonly(rec["tau_ext"]),
        O_T_EE=_readonly(pose),
        O_F_ext_hat_K=_readonly(rec["o_f_ext_k"]),
        position=_readonly(pose[:3, 3]),
        quaternion=_readonly(quaternion_from_matrix(pose[:3, :3])),
        target=_readonly(rec["target"]),
        control_command_success_rate=float(rec["success_rate"]),
        seq_accepted=int(rec["seq_accepted"]),
        accepted=int(rec["accepted"]),
        refused=int(rec["refused"]),
        dropped=int(rec["dropped"]),
        time=int(rec["robot_time_ms"]) * 1e-3,
        t_node_ns=int(rec["t_node_ns"]),
        received_ns=received_ns,
        round_trip=(received_ns - sent) * 1e-9 if mine else None,
    )


def decode_gripper_state(buf, received_ns=0):
    rec = _record(buf, GRIPPER_STATE, "gripper state")
    flags = int(rec["flags"])
    return GripperState(
        width=float(rec["width"]),
        max_width=float(rec["max_width"]),
        commanded=float(rec["commanded"]),
        is_grasped=bool(flags & GRIPPER_GRASPED),
        calibrated=bool(flags & GRIPPER_CALIBRATED),
        moving=bool(flags & GRIPPER_MOVING),
        fault=bool(flags & GRIPPER_FAULT),
        holder=int(rec["client_id"]),
        t_node_ns=int(rec["t_node_ns"]),
        received_ns=received_ns,
    )


def encode_target(kind, client_id, seq, t_send_ns, data, anchor):
    """`kind` is `"cartesian"` or `"joints"`; `data` the 7 floats."""
    msg = np.zeros((), TARGET)
    msg["version"], msg["kind"] = VERSION, KINDS[kind]
    msg["flags"] = TARGET_ANCHOR if anchor else 0
    msg["client_id"], msg["seq"], msg["t_send_ns"] = client_id, seq, t_send_ns
    msg["data"] = data
    return msg.tobytes()


def encode_gripper(kind, client_id, seq, t_send_ns, width, force):
    """`kind` is `"width"` or `"grasp"`."""
    msg = np.zeros((), GRIPPER)
    msg["version"], msg["kind"] = VERSION, GRIPPER_KINDS[kind]
    msg["client_id"], msg["seq"], msg["t_send_ns"] = client_id, seq, t_send_ns
    msg["width"], msg["force"] = width, force
    return msg.tobytes()


def encode_request(client_id, mode=None, speed=None, episode=None):
    """A `cmd/*` payload, the fields in the node's order, absent ones left out."""
    body = {"client_id": client_id}
    for key, value in (("mode", mode), ("speed", speed), ("episode", episode)):
        if value is not None:
            body[key] = value
    return json.dumps(body, separators=(",", ":"))
