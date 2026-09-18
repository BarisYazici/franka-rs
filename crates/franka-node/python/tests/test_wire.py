"""`_wire` against `crates/franka-node/tests/wire.json`, which the Rust side keeps equal to its
types: layouts, samples both ways, codes and JSON payloads. Offline."""

import json
import pathlib

import numpy as np
import pytest

from conftest import rotation_matrix
from franka_node import _wire

CRATE = pathlib.Path(__file__).resolve().parents[2]
WIRE = json.loads((CRATE / "tests" / "wire.json").read_text())
DTYPES = {
    "TargetMsg": _wire.TARGET,
    "StateMsg": _wire.STATE,
    "GripperMsg": _wire.GRIPPER,
    "GripperStateMsg": _wire.GRIPPER_STATE,
}
TYPE_NAMES = {"u1": "u8", "u2": "u16", "u4": "u32", "u8": "u64", "f8": "f64"}


def layout(dtype):
    """`[name, type, offset, count]` rows of a dtype, as `wire.json` writes them."""
    rows = []
    for name in dtype.names:
        field, offset = dtype.fields[name][:2]
        base = field.base
        assert base.byteorder in "<|=", name
        rows.append([name, TYPE_NAMES[base.str[1:]], offset, int(np.prod(field.shape) or 1)])
    return rows


@pytest.mark.parametrize("message", sorted(DTYPES))
def test_layout_matches_the_file(message):
    spec, dtype = WIRE["messages"][message], DTYPES[message]
    assert dtype.itemsize == spec["size"]
    assert layout(dtype) == spec["fields"]


@pytest.mark.parametrize("message", sorted(DTYPES))
def test_sample_encodes_and_decodes(message):
    sample, dtype = WIRE["messages"][message]["sample"], DTYPES[message]
    record = np.zeros((), dtype)
    for name, value in sample["values"].items():
        record[name] = value
    assert record.tobytes().hex() == sample["hex"]
    decoded = np.frombuffer(bytes.fromhex(sample["hex"]), dtype)[0]
    for name, value in sample["values"].items():
        np.testing.assert_array_equal(decoded[name], value, err_msg=name)


def test_encoders_produce_the_samples():
    target = WIRE["messages"]["TargetMsg"]["sample"]["values"]
    kind = {v: k for k, v in _wire.KINDS.items()}[target["kind"]]
    encoded = _wire.encode_target(kind, target["client_id"], target["seq"], target["t_send_ns"],
                                  target["data"], anchor=target["flags"] == _wire.TARGET_ANCHOR)
    assert encoded.hex() == WIRE["messages"]["TargetMsg"]["sample"]["hex"]
    grip = WIRE["messages"]["GripperMsg"]["sample"]["values"]
    kind = {v: k for k, v in _wire.GRIPPER_KINDS.items()}[grip["kind"]]
    encoded = _wire.encode_gripper(kind, grip["client_id"], grip["seq"], grip["t_send_ns"],
                                   grip["width"], grip["force"])
    assert encoded.hex() == WIRE["messages"]["GripperMsg"]["sample"]["hex"]


def test_state_sample_decodes_to_an_arm_state():
    spec = WIRE["messages"]["StateMsg"]["sample"]
    values = spec["values"]
    received = values["t_send_ns_accepted"] + 5
    state = _wire.decode_state(bytes.fromhex(spec["hex"]), values["client_id"], received)
    assert state.phase == WIRE["phases"][values["phase"]]
    assert state.robot_mode == WIRE["robot_modes"][values["robot_mode"]]
    assert state.has_errors and state.holding and state.joints
    assert state.holder == values["client_id"]
    np.testing.assert_array_equal(state.O_T_EE, np.reshape(values["o_t_ee"], (4, 4), order="F"))
    np.testing.assert_array_equal(state.position, np.asarray(values["o_t_ee"])[12:15])
    np.testing.assert_array_equal(state.tau_ext_hat_filtered, values["tau_ext"])
    np.testing.assert_array_equal(state.target, values["target"])
    assert state.time == values["robot_time_ms"] * 1e-3
    assert state.round_trip == pytest.approx(5e-9)
    assert not state.q.flags.writeable
    other = _wire.decode_state(bytes.fromhex(spec["hex"]), values["client_id"] + 1)
    assert other.round_trip is None


def test_gripper_state_sample_decodes():
    spec = WIRE["messages"]["GripperStateMsg"]["sample"]
    values = spec["values"]
    state = _wire.decode_gripper_state(bytes.fromhex(spec["hex"]))
    flags = WIRE["flags"]
    assert state.calibrated == bool(values["flags"] & flags["GRIPPER_CALIBRATED"])
    assert state.is_grasped == bool(values["flags"] & flags["GRIPPER_GRASPED"])
    assert state.moving == bool(values["flags"] & flags["GRIPPER_MOVING"])
    assert state.fault == bool(values["flags"] & flags["GRIPPER_FAULT"])
    assert (state.width, state.commanded, state.max_width) == (
        values["width"], values["commanded"], values["max_width"])
    assert state.holder == values["client_id"]


def test_codes_match_the_file():
    assert _wire.VERSION == WIRE["version"]
    assert list(_wire.PHASES) == WIRE["phases"]
    assert list(_wire.ROBOT_MODES) == WIRE["robot_modes"]
    assert _wire.KINDS == WIRE["kinds"]
    assert _wire.GRIPPER_KINDS == WIRE["gripper_kinds"]
    assert {name: getattr(_wire, name) for name in WIRE["flags"]} == WIRE["flags"]


def test_request_encoder_produces_the_examples():
    minimal, full = WIRE["json"]["CmdRequest"]
    assert _wire.encode_request(7) == minimal
    fields = json.loads(full)
    assert _wire.encode_request(**fields) == full
    for reply in WIRE["json"]["CmdReply"]:
        assert set(json.loads(reply)) <= {"ok", "error"}
    episode = json.loads(WIRE["json"]["EpisodeMsg"][0])
    assert {"arm", "recording_id", "file", "t_node_ns", "phase"} <= set(episode)


def test_length_version_and_unknown_codes():
    good = bytes.fromhex(WIRE["messages"]["StateMsg"]["sample"]["hex"])
    with pytest.raises(_wire.ProtocolError, match="473 bytes"):
        _wire.decode_state(good[:-1])
    with pytest.raises(_wire.ProtocolError, match="wire version 2"):
        _wire.decode_state(b"\x02" + good[1:])
    newer = bytearray(good)
    newer[1], newer[2] = 200, 201
    state = _wire.decode_state(bytes(newer))
    assert (state.phase, state.robot_mode) == ("unknown", "unknown")


def test_quaternion_from_matrix_keeps_w_non_negative():
    for q in ([0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0], [0.5, -0.5, 0.5, -0.5],
              [0.1, 0.7, -0.2, 0.3], [0.0, 0.0, 1.0, 1e-3]):
        q = np.asarray(q) / np.linalg.norm(q)
        back = _wire.quaternion_from_matrix(rotation_matrix(q))
        assert back[3] >= 0.0
        np.testing.assert_allclose(back, q if q[3] >= 0 else -q, atol=1e-9)

