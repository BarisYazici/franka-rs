import struct

import pytest

import statemsg
from statemsg import DecodeError, decode, encode, is_reflex


def test_layout_is_the_nodes():
    assert statemsg.FORMAT == "<BBBBHIQQQQd7d7d7d16d6d7dQQQ" and statemsg.SIZE == 474
    assert statemsg.PHASES == ("idle", "acquired", "active", "stopping", "faulted", "homing")
    assert statemsg.ROBOT_MODES[4:] == ("reflex", "user_stopped", "automatic_error_recovery")


def test_round_trip_by_value():
    ee = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.41, -0.07, 0.35, 1]
    raw = encode(phase="active", robot_mode=2, has_errors=False, holding=True, client_id=7200,
                 seq_accepted=9, t_send_ns_accepted=5, t_node_ns=123_456_789, robot_time_ms=42,
                 success_rate=0.98, q=[0.1] * 7, dq=[0, 0.5, 0, 0.02, 0, 0, 0], tau_ext=[0.0] * 7,
                 o_t_ee=ee, o_f_ext_k=[1, 2, 3, 4, 5, 6], target=[0.4, -0.1, 0.3, 0, 0, 0, 1],
                 accepted=100, refused=2, dropped=1)
    assert len(raw) == 474 and raw[0] == 1 and raw[1] == 2 and raw[2] == 2 and raw[3] == 0
    s = decode(raw)
    assert (s["phase"], s["robot_mode"], s["has_errors"], s["holding"], s["joints"]) == ("active", 2, False, True, False)
    assert (s["client_id"], s["t_node_ns"], s["robot_time_ms"], s["success_rate"]) == (7200, 123_456_789, 42, 0.98)
    assert s["dq"][1] == 0.5 and s["dq"][3] == 0.02 and s["o_t_ee"][12:15] == [0.41, -0.07, 0.35]
    assert s["target"] == [0.4, -0.1, 0.3, 0, 0, 0, 1] and s["o_f_ext_k"] == [1, 2, 3, 4, 5, 6]
    assert (s["accepted"], s["refused"], s["dropped"]) == (100, 2, 1)
    # a byte moved is a different sample, not a silent success
    assert decode(raw[:1] + b"\x04" + raw[2:])["phase"] == "faulted"


def test_refuses_other_lengths_and_versions():
    with pytest.raises(DecodeError, match="473 bytes"):
        decode(encode()[:-1])
    with pytest.raises(DecodeError, match="bytes"):
        decode(b'{"t_ns": 1}')
    with pytest.raises(DecodeError, match="version 2"):
        decode(b"\x02" + encode()[1:])
    with pytest.raises(ValueError):
        encode(dq=[0.0] * 6)


def test_reflex_from_state():
    assert not is_reflex(decode(encode(robot_mode=2)))
    for mode in (4, 5, 6):
        assert is_reflex(decode(encode(robot_mode=mode)))
    assert is_reflex(decode(encode(robot_mode=2, has_errors=True)))
    assert not is_reflex(decode(encode(robot_mode=3)))
