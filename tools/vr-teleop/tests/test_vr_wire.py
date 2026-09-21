"""The 88-byte VrTargetMsg pinned by value, in both directions.

The literal below was produced by the packer this codec replaces, so a field dropped,
resized or reordered fails here rather than on the wire. A round-trip alone would not:
it passes with two symmetric mistakes.
"""
import struct

import pytest

from franka_vr import wire as w

SEQ = 0x0102030405060708
POS = (0.125, -0.25, 1.5)
QUAT = (0.0, 0.5, -0.5, 0.7071067811865476)
GRIPPER = 0.75
BUTTONS = w.VR_BUTTON_RJ | w.VR_BUTTON_GRIP          # 4 | 8 == 12
FLAGS = w.VR_FLAG_ENGAGED | w.VR_FLAG_CONTROLLER_ON  # 1 | 4 == 5

WIRE = bytes.fromhex(
    "48524c56"                                        #  0 magic  "HRLV"
    "01000000"                                        #  4 version 1
    "0807060504030201"                                #  8 seq
    "000000000000c03f" "000000000000d0bf" "000000000000f83f"   # 16 pos
    "0000000000000000" "000000000000e03f"                      # 40 quat xy
    "000000000000e0bf" "cd3b7f669ea0e63f"                      #    quat zw
    "000000000000e83f"                                # 72 gripper
    "05000000"                                        # 80 flags
    "0c000000"                                        # 84 buttons
)


def test_constants():
    assert w.VR_MSG_SIZE == 88 == len(WIRE) == w.VR_FMT.size
    assert w.VR_MSG_MAGIC == 0x564C5248
    assert WIRE[0:4] == b"HRLV"
    assert w.VR_MSG_VERSION == 1
    assert (w.VR_FLAG_ENGAGED, w.VR_FLAG_FRESH, w.VR_FLAG_CONTROLLER_ON) == (1, 2, 4)
    assert (w.FLAG_ENGAGED, w.FLAG_FRESH, w.FLAG_CONTROLLER_ON) == (1, 2, 4)
    assert (w.VR_BUTTON_A, w.VR_BUTTON_B, w.VR_BUTTON_RJ) == (1, 2, 4)
    assert (w.VR_BUTTON_GRIP, w.VR_BUTTON_TRIGGER) == (8, 16)
    assert w.VR_BUTTON_ALL == 31


@pytest.mark.parametrize("prefix, offset", [
    ("<II", 8), ("<IIQ", 16), ("<IIQ3d", 40), ("<IIQ7d", 72),
    ("<IIQ8d", 80), ("<IIQ8dI", 84), ("<IIQ8dII", 88),
])
def test_field_offsets(prefix, offset):
    """seq at 8, pos at 16, quat at 40, gripper at 72, flags at 80, buttons at 84, size 88."""
    assert struct.calcsize(prefix) == offset


def test_pack_gives_exactly_these_bytes():
    assert w.pack_vr_target(SEQ, POS, QUAT, GRIPPER, engaged=True, fresh=False,
                            controller_on=True, buttons=BUTTONS) == WIRE


def test_pack_flag_bits():
    def flags_of(**kw):
        return w.pack_vr_target(0, (0, 0, 0), (0, 0, 0, 1), 0.0, **kw)[80:84]

    assert flags_of(engaged=False, fresh=False, controller_on=False) == b"\x00\x00\x00\x00"
    assert flags_of(engaged=True, fresh=False, controller_on=False) == b"\x01\x00\x00\x00"
    assert flags_of(engaged=False, fresh=True, controller_on=False) == b"\x02\x00\x00\x00"
    assert flags_of(engaged=False, fresh=False, controller_on=True) == b"\x04\x00\x00\x00"
    assert flags_of(engaged=True, fresh=True, controller_on=True) == b"\x07\x00\x00\x00"


def test_unpack_gives_exactly_these_fields():
    assert w.unpack_vr_target(WIRE) == {
        "version": 1,
        "seq": SEQ,
        "pos": POS,
        "quat": QUAT,
        "gripper": GRIPPER,
        "flags": FLAGS,
        "buttons": BUTTONS,
        "engaged": True,
        "fresh": False,
        "controller_on": True,
    }


def test_decode_gives_exactly_these_fields():
    assert w.decode_vr(WIRE) == (POS, QUAT, GRIPPER, FLAGS)


def test_pack_accepts_any_sequence():
    """A list, a tuple and (where installed) a numpy array pack to the same bytes."""
    args = dict(engaged=True, fresh=False, controller_on=True, buttons=BUTTONS)
    assert w.pack_vr_target(SEQ, list(POS), list(QUAT), GRIPPER, **args) == WIRE
    np = pytest.importorskip("numpy")
    assert w.pack_vr_target(SEQ, np.array(POS), np.array(QUAT), GRIPPER, **args) == WIRE


def test_pack_refuses_a_wrong_length():
    with pytest.raises(ValueError, match="pos"):
        w.pack_vr_target(0, (0, 0), (0, 0, 0, 1), 0.0,
                         engaged=False, fresh=False, controller_on=False)
    with pytest.raises(ValueError, match="quat"):
        w.pack_vr_target(0, (0, 0, 0), (0, 0, 1), 0.0,
                         engaged=False, fresh=False, controller_on=False)


def test_decode_reports_a_reason_instead_of_raising():
    assert w.decode_vr(WIRE[:-1]) == "length 87"
    assert w.decode_vr(WIRE + b"\x00") == "length 89"
    assert w.decode_vr(b"") == "length 0"
    assert w.decode_vr(b"XXXX" + WIRE[4:]) == "magic 0x58585858 version 1"
    assert w.decode_vr(WIRE[:4] + b"\x02\x00\x00\x00" + WIRE[8:]) == (
        f"magic {w.VR_MSG_MAGIC:#x} version 2")


def test_unpack_raises_on_a_bad_buffer():
    with pytest.raises(ValueError, match="88-byte buffer, got 87"):
        w.unpack_vr_target(WIRE[:-1])
    with pytest.raises(ValueError, match="bad magic"):
        w.unpack_vr_target(b"XXXX" + WIRE[4:])
