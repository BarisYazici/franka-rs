"""The 88-byte VrTargetMsg: one controller pose, one gripper fraction, the flag bits.

The single codec for the channel the bridge publishes and teleop.py consumes. It was three
copies of the same struct (the bridge's numpy packer, the client's pure decoder, the wire
tools' synthesiser); anything that touches these bytes imports it from here.

Little-endian, no padding; every field is naturally aligned, so a C struct of the same
fields has this layout byte for byte. The offsets are pinned by tests/test_vr_wire.py:

    0   uint32  magic      VR_MSG_MAGIC ("HRLV")
    4   uint32  version    VR_MSG_VERSION
    8   uint64  seq        publisher's monotonic counter
    16  double  pos[3]     [m], remapped robot-base frame
    40  double  quat[4]    x, y, z, w  (scipy Rotation.as_quat order, NOT w-first)
    72  double  gripper    closed fraction [0,1]
    80  uint32  flags      VR_FLAG_*
    84  uint32  buttons    VR_BUTTON_*
    88  (size)

Pure stdlib on purpose: importing it pulls in nothing of the bridge or the client.
"""
import struct

VR_FMT = struct.Struct("<IIQ8dII")          # pos[3] + quat[4] + gripper = 8 doubles
VR_MSG_SIZE = VR_FMT.size                   # 88
assert VR_MSG_SIZE == 88

VR_MSG_MAGIC = 0x564C5248                   # "HRLV" little-endian
VR_MSG_VERSION = 1

VR_FLAG_ENGAGED = 1                         # grip held AND the stream is fresh
VR_FLAG_FRESH = 2                           # the raw 4x4 changed within the last 250 ms
VR_FLAG_CONTROLLER_ON = 4                   # a controller frame arrived within the last 5 s
# The client half of the tree has always spelled these without the VR_ prefix.
FLAG_ENGAGED, FLAG_FRESH, FLAG_CONTROLLER_ON = (
    VR_FLAG_ENGAGED, VR_FLAG_FRESH, VR_FLAG_CONTROLLER_ON)

VR_BUTTON_A = 1
VR_BUTTON_B = 2
VR_BUTTON_RJ = 4        # right thumbstick click (forces a forward-direction relatch)
VR_BUTTON_GRIP = 8      # this channel's grip (RG/LG) is physically held
VR_BUTTON_TRIGGER = 16  # this channel's index trigger past VR_TRIGGER_THRESHOLD
VR_BUTTON_ALL = (VR_BUTTON_A | VR_BUTTON_B | VR_BUTTON_RJ
                 | VR_BUTTON_GRIP | VR_BUTTON_TRIGGER)


def pack_vr_target(seq, pos, quat, gripper, *, engaged, fresh, controller_on, buttons=0):
    """Encode a VrTargetMsg (88 bytes).

    `pos` is any 3-sequence [m]; `quat` any 4-sequence in (x, y, z, w) order; `gripper` the
    closed fraction in [0,1] (0=open..1=closed, which is what `rightTrig[0]` reports).
    numpy arrays are accepted: every element goes through `float()`.
    """
    pos = [float(v) for v in pos]
    if len(pos) != 3:
        raise ValueError(f"pack_vr_target: pos must have shape (3,), got ({len(pos)},)")
    quat = [float(v) for v in quat]
    if len(quat) != 4:
        raise ValueError(f"pack_vr_target: quat must have shape (4,), got ({len(quat)},)")

    flags = ((VR_FLAG_ENGAGED if engaged else 0)
             | (VR_FLAG_FRESH if fresh else 0)
             | (VR_FLAG_CONTROLLER_ON if controller_on else 0))
    return VR_FMT.pack(VR_MSG_MAGIC, VR_MSG_VERSION, int(seq),
                       *pos, *quat, float(gripper), flags, int(buttons))


def decode_vr(payload):
    """(pos, quat, gripper, flags), or a string saying why the bytes are not a VrTargetMsg.

    The lenient decoder: a reader of a live socket must survive a foreign publisher, so a
    wrong length or a wrong magic/version is a reason to report, not an exception. The
    message carries no hand id: which controller it is follows from the port alone.
    """
    if len(payload) != VR_MSG_SIZE:
        return f"length {len(payload)}"
    magic, version, _seq, *rest = VR_FMT.unpack(payload)
    if magic != VR_MSG_MAGIC or version != VR_MSG_VERSION:
        return f"magic {magic:#x} version {version}"
    return tuple(rest[0:3]), tuple(rest[3:7]), rest[7], rest[8]


def unpack_vr_target(buf):
    """Decode an 88-byte VrTargetMsg into a dict, raising ValueError on a bad buffer.

    The strict decoder, and the only one that surfaces `seq` and `buttons`. Used where a
    malformed buffer is a bug rather than noise on a shared socket.
    """
    if len(buf) != VR_MSG_SIZE:
        raise ValueError(f"expected {VR_MSG_SIZE}-byte buffer, got {len(buf)}")
    magic, version, seq, *rest = VR_FMT.unpack(buf)
    if magic != VR_MSG_MAGIC:
        raise ValueError(f"bad magic: expected 0x{VR_MSG_MAGIC:08X}, got 0x{magic:08X}")
    flags = rest[8]
    return {
        "version": version,
        "seq": seq,
        "pos": tuple(rest[0:3]),
        "quat": tuple(rest[3:7]),
        "gripper": rest[7],
        "flags": flags,
        "buttons": rest[9],
        "engaged": bool(flags & VR_FLAG_ENGAGED),
        "fresh": bool(flags & VR_FLAG_FRESH),
        "controller_on": bool(flags & VR_FLAG_CONTROLLER_ON),
    }
