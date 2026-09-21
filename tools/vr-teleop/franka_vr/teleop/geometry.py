"""The pose arithmetic teleop.py stands on: quaternions in the xyzw order every layer of
this stack uses (the wire's, franka-node's TargetMsg, franka-rs's target control), and the
clutch.

Pure functions and one small class, no I/O, so selftest.py can assert the algebra directly.
"""
import math


# ---------------------------------------------------------------- quaternions (xyzw)


def qmul(a, b):
    """Hamilton product, so R(qmul(a, b)) == R(a) @ R(b)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qlen(q):
    return math.sqrt(sum(c * c for c in q))


def qunit(q):
    n = qlen(q) or 1.0
    return tuple(c / n for c in q)


def qdot(a, b):
    return sum(x * y for x, y in zip(a, b))


def qcanon(q, previous):
    """RULE 4: the antipodal quaternion is the same rotation; pick the sign that is
    continuous with the previous sample, so a sign flip is not read as a 2 pi step."""
    return tuple(-c for c in q) if previous is not None and qdot(q, previous) < 0.0 else q


def axis_angle(axis, angle):
    """A unit quaternion (xyzw) about `axis`."""
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    k = math.sin(angle / 2.0) / n
    return (axis[0] * k, axis[1] * k, axis[2] * k, math.cos(angle / 2.0))


def qpow(q, s):
    """`q`'s rotation about the same axis, its angle multiplied by `s`: the shortest-arc
    quaternion power, which is `qslerp(identity, q, s)` without the branch. Used to scale a
    wrist's rotation delta the way `spatial_scale` scales a hand's translation -- at s = 1 it
    is the identity, and at any s a zero delta stays zero, which is what keeps the clutch's
    latch tick exact. `atan2` rather than `acos` because the interesting deltas are small,
    where `acos` loses half its precision."""
    q = qunit(q)
    if q[3] < 0.0:
        q = tuple(-c for c in q)            # shortest arc: never scale the long way round
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2])
    if norm < 1e-15:
        return (0.0, 0.0, 0.0, 1.0)
    angle = 2.0 * math.atan2(norm, q[3])
    k = math.sin(s * angle / 2.0) / norm
    return (q[0] * k, q[1] * k, q[2] * k, math.cos(s * angle / 2.0))


def qangle(a, b):
    return 2.0 * math.acos(max(-1.0, min(1.0, abs(qdot(a, b)))))


def qslerp(a, b, t):
    """From a towards b, both unit, shortest arc."""
    if qdot(a, b) < 0.0:
        b = tuple(-c for c in b)
    dot = max(-1.0, min(1.0, qdot(a, b)))
    if dot > 0.999999:
        return qunit(tuple(x + t * (y - x) for x, y in zip(a, b)))
    theta = math.acos(dot)
    s = math.sin(theta)
    wa, wb = math.sin((1.0 - t) * theta) / s, math.sin(t * theta) / s
    return qunit(tuple(wa * x + wb * y for x, y in zip(a, b)))


def quat_of(m):
    """The quaternion (xyzw) of a column-major 4x4's rotation, Shepperd's method."""
    r = ((m[0], m[4], m[8]), (m[1], m[5], m[9]), (m[2], m[6], m[10]))
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i][i] - r[j][j] - r[k][k]) * 2.0
    q = [0.0, 0.0, 0.0, (r[k][j] - r[j][k]) / s]
    q[i] = 0.25 * s
    q[j] = (r[j][i] + r[i][j]) / s
    q[k] = (r[k][i] + r[i][k]) / s
    return tuple(q)


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


# ---------------------------------------------------------------- the clutch


class Clutch:
    """The grip clutch, with both scales.

    Two offsets, latched on every rising edge of the grip and kept DECOUPLED -- a base-frame
    translation that is added and never rotated, and an orientation delta taken in the base
    frame. Composed as one homogeneous transform, the hand's translation would be rotated by
    the latch-time orientation offset and the arm would move the wrong way.
    On the latch tick compose() returns the arm's measured pose, which is what makes a
    re-engage jump-free by construction rather than by clamping.

    One right-multiplied `R_off = R_vr^T R_robot`, composed as `R_vr R_off`, is exactly what
    this does at `rot_scale` 1.0 -- `q_hand (x) conj(q_hand0) (x) q_arm0`
    is the same product, only factored so the DELTA `q_hand (x) conj(q_hand0)` is a thing in
    its own right and can be scaled. A scaled delta is still the identity when the hand has
    not moved, so the latch tick stays exact at every scale.
    """

    def __init__(self, scale, rot_scale=1.0):
        self.scale = float(scale)
        self.rot_scale = float(rot_scale)
        self.p_off = None
        self.q_hand0 = None
        self.q_arm0 = None

    def latch(self, p_arm, q_arm, p_hand, q_hand):
        self.p_off = tuple(a - self.scale * h for a, h in zip(p_arm, p_hand))
        self.q_hand0 = qunit(q_hand)
        self.q_arm0 = qunit(q_arm)

    def compose(self, p_hand, q_hand):
        p = tuple(self.scale * h + o for h, o in zip(p_hand, self.p_off))
        delta = qmul(qunit(q_hand), qconj(self.q_hand0))        # in the base frame
        if self.rot_scale != 1.0:
            delta = qpow(delta, self.rot_scale)
        return p, qunit(qmul(delta, self.q_arm0))

    @property
    def latched(self):
        return self.p_off is not None

    def drop(self):
        self.p_off = self.q_hand0 = self.q_arm0 = None
