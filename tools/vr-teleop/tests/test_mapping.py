"""`VrMapper` pinned by value: the axis remap, the latch, the two change detectors.

Where a number can be checked against the headset instead of against this
file, it is: `RECORDED_DISENGAGED_QUAT` is the quaternion the real bridge
published on every disengaged tick of a recorded two-controller session, and it
is a function of the axis remap alone. Swap two axes or flip a sign in
`DEFAULT_RMAT_REORDER` and that literal stops matching -- which is exactly the
bug class the remap comment warns about.

Every threshold is asserted from both sides: the value that passes and the
value that fails.
"""

import numpy as np
import pytest

from franka_vr.quest_bridge.mapping import (
    CONTROLLER_ON_TIMEOUT_S,
    DEFAULT_RMAT_REORDER,
    FRESH_TIMEOUT_S,
    VR_TRIGGER_THRESHOLD,
    VrMapper,
    vec_to_reorder_mat,
)
from franka_vr.quest_bridge.transforms import rmat_to_quat
from franka_vr.wire import (
    VR_BUTTON_A,
    VR_BUTTON_B,
    VR_BUTTON_GRIP,
    VR_BUTTON_RJ,
    VR_BUTTON_TRIGGER,
)

# Recorded reality: the quaternion the bridge published on EVERY disengaged
# tick of a real two-controller session (3000/3000 desk-idle messages carried
# it, up to the sign flip the branch cut produces). It is `rmat_to_quat` of the
# axis-remap matrix and nothing else, so it pins the remap against hardware.
RECORDED_DISENGAGED_QUAT = (0.5, -0.5, -0.5, 0.5)
# One full-precision disengaged sample from that recording (controller r).
RECORDED_POS = (0.464028, -0.321501, -0.0576818)
RECORDED_QUAT = (0.5, -0.49999999999999994, -0.5, 0.49999999999999994)


def pose(t=(0.0, 0.0, 0.0), rot=None):
    T = np.eye(4)
    if rot is not None:
        T[:3, :3] = rot
    T[:3, 3] = t
    return T


def frame(cid="r", t=(0.0, 0.0, 0.0), rot=None, grip=False, stick=False,
          trig=0.0, a=False, b=False):
    U = cid.upper()
    poses = {cid: pose(t, rot)}
    buttons = {U + "G": grip, U + "J": stick,
               "rightTrig" if cid == "r" else "leftTrig": [trig],
               "A" if cid == "r" else "X": a,
               "B" if cid == "r" else "Y": b}
    return poses, buttons


def rot_z(ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# ------------------------------------------------------------- the axis remap


def test_reorder_matrix_is_the_documented_signed_permutation():
    assert DEFAULT_RMAT_REORDER == [-3, -1, 2, 4]
    M = vec_to_reorder_mat(DEFAULT_RMAT_REORDER)
    # robot_x = -head_z, robot_y = -head_x, robot_z = +head_y.
    assert M.tolist() == [
        [0.0, 0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def test_translation_takes_the_remap_and_the_scale_only():
    m = VrMapper(controller_id="r", spatial_scale=1.0)
    s = m.update(*frame(t=(1.0, 2.0, 3.0)), now=0.0)
    assert tuple(s.pos) == (-3.0, -1.0, 2.0)

    m2 = VrMapper(controller_id="r", spatial_scale=0.4)
    s2 = m2.update(*frame(t=(1.0, 2.0, 3.0)), now=0.0)
    assert tuple(np.round(s2.pos, 12)) == (-1.2, -0.4, 0.8)


def test_the_stock_droid_remap_would_give_a_different_answer():
    """The negative direction for the two tests above: they discriminate."""
    m = VrMapper(controller_id="r", rmat_reorder=[-2, -1, -3, 4])
    s = m.update(*frame(t=(1.0, 2.0, 3.0)), now=0.0)
    assert tuple(s.pos) == (-2.0, -1.0, -3.0)
    assert tuple(np.round(s.quat, 9)) != RECORDED_DISENGAGED_QUAT


def test_disengaged_quaternion_matches_the_recorded_headset_constant():
    """While the grip is released the latch is inv(pose) every tick, so the
    published rotation is the remap's own -- the constant the real bridge
    published for a whole session."""
    m = VrMapper(controller_id="r")
    for i, t in enumerate([(0.1, 0.2, 0.3), (0.5, -0.1, 0.9), (-2.0, 0.0, 4.0)]):
        s = m.update(*frame(t=t, rot=rot_z(0.3 * i)), now=0.01 * i)
        q = np.asarray(s.quat)
        if q[3] < 0:
            q = -q  # the branch cut: the sign flipped in 26 % of recorded ticks
        assert np.allclose(q, RECORDED_DISENGAGED_QUAT, atol=1e-12)
    assert np.allclose(rmat_to_quat(vec_to_reorder_mat(DEFAULT_RMAT_REORDER)[:3, :3]),
                       RECORDED_DISENGAGED_QUAT, atol=1e-12)


def test_a_recorded_disengaged_sample_reproduces_bit_for_bit():
    """Take the pose the recorded `pos` implies, put it back through the
    mapper, and the same message comes out."""
    M = vec_to_reorder_mat(DEFAULT_RMAT_REORDER)[:3, :3]
    head_t = np.linalg.inv(M) @ np.asarray(RECORDED_POS)
    m = VrMapper(controller_id="r", spatial_scale=1.0)
    s = m.update(*frame(t=tuple(head_t)), now=0.0)
    assert np.allclose(s.pos, RECORDED_POS, atol=1e-15)
    q = np.asarray(s.quat)
    assert np.allclose(q if q[3] > 0 else -q, np.abs(RECORDED_QUAT) * np.sign(RECORDED_QUAT),
                       atol=1e-12)
    assert (s.fresh, s.engaged, s.buttons) == (True, False, 0)


# --------------------------------------------------------- the two detectors


def test_fresh_window_is_250ms_from_the_last_CHANGE():
    m = VrMapper(controller_id="r")
    still = frame(t=(1.0, 0.0, 0.0))
    assert m.update(*still, now=0.0).fresh is True
    assert m.update(*still, now=0.249).fresh is True     # 249 ms of no change
    assert m.update(*still, now=FRESH_TIMEOUT_S).fresh is False
    assert m.update(*still, now=0.251).fresh is False
    # One changed matrix re-arms it.
    assert m.update(*frame(t=(1.1, 0.0, 0.0)), now=0.252).fresh is True
    assert FRESH_TIMEOUT_S == 0.25


def test_controller_on_window_is_5s_from_the_last_change():
    m = VrMapper(controller_id="r")
    still = frame(t=(1.0, 0.0, 0.0))
    m.update(*still, now=0.0)
    s = m.update(*still, now=4.999)
    assert (s.fresh, s.controller_on) == (False, True)
    assert m.update(*still, now=CONTROLLER_ON_TIMEOUT_S).controller_on is False
    assert CONTROLLER_ON_TIMEOUT_S == 5.0


def test_engaged_needs_the_grip_AND_freshness():
    m = VrMapper(controller_id="r")
    held = frame(t=(1.0, 0.0, 0.0), grip=True)
    assert m.update(*held, now=0.0).engaged is True
    assert m.update(*held, now=0.3).engaged is False     # same matrix: stale
    assert m.update(*frame(t=(2.0, 0.0, 0.0)), now=0.4).engaged is False  # no grip


# ----------------------------------------------------------------- the latch


def test_latch_freezes_while_gripped_and_relatches_when_released():
    m = VrMapper(controller_id="r")
    m.update(*frame(t=(0.0, 0.0, 0.0), rot=rot_z(0.0)), now=0.0)
    latched = m.vr_to_global_mat.copy()

    gripped = m.update(*frame(t=(0.0, 0.0, 0.0), rot=rot_z(0.5), grip=True), now=0.01)
    assert np.array_equal(m.vr_to_global_mat, latched)   # frozen under grip
    assert not np.allclose(np.asarray(gripped.quat), RECORDED_DISENGAGED_QUAT)

    released = m.update(*frame(t=(0.0, 0.0, 0.0), rot=rot_z(0.9)), now=0.02)
    assert not np.array_equal(m.vr_to_global_mat, latched)
    q = np.asarray(released.quat)
    assert np.allclose(q if q[3] > 0 else -q, RECORDED_DISENGAGED_QUAT, atol=1e-12)


def test_thumbstick_click_forces_one_relatch_mid_grip():
    m = VrMapper(controller_id="r")
    m.update(*frame(rot=rot_z(0.0)), now=0.0)
    m.update(*frame(rot=rot_z(0.5), grip=True), now=0.01)
    frozen = m.vr_to_global_mat.copy()
    s = m.update(*frame(rot=rot_z(0.7), grip=True, stick=True), now=0.02)
    assert not np.array_equal(m.vr_to_global_mat, frozen)
    assert s.buttons & VR_BUTTON_RJ
    q = np.asarray(s.quat)
    assert np.allclose(q if q[3] > 0 else -q, RECORDED_DISENGAGED_QUAT, atol=1e-12)
    # ... and exactly one: the next gripped tick is frozen again.
    relatched = m.vr_to_global_mat.copy()
    m.update(*frame(rot=rot_z(0.9), grip=True), now=0.03)
    assert np.array_equal(m.vr_to_global_mat, relatched)


def test_the_very_first_frame_latches_even_if_it_is_already_gripping():
    m = VrMapper(controller_id="r")
    s = m.update(*frame(rot=rot_z(0.4), grip=True), now=0.0)
    assert not np.array_equal(m.vr_to_global_mat, np.eye(4))
    q = np.asarray(s.quat)
    assert np.allclose(q if q[3] > 0 else -q, RECORDED_DISENGAGED_QUAT, atol=1e-12)


def test_a_pose_with_no_rotation_is_an_unusable_frame_and_keeps_the_latch():
    """A singular pose (no inverse to latch) and a left-handed one under a
    frozen grip (no quaternion) map to None like any unusable frame. They used
    to raise out of `update()` and end the bridge. Neither touches the latch
    nor the freshness clock, and the next good frame carries on from both."""
    m = VrMapper(controller_id="r")
    m.update(*frame(t=(0.1, 0.0, 0.0), rot=rot_z(0.3)), now=0.0)
    m.update(*frame(t=(0.2, 0.0, 0.0), rot=rot_z(0.4), grip=True), now=0.01)
    latched = m.vr_to_global_mat.copy()

    released = {"RG": False, "RJ": False, "rightTrig": [0.0]}
    assert m.update({"r": np.zeros((4, 4))}, released, 0.02) is None
    mirrored = pose(t=(0.3, 0.0, 0.0), rot=np.diag([1.0, 1.0, -1.0]))
    assert m.update(*frame(grip=True), now=0.03) is not None      # still frozen
    assert m.update({"r": mirrored}, dict(released, RG=True), 0.04) is None
    assert np.array_equal(m.vr_to_global_mat, latched)
    # Nor a change: the 0.03 pose again 260 ms later is stale, garbage in between or not.
    assert m.update(*frame(grip=True), now=0.29).fresh is False

    s = m.update(*frame(t=(0.2, 0.0, 0.0), rot=rot_z(0.4), grip=True), now=0.30)
    assert s is not None and s.engaged
    assert np.array_equal(m.vr_to_global_mat, latched)


def test_spatial_rotation_false_is_the_body_frame_form():
    first = frame(t=(0.2, 0.1, 0.0), rot=rot_z(0.6))
    poses, buttons = frame(t=(0.3, 0.0, 0.0), rot=rot_x(0.4), grip=True)
    spatial = VrMapper(controller_id="r", spatial_rotation=True)
    body = VrMapper(controller_id="r", spatial_rotation=False)
    spatial.update(*first, now=0.0)
    body.update(*first, now=0.0)
    a = spatial.update(poses, buttons, 0.01)
    b = body.update(poses, buttons, 0.01)
    assert not np.allclose(a.quat, b.quat)
    assert spatial.spatial_rotation is True and body.spatial_rotation is False


# ------------------------------------------------------- unusable frames


@pytest.mark.parametrize("poses,buttons", [
    ({}, {"RG": False, "RJ": False}),                       # empty frame
    ({"l": np.eye(4)}, {"RG": False, "RJ": False}),         # other controller only
    ({"r": np.eye(4)}, {"RJ": False}),                      # grip key missing
    ({"r": np.eye(4)}, {"RG": False}),                      # stick key missing
    ({"r": np.eye(3)}, {"RG": False, "RJ": False}),         # wrong shape
    ({"r": np.full((4, 4), np.nan)}, {"RG": False, "RJ": False}),
])
def test_unusable_frames_map_to_none(poses, buttons):
    assert VrMapper(controller_id="r").update(poses, buttons, 0.0) is None


def test_a_usable_frame_is_not_none():
    """The other direction for the table above."""
    assert VrMapper(controller_id="r").update(*frame(), now=0.0) is not None


# ---------------------------------------------------------------- the bits


def test_trigger_bit_threshold_and_gripper_clamp():
    m = VrMapper(controller_id="r")
    assert VR_TRIGGER_THRESHOLD == 0.5
    assert m.update(*frame(t=(0.1, 0, 0), trig=0.5), now=0.0).buttons & VR_BUTTON_TRIGGER == 0
    s = m.update(*frame(t=(0.2, 0, 0), trig=0.5001), now=0.01)
    assert s.buttons & VR_BUTTON_TRIGGER
    assert s.gripper == 0.5001
    assert m.update(*frame(t=(0.3, 0, 0), trig=4.0), now=0.02).gripper == 1.0
    assert m.update(*frame(t=(0.4, 0, 0), trig=-2.0), now=0.03).gripper == 0.0
    assert m.update(*frame(t=(0.5, 0, 0), trig=float("nan")), now=0.04).gripper == 0.0


def test_face_buttons_are_per_controller_in_the_same_two_bits():
    r = VrMapper(controller_id="r").update(*frame("r", a=True, b=False, grip=True), now=0.0)
    left = VrMapper(controller_id="l").update(*frame("l", a=True, b=False, grip=True), now=0.0)
    assert r.buttons == left.buttons == VR_BUTTON_A | VR_BUTTON_GRIP
    both = VrMapper(controller_id="l").update(*frame("l", a=True, b=True), now=0.0)
    assert both.buttons == VR_BUTTON_A | VR_BUTTON_B
    assert both.buttons & VR_BUTTON_GRIP == 0


def test_controller_id_is_validated():
    with pytest.raises(ValueError):
        VrMapper(controller_id="left")
