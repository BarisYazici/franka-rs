"""PURE pose/button -> VrTargetMsg-fields mapping for the quest bridge.

Began as a port of the front half of DROID's VRPolicy; reimplemented as a
pure function, no DROID function or block remains -- see NOTICE.

A freshness/latch/remap state machine that turns raw `OculusReader` frames
into an absolute target pose. There is no robot-state control in it: no
P-controller, no velocity limiting, no velocity action -- the consumer's
clutch re-anchors BOTH the position and the orientation origin on every grip
edge, and the 1 kHz impedance loop provides the smoothing. So this module has
no robot state in it at all, which is also why it is trivially unit-testable
-- and it must stay that way.

THE THREE THINGS THAT MATTER HERE
---------------------------------
1. FRESHNESS. `OculusReader.get_transformations_and_buttons()` returns the LAST
   CACHED frame forever -- when the headset sleeps, the controller loses
   tracking, or adb drops, it keeps handing back the same 4x4 with no error.
   A consumer that trusts the reader therefore keeps driving from a frozen
   pose while every downstream indicator still reads healthy. So freshness is
   derived from CHANGE in the raw matrix: identical consecutive matrices are
   normal for a tick or two (50 Hz polling of a ~72 Hz stream), but 250 ms of
   ZERO change means a dead stream. The consumer applies a SECOND, independent
   guard (receive-time staleness on the ZMQ message), so both a dead reader and
   a dead bridge are covered. `controller_on` is the same change-detector with a
   longer (5 s) window -- it answers "has this controller reported ANYTHING
   new lately", for a UI-level "is it even turned on" indicator, distinct from
   the tighter `fresh` gate that `engaged` depends on.

2. THE FORWARD-DIRECTION LATCH (`vr_to_global_mat`):
   continuously re-latched to inv(pose) while the grip is NOT held, frozen
   while it IS held, and force-relatched by the right thumbstick click (RJ).
   That is what lets the operator stand anywhere relative to the robot: while
   disengaged the published rotation is the CONSTANT `global_to_env_mat` (see
   the tests), and the moment they grip, rotation deltas start accumulating
   from wherever they were holding the controller. The relatch condition is
   recomputed fresh every tick from the CURRENT grip/stick state -- it is not
   a sticky flag that has to be re-armed by some other event, and it also
   covers the very first frame ever seen (before any latch exists, regardless
   of whether the grip already happens to be held on that first frame).

3. THE AXIS REMAP, `rmat_reorder = [-3, -1, 2, 4]` (our measured map, in
   DROID's `rmat_reorder` encoding -- DROID's stock `[-2,-1,-3,4]` was
   measured as swapping up<->forward):
       robot_x (forward) = -head_z
       robot_y (left)    = -head_x
       robot_z (up)      = +head_y
   for a headset placed UPRIGHT facing the workspace. TRANSLATION uses ONLY
   this fixed remap -- it is deliberately NOT rotated by `vr_to_global_mat`, so
   a given hand motion in space always maps to the same robot axis no matter
   how the controller is tilted. ORIENTATION does go through the latch. The two
   therefore live in different frames, which is fine: the consumer's clutch
   anchors them INDEPENDENTLY -- the position offset is added in the base frame
   and the rotation offset right-multiplied. See docs/design.md.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .transforms import rmat_to_quat
from ..wire import (
    VR_BUTTON_A,
    VR_BUTTON_B,
    VR_BUTTON_GRIP,
    VR_BUTTON_RJ,
    VR_BUTTON_TRIGGER,
)

# Where the analog index trigger counts as "pressed" for the BUTTON bit (the
# panel's controller indicator and the home-hold gesture). The analog value
# itself still rides on the wire as `gripper` and is what actually drives the
# gripper -- this threshold is only about what a discrete indicator shows.
VR_TRIGGER_THRESHOLD = 0.5

# Our measured tracker->robot axis map, in DROID's rmat_reorder encoding.
# See the module docstring.
DEFAULT_RMAT_REORDER = [-3, -1, 2, 4]

# Zero CHANGE in the raw 4x4 for longer than this means the stream is dead
# (not merely slow) -- see the module docstring's point 1.
FRESH_TIMEOUT_S = 0.25

# Zero CHANGE in the raw 4x4 for longer than this clears `controller_on`
# (DROID's `num_wait_sec`, repurposed here as the same change-detector with a
# longer window). Published as a status flag only -- long enough that a
# controller resting between moves still reads as on; it is NOT what gates
# driving -- `engaged` is.
CONTROLLER_ON_TIMEOUT_S = 5.0


def vec_to_reorder_mat(vec) -> np.ndarray:
    """Signed permutation matrix from an axis-map vector.

    Entry i of `vec` means `out_axis[i] = sign(vec[i]) * in_axis[|vec[i]|-1]`:
    row i of the result picks out input axis `|vec[i]|-1`, signed.
    """
    indices = [int(abs(v)) - 1 for v in vec]
    X = np.zeros((len(vec), len(vec)))
    X[np.arange(len(vec)), indices] = np.sign(vec)
    return X


@dataclass(frozen=True)
class VrSample:
    """One mapped tick -- exactly the payload fields of a `VrTargetMsg`."""
    pos: np.ndarray        # (3,) [m], robot-base frame, spatial-scaled
    quat: np.ndarray       # (4,) x, y, z, w
    gripper: float         # [0,1], closed fraction
    engaged: bool          # grip held AND fresh
    fresh: bool
    controller_on: bool
    buttons: int           # VR_BUTTON_* bitfield


class VrMapper:
    """The stateful half of the bridge: latch + freshness. Pure Python/numpy,
    no I/O, no threads -- `update()` is a total function of (frame, clock).
    """

    def __init__(
        self,
        controller_id: str = "r",
        spatial_scale: float = 1.0,
        rmat_reorder=None,
        spatial_rotation: bool = True,
        fresh_timeout_s: float = FRESH_TIMEOUT_S,
        controller_on_timeout_s: float = CONTROLLER_ON_TIMEOUT_S,
    ):
        if controller_id not in ("r", "l"):
            raise ValueError(f"controller_id must be 'r' or 'l', got {controller_id!r}")
        self.controller_id = controller_id
        self.spatial_scale = float(spatial_scale)
        self.global_to_env_mat = vec_to_reorder_mat(
            DEFAULT_RMAT_REORDER if rmat_reorder is None else list(rmat_reorder)
        )
        self.fresh_timeout_s = float(fresh_timeout_s)
        self.controller_on_timeout_s = float(controller_on_timeout_s)

        # SPATIAL vs BODY rotation frame. `False` is DROID's convention: the
        # delta is `latched^-1 @ pose`, i.e. measured in the
        # CONTROLLER's own (moving) frame. `True` (the default) measures it as
        # `pose @ latched^-1` -- in the fixed headset frame. On a two-arm rig
        # the body-frame form makes mirrored left/right holds disagree,
        # because the two controllers' local axes are mirror images; the
        # headset frame is shared, so both hands agree.
        self.spatial_rotation = bool(spatial_rotation)

        # The forward-direction reference. Starts at identity; `_latched`
        # tracks whether it has ever been set from a real pose yet, which
        # forces a latch on the first usable frame regardless of grip state.
        self._vr_to_global = np.eye(4)
        self._latched = False

        # Freshness bookkeeping (see the module docstring's point 1). Both
        # `fresh` and `controller_on` are derived from the SAME "when did the
        # raw matrix last change" clock, just with different timeouts.
        self._last_matrix: Optional[np.ndarray] = None
        self._last_change_t: Optional[float] = None

    @property
    def vr_to_global_mat(self) -> np.ndarray:
        """The current forward-direction reference (4x4). Exposed for the
        tests and the probe; the bridge never needs to read it."""
        return self._vr_to_global

    def update(self, poses: dict, buttons: dict, now: float) -> Optional[VrSample]:
        """Map one reader frame. Returns None for an UNUSABLE frame.

        The missing-key guard is not cosmetic: a dropped/partial logcat
        frame (the controller lost tracking for a tick)
        omits this controller's pose or its 'RG'/'RJ' keys, and DROID's
        reference reader thread KeyError-crashes on those -- which froze
        `movement_enabled` at False forever, i.e. teleop silently stopped
        working with no error anywhere. Skipping the frame is the fix; the
        caller publishes a not-fresh message for that tick and carries on.
        A pose with no rotation in it (singular, or left-handed) is skipped
        the same way, before it can touch the latch or the freshness clock.
        """
        cid = self.controller_id
        grip_key = cid.upper() + "G"
        stick_key = cid.upper() + "J"
        trig_key = "rightTrig" if cid == "r" else "leftTrig"

        if not poses or cid not in poses or grip_key not in buttons or stick_key not in buttons:
            return None

        pose = np.asarray(poses[cid], dtype=np.float64)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            return None

        grip = bool(buttons[grip_key])
        stick = bool(buttons[stick_key])

        # --- forward-direction latch ---
        # Relatch whenever the grip is NOT held (continuous "stand anywhere"
        # re-anchoring while disengaged), OR the thumbstick was just clicked
        # (RJ forces exactly one relatch even mid-grip), OR nothing has ever
        # been latched yet (bootstrap on the very first usable frame, even if
        # that frame happens to already be gripping). Frozen only when
        # grip-held AND already latched AND no RJ this tick.
        relatch = (not grip) or stick or not self._latched
        try:
            vr_to_global = np.linalg.inv(pose) if relatch else self._vr_to_global
            # --- orientation: relative to the latched forward direction ---
            if self.spatial_rotation:
                rot_mat = self.global_to_env_mat @ pose @ vr_to_global
            else:
                rot_mat = self.global_to_env_mat @ vr_to_global @ pose
            quat = rmat_to_quat(rot_mat[:3, :3])
        except (np.linalg.LinAlgError, ValueError):
            # Singular or left-handed: garbage, and an unusable frame like a
            # NaN one. The previous latch stays; the caller rides out the gap.
            return None
        self._vr_to_global = vr_to_global
        self._latched = True

        # --- freshness: CHANGE in the raw matrix, not arrival of a frame ---
        if self._last_matrix is None or not np.array_equal(pose, self._last_matrix):
            self._last_change_t = now
        self._last_matrix = pose.copy()
        fresh = (now - self._last_change_t) < self.fresh_timeout_s
        controller_on = (now - self._last_change_t) < self.controller_on_timeout_s

        # --- translation: fixed remap ONLY (decoupled from controller tilt) --
        pos = self.spatial_scale * (self.global_to_env_mat @ pose)[:3, 3]

        trig = buttons.get(trig_key)
        try:
            gripper = float(trig[0])
        except (TypeError, IndexError, ValueError):
            gripper = 0.0
        if not np.isfinite(gripper):
            gripper = 0.0
        gripper = float(np.clip(gripper, 0.0, 1.0))

        # The face buttons are per-CONTROLLER on a Quest: the right one has
        # A/B, the left one X/Y, in the same two positions. One bit each, so
        # a reader never has to know which hand a channel is reading --
        # `VR_BUTTON_B` is "the lower-right face button of THIS channel's
        # controller", B or Y.
        btn = 0
        if bool(buttons.get("A" if cid == "r" else "X", False)):
            btn |= VR_BUTTON_A
        if bool(buttons.get("B" if cid == "r" else "Y", False)):
            btn |= VR_BUTTON_B
        if stick:
            btn |= VR_BUTTON_RJ
        # Grip and trigger as DISCRETE bits, for the panel's controller
        # indicator and any button-driven gesture -- `engaged` and `gripper`
        # could not serve: `engaged` also carries freshness and the clutch
        # latch, and `gripper` is the analog trigger, not the side grip.
        if grip:
            btn |= VR_BUTTON_GRIP
        if gripper > VR_TRIGGER_THRESHOLD:
            btn |= VR_BUTTON_TRIGGER

        return VrSample(
            pos=np.asarray(pos, dtype=np.float64),
            quat=np.asarray(quat, dtype=np.float64),
            gripper=gripper,
            engaged=bool(grip and fresh),
            fresh=bool(fresh),
            controller_on=bool(controller_on),
            buttons=btn,
        )
