"""Rotation-matrix -> quaternion for the quest bridge (scipy-Rotation-backed).

QUATERNION ORDER IS (x, y, z, w) -- scipy's `Rotation.as_quat()` convention,
NOT w-first. That is the order `VrTargetMsg.quat` carries on the wire and the
order the teleop client decodes, so changing it here silently rotates every
commanded pose.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R


def rmat_to_quat(rot_mat) -> np.ndarray:
    """3x3 rotation matrix -> quaternion (x, y, z, w)."""
    return R.from_matrix(rot_mat).as_quat()
