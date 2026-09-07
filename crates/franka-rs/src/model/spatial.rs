//! Spatial (6D) algebra helpers used by the native rigid-body backend.
//!
//! The conventions follow Featherstone's *Rigid Body Dynamics Algorithms* and
//! Pinocchio, which is what libfranka's `franka::RobotModel` is built on:
//!
//! * A rigid transform `aMb` maps quantities expressed in frame `b` to frame `a`.
//! * Motion vectors are stored angular-first as `(w, v)` and force vectors as
//!   `(n, f)`. Only the public `Model` API re-orders them to libfranka's
//!   linear-first layout.
//! * A [`SpatialInertia`] is stored as `(mass, moment = mass * com, inertia
//!   about the frame origin)`, which is additive without any special case for
//!   zero mass. It is mathematically identical to Pinocchio's
//!   `(mass, lever, inertia-at-com)` representation used by
//!   `RobotModel::updateInertiaIfNeeded`.
//!
//! Transforms are deliberately *not* `nalgebra::Isometry3`: libfranka feeds the
//! raw `F_T_EE` / `EE_T_K` blocks from the robot state straight into Eigen /
//! Pinocchio without re-orthonormalising them, and the Franka Hand transform
//! shipped by the robot uses `0.7071` rather than `cos(pi/4)`. Round-tripping
//! that through a unit quaternion would move the end-effector pose by ~1e-6,
//! far outside the 1e-9 conformance tolerance.

use nalgebra::{Matrix3, Vector3};

/// Skew-symmetric matrix `v^` such that `v^ * a == v.cross(&a)`.
#[inline]
pub(crate) fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// A rigid transform stored as a raw rotation block and a translation.
///
/// Port of the parts of `pinocchio::SE3` that `franka::RobotModel` relies on.
/// There is no explicit `inverse`: [`Transform::act_inv_motion`] transposes the
/// rotation block in place, exactly the way `pinocchio::SE3::inverse()` does,
/// which matters because `F_T_EE` and `EE_T_K` arrive from the robot slightly
/// non-orthonormal and a true matrix inverse would not agree with libfranka.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Transform {
    /// Rotation block, column-major in memory like every `nalgebra` matrix.
    pub(crate) rotation: Matrix3<f64>,
    /// Translation of the child frame origin in the parent frame.
    pub(crate) translation: Vector3<f64>,
}

impl Transform {
    /// The identity transform.
    pub(crate) const fn identity() -> Self {
        Self {
            rotation: Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            translation: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    /// Builds a transform from a rotation block and a translation.
    pub(crate) fn new(rotation: Matrix3<f64>, translation: Vector3<f64>) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Builds a transform from a URDF `origin` (`xyz` plus fixed-axis `rpy`).
    ///
    /// The URDF convention is `R = Rz(yaw) * Ry(pitch) * Rx(roll)`.
    pub(crate) fn from_xyz_rpy(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        let (sr, cr) = rpy[0].sin_cos();
        let (sp, cp) = rpy[1].sin_cos();
        let (sy, cy) = rpy[2].sin_cos();
        let rotation = Matrix3::new(
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        );
        Self::new(rotation, Vector3::new(xyz[0], xyz[1], xyz[2]))
    }

    /// Builds a transform from a column-major 4x4 homogeneous matrix.
    ///
    /// The rotation block is taken verbatim, exactly like
    /// `RobotModel::updateFramePlacements` does with `F_T_EE` / `EE_T_K`.
    pub(crate) fn from_column_major(m: &[f64; 16]) -> Self {
        Self::new(
            Matrix3::new(m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]),
            Vector3::new(m[12], m[13], m[14]),
        )
    }

    /// Serialises this transform as a column-major 4x4 homogeneous matrix.
    pub(crate) fn to_column_major(self) -> [f64; 16] {
        let r = &self.rotation;
        let t = &self.translation;
        [
            r[(0, 0)],
            r[(1, 0)],
            r[(2, 0)],
            0.0,
            r[(0, 1)],
            r[(1, 1)],
            r[(2, 1)],
            0.0,
            r[(0, 2)],
            r[(1, 2)],
            r[(2, 2)],
            0.0,
            t.x,
            t.y,
            t.z,
            1.0,
        ]
    }

    /// Composes two transforms: `self * other` (`aMb * bMc == aMc`).
    pub(crate) fn compose(&self, other: &Transform) -> Transform {
        Transform::new(
            self.rotation * other.rotation,
            self.rotation * other.translation + self.translation,
        )
    }

    /// A rotation of `angle` about `axis`, with a zero translation.
    ///
    /// This is the joint transform of a revolute joint, mirroring Pinocchio's
    /// `JointModelRevoluteUnaligned`/`JointModelRZ` configuration transform.
    pub(crate) fn from_axis_angle(axis: &Vector3<f64>, angle: f64) -> Transform {
        let (s, c) = angle.sin_cos();
        let v = 1.0 - c;
        let (x, y, z) = (axis.x, axis.y, axis.z);
        let rotation = Matrix3::new(
            c + x * x * v,
            x * y * v - z * s,
            x * z * v + y * s,
            y * x * v + z * s,
            c + y * y * v,
            y * z * v - x * s,
            z * x * v - y * s,
            z * y * v + x * s,
            c + z * z * v,
        );
        Transform::new(rotation, Vector3::zeros())
    }

    /// Transforms a motion vector from the child frame into the parent frame.
    pub(crate) fn act_motion(&self, m: &Motion) -> Motion {
        let angular = self.rotation * m.angular;
        Motion {
            angular,
            linear: self.rotation * m.linear + self.translation.cross(&angular),
        }
    }

    /// Transforms a motion vector from the parent frame into the child frame.
    ///
    /// Port of `pinocchio::SE3::actInv(Motion)`.
    pub(crate) fn act_inv_motion(&self, m: &Motion) -> Motion {
        let rt = self.rotation.transpose();
        Motion {
            angular: rt * m.angular,
            linear: rt * (m.linear - self.translation.cross(&m.angular)),
        }
    }

    /// Transforms a force vector from the child frame into the parent frame.
    pub(crate) fn act_force(&self, f: &Force) -> Force {
        let linear = self.rotation * f.linear;
        Force {
            angular: self.rotation * f.angular + self.translation.cross(&linear),
            linear,
        }
    }

    /// Transforms a spatial inertia from the child frame into the parent frame.
    ///
    /// The parent-frame counterpart of `pinocchio::SE3::act(Inertia)`.
    pub(crate) fn act_inertia(&self, i: &SpatialInertia) -> SpatialInertia {
        let rotated_moment = self.rotation * i.moment;
        let p = &self.translation;
        let sp = skew(p);
        let sh = skew(&rotated_moment);
        SpatialInertia {
            mass: i.mass,
            moment: rotated_moment + i.mass * p,
            inertia: self.rotation * i.inertia * self.rotation.transpose()
                - sh * sp
                - sp * sh
                - i.mass * (sp * sp),
        }
    }
}

/// A spatial motion vector `(angular, linear)`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Motion {
    /// Angular part (velocity or acceleration).
    pub(crate) angular: Vector3<f64>,
    /// Linear part, taken at the frame origin.
    pub(crate) linear: Vector3<f64>,
}

impl Motion {
    /// The zero motion.
    pub(crate) fn zero() -> Self {
        Self {
            angular: Vector3::zeros(),
            linear: Vector3::zeros(),
        }
    }

    /// A pure translational motion (used for the base acceleration `-g`).
    pub(crate) fn from_linear(linear: Vector3<f64>) -> Self {
        Self {
            angular: Vector3::zeros(),
            linear,
        }
    }

    /// A pure rotation about `axis` scaled by `magnitude`.
    pub(crate) fn from_axis(axis: &Vector3<f64>, magnitude: f64) -> Self {
        Self {
            angular: axis * magnitude,
            linear: Vector3::zeros(),
        }
    }

    /// Component-wise sum.
    pub(crate) fn add(&self, other: &Motion) -> Motion {
        Motion {
            angular: self.angular + other.angular,
            linear: self.linear + other.linear,
        }
    }

    /// Motion-motion cross product `self x other`.
    pub(crate) fn cross_motion(&self, other: &Motion) -> Motion {
        Motion {
            angular: self.angular.cross(&other.angular),
            linear: self.angular.cross(&other.linear) + self.linear.cross(&other.angular),
        }
    }

    /// Motion-force cross product `self x* force`.
    pub(crate) fn cross_force(&self, force: &Force) -> Force {
        Force {
            angular: self.angular.cross(&force.angular) + self.linear.cross(&force.linear),
            linear: self.angular.cross(&force.linear),
        }
    }
}

/// A spatial force vector `(moment, linear force)`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Force {
    /// Moment about the frame origin.
    pub(crate) angular: Vector3<f64>,
    /// Linear force.
    pub(crate) linear: Vector3<f64>,
}

impl Force {
    /// The zero force.
    pub(crate) fn zero() -> Self {
        Self {
            angular: Vector3::zeros(),
            linear: Vector3::zeros(),
        }
    }

    /// Component-wise sum.
    pub(crate) fn add(&self, other: &Force) -> Force {
        Force {
            angular: self.angular + other.angular,
            linear: self.linear + other.linear,
        }
    }
}

/// A rigid-body spatial inertia expressed about a frame origin.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct SpatialInertia {
    /// Body mass.
    pub(crate) mass: f64,
    /// First moment of mass, `mass * center_of_mass`.
    pub(crate) moment: Vector3<f64>,
    /// Rotational inertia about the *frame origin*, not the centre of mass.
    pub(crate) inertia: Matrix3<f64>,
}

impl SpatialInertia {
    /// The zero inertia.
    pub(crate) fn zero() -> Self {
        Self {
            mass: 0.0,
            moment: Vector3::zeros(),
            inertia: Matrix3::zeros(),
        }
    }

    /// Builds an inertia from mass, centre of mass and the rotational inertia
    /// *about the centre of mass*, which is how both URDF `<inertial>` blocks
    /// and libfranka's `I_total` are specified.
    pub(crate) fn from_com(
        mass: f64,
        com: &Vector3<f64>,
        inertia_at_com: &Matrix3<f64>,
    ) -> SpatialInertia {
        let sc = skew(com);
        SpatialInertia {
            mass,
            moment: com * mass,
            inertia: inertia_at_com - mass * (sc * sc),
        }
    }

    /// Component-wise sum. Additive in this representation with no special case
    /// for a zero total mass, unlike Pinocchio's `Inertia::operator+`.
    pub(crate) fn add(&self, other: &SpatialInertia) -> SpatialInertia {
        SpatialInertia {
            mass: self.mass + other.mass,
            moment: self.moment + other.moment,
            inertia: self.inertia + other.inertia,
        }
    }

    /// Applies the inertia to a motion, giving the corresponding force.
    pub(crate) fn apply(&self, m: &Motion) -> Force {
        Force {
            angular: self.inertia * m.angular + self.moment.cross(&m.linear),
            linear: self.mass * m.linear - self.moment.cross(&m.angular),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_xyz_rpy_matches_manual_rotation() {
        let t = Transform::from_xyz_rpy([1.0, 2.0, 3.0], [std::f64::consts::FRAC_PI_2, 0.0, 0.0]);
        // Rx(90 deg) maps y -> z and z -> -y.
        let y = t.rotation * Vector3::new(0.0, 1.0, 0.0);
        assert!((y - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-15);
        let z = t.rotation * Vector3::new(0.0, 0.0, 1.0);
        assert!((z - Vector3::new(0.0, -1.0, 0.0)).norm() < 1e-15);
        assert_eq!(t.translation, Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn column_major_round_trip() {
        let t = Transform::from_xyz_rpy([0.1, -0.2, 0.3], [0.4, 0.5, -0.6]);
        let back = Transform::from_column_major(&t.to_column_major());
        assert!((back.rotation - t.rotation).norm() < 1e-15);
        assert!((back.translation - t.translation).norm() < 1e-15);
    }

    #[test]
    fn act_inv_motion_undoes_act_motion() {
        let t = Transform::from_xyz_rpy([0.1, -0.2, 0.3], [0.4, 0.5, -0.6]);
        let m = Motion {
            angular: Vector3::new(1.0, -2.0, 0.5),
            linear: Vector3::new(-0.3, 0.7, 2.0),
        };
        let round = t.act_inv_motion(&t.act_motion(&m));
        assert!((round.angular - m.angular).norm() < 1e-14);
        assert!((round.linear - m.linear).norm() < 1e-14);
    }

    #[test]
    fn inertia_transform_preserves_kinetic_energy() {
        // 0.5 * v^T I v is frame invariant when v is transformed along with I.
        let i = SpatialInertia::from_com(
            2.5,
            &Vector3::new(0.01, -0.02, 0.3),
            &Matrix3::new(0.1, 0.01, 0.0, 0.01, 0.2, 0.02, 0.0, 0.02, 0.3),
        );
        let t = Transform::from_xyz_rpy([0.1, -0.2, 0.3], [0.4, 0.5, -0.6]);
        let v_child = Motion {
            angular: Vector3::new(0.3, -1.2, 0.7),
            linear: Vector3::new(0.5, 0.1, -0.9),
        };
        let f_child = i.apply(&v_child);
        let energy_child =
            v_child.angular.dot(&f_child.angular) + v_child.linear.dot(&f_child.linear);

        let i_parent = t.act_inertia(&i);
        let v_parent = t.act_motion(&v_child);
        let f_parent = i_parent.apply(&v_parent);
        let energy_parent =
            v_parent.angular.dot(&f_parent.angular) + v_parent.linear.dot(&f_parent.linear);

        assert!((energy_child - energy_parent).abs() < 1e-12);
    }
}
