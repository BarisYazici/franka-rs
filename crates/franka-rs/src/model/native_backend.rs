//! Native serial-chain rigid-body backend.
//!
//! This is the default [`RobotModelBackend`] implementation. It is a direct
//! port of the behaviour of libfranka's `franka::RobotModel`
//! (`src/robot_model.cpp`, identical in 0.20.4 and 0.21.2), which delegates to
//! Pinocchio:
//!
//! | libfranka / Pinocchio | here |
//! |---|---|
//! | `pinocchio::forwardKinematics` + `data.oMi[i]` | `NativeBackend::forward_kinematics` |
//! | `getJointJacobian(..., LOCAL)` / `getFrameJacobian(..., LOCAL)` | body Jacobian |
//! | `... LOCAL_WORLD_ALIGNED` | zero Jacobian |
//! | `pinocchio::crba` | `NativeBackend::crba` |
//! | `pinocchio::rnea` / `computeGeneralizedGravity` | `NativeBackend::rnea` |
//! | `RobotModel::updateInertiaIfNeeded` | `NativeBackend::body_inertias` |
//!
//! Everything after [`NativeBackend::from_urdf`] runs on fixed-size arrays and
//! `nalgebra` static matrices, so no heap allocation happens on the query path.

use nalgebra::{Matrix3, Vector3};
use std::collections::HashMap;

use crate::error::{FrankaError, FrankaResult};
use crate::model::spatial::{Force, Motion, SpatialInertia, Transform};
use crate::model::RobotModelBackend;

/// Number of actuated joints of an FR3 / FER arm.
pub(crate) const DOF: usize = 7;

/// Name of the flange link, mirroring `RobotModel::kLastLinkName`.
const LAST_LINK_NAME: &str = "link8";

/// Serial-chain rigid-body model built from a URDF.
#[derive(Debug, Clone)]
pub struct NativeBackend {
    /// Transform from joint `i-1`'s frame (or the world for `i == 0`) to the
    /// frame of joint `i` before its own rotation is applied. Fixed joints in
    /// between are folded into this placement, exactly like Pinocchio does.
    joint_placement: [Transform; DOF],
    /// Unit rotation axis of each joint, in that joint's own frame.
    joint_axis: [Vector3<f64>; DOF],
    /// Body inertia of the link driven by each joint, about that joint's frame
    /// origin. Inertias of links behind fixed joints are merged in, exactly
    /// like Pinocchio's URDF parser does.
    link_inertia: [SpatialInertia; DOF],
    /// Transform from joint 7's frame to the flange (`link8`) frame.
    flange_placement: Transform,
}

/// Which joint a frame hangs off, and where it sits relative to that joint.
struct FrameSpec {
    /// Number of joints supporting the frame (`1..=DOF`).
    support: usize,
    /// Placement of the frame in the frame of joint `support`.
    offset: Transform,
}

impl NativeBackend {
    /// Builds the model from a URDF string.
    ///
    /// The chain from the URDF root to the `link8` frame must contain exactly
    /// seven revolute (or continuous) joints; fixed joints anywhere along the
    /// way are folded into the neighbouring placements and the inertias of the
    /// links they carry are merged into the supporting joint, which is what
    /// `pinocchio::urdf::buildModelFromXML` does for the model libfranka builds.
    pub fn from_urdf(urdf: &str) -> FrankaResult<NativeBackend> {
        let robot = urdf_rs::read_from_string(urdf)
            .map_err(|e| FrankaError::Model(format!("libfranka model: cannot parse URDF: {e}")))?;

        // child link name -> joint driving it
        let mut joint_by_child: HashMap<&str, &urdf_rs::Joint> = HashMap::new();
        for joint in &robot.joints {
            if joint_by_child
                .insert(joint.child.link.as_str(), joint)
                .is_some()
            {
                return Err(FrankaError::Model(format!(
                    "libfranka model: link '{}' is the child of more than one joint",
                    joint.child.link
                )));
            }
        }

        if !robot.links.iter().any(|l| l.name == LAST_LINK_NAME) {
            return Err(FrankaError::Model(format!(
                "libfranka model: URDF has no '{LAST_LINK_NAME}' link"
            )));
        }

        // Walk from the flange up to the root and collect the joint chain.
        let mut chain: Vec<&urdf_rs::Joint> = Vec::new();
        let mut cursor = LAST_LINK_NAME;
        while let Some(joint) = joint_by_child.get(cursor) {
            chain.push(joint);
            cursor = joint.parent.link.as_str();
            if chain.len() > robot.joints.len() {
                return Err(FrankaError::Model(
                    "libfranka model: URDF joint tree contains a cycle".to_string(),
                ));
            }
        }
        chain.reverse();

        let mut joint_placement = [Transform::identity(); DOF];
        let mut joint_axis = [Vector3::z(); DOF];
        let mut movable_names: Vec<&str> = Vec::new();
        let mut pending = Transform::identity();

        for joint in &chain {
            let origin = Transform::from_xyz_rpy(joint.origin.xyz.0, joint.origin.rpy.0);
            match &joint.joint_type {
                urdf_rs::JointType::Fixed => {
                    pending = pending.compose(&origin);
                }
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous => {
                    let index = movable_names.len();
                    if index >= DOF {
                        return Err(FrankaError::Model(format!(
                            "libfranka model: URDF chain to '{LAST_LINK_NAME}' has more than {DOF} movable joints"
                        )));
                    }
                    let axis = Vector3::new(
                        joint.axis.xyz.0[0],
                        joint.axis.xyz.0[1],
                        joint.axis.xyz.0[2],
                    );
                    let norm = axis.norm();
                    if norm < 1e-12 {
                        return Err(FrankaError::Model(format!(
                            "libfranka model: joint '{}' has a degenerate axis",
                            joint.name
                        )));
                    }
                    joint_placement[index] = pending.compose(&origin);
                    joint_axis[index] = axis / norm;
                    movable_names.push(joint.name.as_str());
                    pending = Transform::identity();
                }
                other => {
                    return Err(FrankaError::Model(format!(
                        "libfranka model: joint '{}' has unsupported type {other:?}",
                        joint.name
                    )));
                }
            }
        }

        if movable_names.len() != DOF {
            return Err(FrankaError::Model(format!(
                "libfranka model: URDF chain to '{LAST_LINK_NAME}' has {} movable joints, expected {DOF}",
                movable_names.len()
            )));
        }

        // Whatever fixed joints remain after joint 7 place the flange frame.
        let flange_placement = pending;

        // Merge every link's inertia into the joint that supports it.
        let mut link_inertia = [SpatialInertia::zero(); DOF];
        for link in &robot.links {
            if link.inertial.mass.value == 0.0 {
                continue;
            }
            let Some((support, placement)) =
                support_of(&joint_by_child, &movable_names, link.name.as_str())?
            else {
                // Rigidly attached to the world: Pinocchio folds it into the
                // universe body, where it has no effect on a fixed-base model.
                continue;
            };
            let inertial = &link.inertial;
            let com_frame = Transform::from_xyz_rpy(inertial.origin.xyz.0, inertial.origin.rpy.0);
            let i = &inertial.inertia;
            let inertia_at_com = Matrix3::new(
                i.ixx, i.ixy, i.ixz, i.ixy, i.iyy, i.iyz, i.ixz, i.iyz, i.izz,
            );
            // The URDF inertia is given in the inertial frame's axes; rotate it
            // into the link frame before shifting it to the joint origin.
            let rotated = com_frame.rotation * inertia_at_com * com_frame.rotation.transpose();
            let body =
                SpatialInertia::from_com(inertial.mass.value, &com_frame.translation, &rotated);
            link_inertia[support] = link_inertia[support].add(&placement.act_inertia(&body));
        }

        Ok(NativeBackend {
            joint_placement,
            joint_axis,
            link_inertia,
            flange_placement,
        })
    }

    /// Placement of the flange (`link8`) frame in joint 7's frame, as a
    /// column-major 4x4 matrix.
    ///
    /// This is `pinocchio_model_.frames[last_link_frame_index_].placement`, the
    /// transform `RobotModel::updateInertiaIfNeeded` uses to move the total
    /// load from the flange frame into the last joint's frame.
    pub fn flange_placement(&self) -> [f64; 16] {
        self.flange_placement.to_column_major()
    }

    /// Forward kinematics.
    ///
    /// Returns the world pose of every joint frame (`data.oMi[1..=7]` in
    /// Pinocchio terms, 0-indexed here) together with the parent-to-child
    /// transform of each joint.
    fn forward_kinematics(&self, q: &[f64; DOF]) -> ([Transform; DOF], [Transform; DOF]) {
        let mut local = [Transform::identity(); DOF];
        let mut world = [Transform::identity(); DOF];
        for i in 0..DOF {
            local[i] = self.joint_placement[i]
                .compose(&Transform::from_axis_angle(&self.joint_axis[i], q[i]));
            world[i] = if i == 0 {
                local[0]
            } else {
                world[i - 1].compose(&local[i])
            };
        }
        (world, local)
    }

    /// Resolves a `Frame` into the joint it hangs off and its offset.
    fn frame_spec(&self, frame: FrameId, f_t_ee: &[f64; 16], ee_t_k: &[f64; 16]) -> FrameSpec {
        match frame {
            FrameId::Joint(index) => FrameSpec {
                support: index,
                offset: Transform::identity(),
            },
            FrameId::Flange => FrameSpec {
                support: DOF,
                offset: self.flange_placement,
            },
            FrameId::EndEffector => FrameSpec {
                support: DOF,
                offset: self
                    .flange_placement
                    .compose(&Transform::from_column_major(f_t_ee)),
            },
            FrameId::Stiffness => FrameSpec {
                support: DOF,
                offset: self
                    .flange_placement
                    .compose(&Transform::from_column_major(f_t_ee))
                    .compose(&Transform::from_column_major(ee_t_k)),
            },
        }
    }

    /// World pose of a frame, column-major.
    fn frame_pose(&self, q: &[f64; DOF], spec: &FrameSpec) -> [f64; 16] {
        let (world, _) = self.forward_kinematics(q);
        world[spec.support - 1]
            .compose(&spec.offset)
            .to_column_major()
    }

    /// Geometric Jacobian of a frame, 6x7 column-major with the linear rows first.
    ///
    /// `local == true` reproduces Pinocchio's `LOCAL` reference frame (the body
    /// Jacobian), `local == false` its `LOCAL_WORLD_ALIGNED` (the zero
    /// Jacobian). The intermediate quantities follow Pinocchio exactly: the
    /// columns are first built in its `WORLD` convention (about the world
    /// origin) and then moved to the target frame, so a slightly
    /// non-orthonormal `F_T_EE` rotation block behaves the same way here as it
    /// does in libfranka.
    fn frame_jacobian(&self, q: &[f64; DOF], spec: &FrameSpec, local: bool) -> [f64; 42] {
        let (world, _) = self.forward_kinematics(q);
        let frame = world[spec.support - 1].compose(&spec.offset);
        let rt = frame.rotation.transpose();

        let mut out = [0.0f64; 42];
        for j in 0..spec.support {
            // Pinocchio's `WORLD` column: the joint's motion subspace mapped
            // into the world frame, taken about the world origin.
            let column = world[j].act_motion(&Motion::from_axis(&self.joint_axis[j], 1.0));
            let angular_world = column.angular;
            let shifted = column.linear - frame.translation.cross(&angular_world);
            let (linear, angular) = if local {
                (rt * shifted, rt * angular_world)
            } else {
                (shifted, angular_world)
            };
            out[j * 6] = linear.x;
            out[j * 6 + 1] = linear.y;
            out[j * 6 + 2] = linear.z;
            out[j * 6 + 3] = angular.x;
            out[j * 6 + 4] = angular.y;
            out[j * 6 + 5] = angular.z;
        }
        out
    }

    /// The per-joint body inertias with the total load added to the last link.
    ///
    /// Port of `RobotModel::updateInertiaIfNeeded`: the load is expressed in
    /// the flange frame and moved into the last joint's frame with the flange
    /// frame's placement, i.e.
    /// `initial_inertia + placement.act(Inertia(m_total, com, I_total))`.
    fn body_inertias(
        &self,
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [SpatialInertia; DOF] {
        let mut inertias = self.link_inertia;
        let load = SpatialInertia::from_com(
            m_total,
            &Vector3::new(f_x_ctotal[0], f_x_ctotal[1], f_x_ctotal[2]),
            &Matrix3::from_column_slice(i_total),
        );
        inertias[DOF - 1] = inertias[DOF - 1].add(&self.flange_placement.act_inertia(&load));
        inertias
    }

    /// Recursive Newton-Euler algorithm.
    ///
    /// Equivalent to `pinocchio::rnea(model, data, q, dq, ddq)` with
    /// `model.gravity.linear() == gravity_earth`; with `dq == ddq == 0` it is
    /// `pinocchio::computeGeneralizedGravity`.
    fn rnea(
        &self,
        q: &[f64; DOF],
        dq: &[f64; DOF],
        ddq: &[f64; DOF],
        gravity_earth: &Vector3<f64>,
        inertias: &[SpatialInertia; DOF],
    ) -> [f64; DOF] {
        let (_, local) = self.forward_kinematics(q);

        let mut velocity = [Motion::zero(); DOF];
        let mut acceleration = [Motion::zero(); DOF];
        let mut force = [Force::zero(); DOF];

        // Pinocchio models gravity as a base acceleration of -g.
        let base_acceleration = Motion::from_linear(-gravity_earth);

        for i in 0..DOF {
            let parent_velocity = if i == 0 {
                Motion::zero()
            } else {
                velocity[i - 1]
            };
            let parent_acceleration = if i == 0 {
                base_acceleration
            } else {
                acceleration[i - 1]
            };

            let joint_velocity = Motion::from_axis(&self.joint_axis[i], dq[i]);
            let joint_acceleration = Motion::from_axis(&self.joint_axis[i], ddq[i]);

            let v = local[i]
                .act_inv_motion(&parent_velocity)
                .add(&joint_velocity);
            let a = local[i]
                .act_inv_motion(&parent_acceleration)
                .add(&joint_acceleration)
                .add(&v.cross_motion(&joint_velocity));

            velocity[i] = v;
            acceleration[i] = a;
            force[i] = inertias[i]
                .apply(&a)
                .add(&v.cross_force(&inertias[i].apply(&v)));
        }

        let mut tau = [0.0f64; DOF];
        for i in (0..DOF).rev() {
            tau[i] = self.joint_axis[i].dot(&force[i].angular);
            if i > 0 {
                force[i - 1] = force[i - 1].add(&local[i].act_force(&force[i]));
            }
        }
        tau
    }

    /// Composite rigid body algorithm, giving the 7x7 joint-space inertia
    /// matrix column-major. Equivalent to `pinocchio::crba` followed by
    /// libfranka's mirroring of the strictly-lower triangle.
    fn crba(&self, q: &[f64; DOF], inertias: &[SpatialInertia; DOF]) -> [f64; DOF * DOF] {
        let (_, local) = self.forward_kinematics(q);

        let mut composite = *inertias;
        for i in (1..DOF).rev() {
            composite[i - 1] = composite[i - 1].add(&local[i].act_inertia(&composite[i]));
        }

        let mut mass = [0.0f64; DOF * DOF];
        for i in 0..DOF {
            let mut f = composite[i].apply(&Motion::from_axis(&self.joint_axis[i], 1.0));
            mass[i * DOF + i] = self.joint_axis[i].dot(&f.angular);
            let mut j = i;
            while j > 0 {
                f = local[j].act_force(&f);
                j -= 1;
                let value = self.joint_axis[j].dot(&f.angular);
                mass[i * DOF + j] = value;
                mass[j * DOF + i] = value;
            }
        }
        mass
    }
}

/// Internal frame selector, mirroring `franka::Frame`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameId {
    /// `1..=DOF`
    Joint(usize),
    Flange,
    EndEffector,
    Stiffness,
}

/// Finds the movable joint a link is rigidly attached to, and the link frame's
/// placement in that joint's frame. `Ok(None)` means the link is rigidly
/// attached to the world.
fn support_of(
    joint_by_child: &HashMap<&str, &urdf_rs::Joint>,
    movable_names: &[&str],
    link: &str,
) -> FrankaResult<Option<(usize, Transform)>> {
    let mut placement = Transform::identity();
    let mut cursor = link;
    let mut steps = 0usize;
    while let Some(joint) = joint_by_child.get(cursor) {
        steps += 1;
        if steps > joint_by_child.len() + 1 {
            return Err(FrankaError::Model(
                "libfranka model: URDF joint tree contains a cycle".to_string(),
            ));
        }
        if let Some(index) = movable_names.iter().position(|n| *n == joint.name.as_str()) {
            return Ok(Some((index, placement)));
        }
        if joint.joint_type != urdf_rs::JointType::Fixed {
            // A movable joint outside the chain to the flange: the arm model
            // libfranka builds has none, so refuse rather than silently drop it.
            return Err(FrankaError::Model(format!(
                "libfranka model: link '{link}' is behind movable joint '{}', which is not part of the arm chain",
                joint.name
            )));
        }
        placement =
            Transform::from_xyz_rpy(joint.origin.xyz.0, joint.origin.rpy.0).compose(&placement);
        cursor = joint.parent.link.as_str();
    }
    Ok(None)
}

const IDENTITY_16: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

impl RobotModelBackend for NativeBackend {
    fn coriolis(
        &self,
        q: &[f64; DOF],
        dq: &[f64; DOF],
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
        gravity_earth: &[f64; 3],
    ) -> [f64; DOF] {
        let inertias = self.body_inertias(i_total, m_total, f_x_ctotal);
        let g = Vector3::new(gravity_earth[0], gravity_earth[1], gravity_earth[2]);
        let zero = [0.0f64; DOF];
        let full = self.rnea(q, dq, &zero, &g, &inertias);
        let gravity = self.rnea(q, &zero, &zero, &g, &inertias);
        let mut out = [0.0f64; DOF];
        for i in 0..DOF {
            out[i] = full[i] - gravity[i];
        }
        out
    }

    fn gravity(
        &self,
        q: &[f64; DOF],
        gravity_earth: &[f64; 3],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [f64; DOF] {
        // `RobotModel::gravity` installs the load with a *zero* rotational
        // inertia; the generalized gravity does not depend on it either way.
        let inertias = self.body_inertias(&[0.0; 9], m_total, f_x_ctotal);
        let g = Vector3::new(gravity_earth[0], gravity_earth[1], gravity_earth[2]);
        let zero = [0.0f64; DOF];
        self.rnea(q, &zero, &zero, &g, &inertias)
    }

    fn mass(
        &self,
        q: &[f64; DOF],
        i_total: &[f64; 9],
        m_total: f64,
        f_x_ctotal: &[f64; 3],
    ) -> [f64; DOF * DOF] {
        let inertias = self.body_inertias(i_total, m_total, f_x_ctotal);
        self.crba(q, &inertias)
    }

    fn pose(&self, q: &[f64; DOF], joint_index: usize) -> [f64; 16] {
        debug_assert!((1..=DOF).contains(&joint_index));
        let spec = self.frame_spec(
            FrameId::Joint(joint_index.clamp(1, DOF)),
            &IDENTITY_16,
            &IDENTITY_16,
        );
        self.frame_pose(q, &spec)
    }

    fn pose_flange(&self, q: &[f64; DOF]) -> [f64; 16] {
        let spec = self.frame_spec(FrameId::Flange, &IDENTITY_16, &IDENTITY_16);
        self.frame_pose(q, &spec)
    }

    fn pose_ee(&self, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 16] {
        let spec = self.frame_spec(FrameId::EndEffector, f_t_ee, &IDENTITY_16);
        self.frame_pose(q, &spec)
    }

    fn pose_stiffness(&self, q: &[f64; DOF], f_t_ee: &[f64; 16], ee_t_k: &[f64; 16]) -> [f64; 16] {
        let spec = self.frame_spec(FrameId::Stiffness, f_t_ee, ee_t_k);
        self.frame_pose(q, &spec)
    }

    fn body_jacobian(&self, q: &[f64; DOF], joint_index: usize) -> [f64; 42] {
        debug_assert!((1..=DOF).contains(&joint_index));
        let spec = self.frame_spec(
            FrameId::Joint(joint_index.clamp(1, DOF)),
            &IDENTITY_16,
            &IDENTITY_16,
        );
        self.frame_jacobian(q, &spec, true)
    }

    fn body_jacobian_flange(&self, q: &[f64; DOF]) -> [f64; 42] {
        let spec = self.frame_spec(FrameId::Flange, &IDENTITY_16, &IDENTITY_16);
        self.frame_jacobian(q, &spec, true)
    }

    fn body_jacobian_ee(&self, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 42] {
        let spec = self.frame_spec(FrameId::EndEffector, f_t_ee, &IDENTITY_16);
        self.frame_jacobian(q, &spec, true)
    }

    fn body_jacobian_stiffness(
        &self,
        q: &[f64; DOF],
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> [f64; 42] {
        let spec = self.frame_spec(FrameId::Stiffness, f_t_ee, ee_t_k);
        self.frame_jacobian(q, &spec, true)
    }

    fn zero_jacobian(&self, q: &[f64; DOF], joint_index: usize) -> [f64; 42] {
        debug_assert!((1..=DOF).contains(&joint_index));
        let spec = self.frame_spec(
            FrameId::Joint(joint_index.clamp(1, DOF)),
            &IDENTITY_16,
            &IDENTITY_16,
        );
        self.frame_jacobian(q, &spec, false)
    }

    fn zero_jacobian_flange(&self, q: &[f64; DOF]) -> [f64; 42] {
        let spec = self.frame_spec(FrameId::Flange, &IDENTITY_16, &IDENTITY_16);
        self.frame_jacobian(q, &spec, false)
    }

    fn zero_jacobian_ee(&self, q: &[f64; DOF], f_t_ee: &[f64; 16]) -> [f64; 42] {
        let spec = self.frame_spec(FrameId::EndEffector, f_t_ee, &IDENTITY_16);
        self.frame_jacobian(q, &spec, false)
    }

    fn zero_jacobian_stiffness(
        &self,
        q: &[f64; DOF],
        f_t_ee: &[f64; 16],
        ee_t_k: &[f64; 16],
    ) -> [f64; 42] {
        let spec = self.frame_spec(FrameId::Stiffness, f_t_ee, ee_t_k);
        self.frame_jacobian(q, &spec, false)
    }
}
