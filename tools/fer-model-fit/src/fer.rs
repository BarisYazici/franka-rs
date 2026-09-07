//! The Franka Emika Robot (FER)'s kinematics, its ten-parameter-per-link inertial
//! description and the URDF those two together produce.
//!
//! The joint origins and axes are `franka_description`'s `fer_joint1..8`,
//! which agree with the robot's own `libfcimodels_x64.so` to 4e-16 (see
//! `docs/book/src/model.md`). The inertial parameters are what
//! [`crate::fit`] identifies from that shared object.

use nalgebra::{Matrix3, Vector3};
use std::f64::consts::FRAC_PI_2;

/// Actuated joints.
pub const DOF: usize = 7;

/// Inertial parameters of one link, about **its own joint frame's origin**:
/// `[m, h_x, h_y, h_z, I_xx, I_xy, I_xz, I_yy, I_yz, I_zz]` with `h = m * c`.
///
/// This is the parameterisation rigid-body dynamics is linear in, which is what
/// makes the identification a linear least-squares problem.
pub type LinkParams = [f64; 10];

/// The seven links driven by the seven joints.
pub type Params = [LinkParams; DOF];

/// `(xyz, rpy)` of `fer_joint1 .. fer_joint8`; the eighth is the fixed
/// joint that places the flange (`link8`).
pub const JOINT_ORIGINS: [([f64; 3], [f64; 3]); 8] = [
    ([0.0, 0.0, 0.333], [0.0, 0.0, 0.0]),
    ([0.0, 0.0, 0.0], [-FRAC_PI_2, 0.0, 0.0]),
    ([0.0, -0.316, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([0.0825, 0.0, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([-0.0825, 0.384, 0.0], [-FRAC_PI_2, 0.0, 0.0]),
    ([0.0, 0.0, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([0.088, 0.0, 0.0], [FRAC_PI_2, 0.0, 0.0]),
    ([0.0, 0.0, 0.107], [0.0, 0.0, 0.0]),
];

/// Joint position limits (libfranka 0.9.2 `robot.h`, `franka_description`).
pub const Q_MIN: [f64; DOF] = [
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
];
/// Joint position limits, upper.
pub const Q_MAX: [f64; DOF] = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];

/// Joint velocity limits (libfranka 0.9.2 `rate_limiting.h`, `kMaxJointVelocity`).
pub const DQ_MAX: [f64; DOF] = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61];

/// Joint effort limits, for the `<limit>` elements of the generated URDF.
const TAU_MAX: [f64; DOF] = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0];

/// `franka_description`'s published `<inertial>` elements for `fer_link1..7`,
/// as `(mass, com, [ixx, ixy, ixz, iyy, iyz, izz])` in the link frame.
///
/// Used only as the point the identification's minimum-norm correction is taken
/// from, so that the fitted parameters stay recognisably close to the published
/// ones instead of drifting into the unidentifiable subspace.
pub const FRANKA_DESCRIPTION_INERTIALS: [(f64, [f64; 3], [f64; 6]); DOF] = [
    (
        4.970684,
        [0.003875, 0.002081, -0.04762],
        [0.70337, -0.000139, 0.006772, 0.70661, 0.019169, 0.009117],
    ),
    (
        0.646926,
        [-0.003141, -0.02872, 0.003495],
        [0.007962, -0.003925, 0.010254, 0.02811, 0.000704, 0.025995],
    ),
    (
        3.228604,
        [2.7518e-2, 3.9252e-2, -6.6502e-2],
        [0.037242, -0.004761, -0.011396, 0.036155, -0.012805, 0.01083],
    ),
    (
        3.587895,
        [-5.317e-2, 1.04419e-1, 2.7454e-2],
        [0.025853, 0.007796, -0.001332, 0.019552, 0.008641, 0.028323],
    ),
    (
        1.225946,
        [-1.1953e-2, 4.1065e-2, -3.8437e-2],
        [0.035549, -0.002117, -0.004037, 0.029474, 0.000229, 0.008627],
    ),
    (
        1.666555,
        [6.0149e-2, -1.4117e-2, -1.0517e-2],
        [0.001964, 0.000109, -0.001158, 0.004354, 0.000341, 0.005433],
    ),
    (
        0.735522,
        [1.0517e-2, -4.252e-3, 6.1597e-2],
        [
            0.012516, -0.000428, -0.001196, 0.010027, -0.000741, 0.004815,
        ],
    ),
];

/// `(mass, com, I_com)` -> the ten parameters about the joint origin.
pub fn params_from_com(mass: f64, com: &[f64; 3], i_com: &[f64; 6]) -> LinkParams {
    let c = Vector3::new(com[0], com[1], com[2]);
    let i_c = Matrix3::new(
        i_com[0], i_com[1], i_com[2], i_com[1], i_com[3], i_com[4], i_com[2], i_com[4], i_com[5],
    );
    let i_j = i_c + mass * (c.dot(&c) * Matrix3::identity() - c * c.transpose());
    [
        mass,
        mass * c.x,
        mass * c.y,
        mass * c.z,
        i_j[(0, 0)],
        i_j[(0, 1)],
        i_j[(0, 2)],
        i_j[(1, 1)],
        i_j[(1, 2)],
        i_j[(2, 2)],
    ]
}

/// The ten parameters about the joint origin -> `(mass, com, [ixx, ixy, ixz, iyy, iyz, izz])`
/// about the centre of mass, i.e. what a URDF `<inertial>` element carries.
///
/// Requires a strictly positive mass; every link of the fitted FER has one.
pub fn com_from_params(p: &LinkParams) -> (f64, [f64; 3], [f64; 6]) {
    let mass = p[0];
    let c = Vector3::new(p[1] / mass, p[2] / mass, p[3] / mass);
    let i_j = Matrix3::new(p[4], p[5], p[6], p[5], p[7], p[8], p[6], p[8], p[9]);
    let i_c = i_j - mass * (c.dot(&c) * Matrix3::identity() - c * c.transpose());
    (
        mass,
        [c.x, c.y, c.z],
        [
            i_c[(0, 0)],
            i_c[(0, 1)],
            i_c[(0, 2)],
            i_c[(1, 1)],
            i_c[(1, 2)],
            i_c[(2, 2)],
        ],
    )
}

/// `franka_description`'s inertials in the identification's parameterisation.
pub fn franka_description_params() -> Params {
    let mut out = [[0.0; 10]; DOF];
    for (i, (mass, com, i_com)) in FRANKA_DESCRIPTION_INERTIALS.iter().enumerate() {
        out[i] = params_from_com(*mass, com, i_com);
    }
    out
}

/// A number written so that reading it back is exact.
fn f(value: f64) -> String {
    // `{:?}` on `f64` is Rust's shortest round-tripping representation.
    format!("{value:?}")
}

/// Renders a mesh-free, arm-only FER URDF carrying `params`.
///
/// The link names are libfranka's own (`link0 .. link8`), not
/// `franka_description`'s `fer_*`, because
/// `franka::model::NativeBackend::from_urdf` looks for the flange by the name
/// `RobotModel::kLastLinkName` uses.
pub fn urdf(params: &Params, header: &str) -> String {
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    out.push_str(header);
    out.push_str("<robot name=\"fer\">\n");
    out.push_str("  <link name=\"link0\"/>\n");

    for i in 0..DOF {
        let (mass, com, i_com) = com_from_params(&params[i]);
        out.push_str(&format!("  <link name=\"link{}\">\n", i + 1));
        out.push_str("    <inertial>\n");
        out.push_str(&format!(
            "      <origin rpy=\"0 0 0\" xyz=\"{} {} {}\"/>\n",
            f(com[0]),
            f(com[1]),
            f(com[2])
        ));
        out.push_str(&format!("      <mass value=\"{}\"/>\n", f(mass)));
        out.push_str(&format!(
            "      <inertia ixx=\"{}\" ixy=\"{}\" ixz=\"{}\" iyy=\"{}\" iyz=\"{}\" izz=\"{}\"/>\n",
            f(i_com[0]),
            f(i_com[1]),
            f(i_com[2]),
            f(i_com[3]),
            f(i_com[4]),
            f(i_com[5])
        ));
        out.push_str("    </inertial>\n");
        out.push_str("  </link>\n");

        let (xyz, rpy) = JOINT_ORIGINS[i];
        out.push_str(&format!(
            "  <joint name=\"joint{}\" type=\"revolute\">\n",
            i + 1
        ));
        out.push_str(&format!(
            "    <origin rpy=\"{} {} {}\" xyz=\"{} {} {}\"/>\n",
            f(rpy[0]),
            f(rpy[1]),
            f(rpy[2]),
            f(xyz[0]),
            f(xyz[1]),
            f(xyz[2])
        ));
        out.push_str(&format!("    <parent link=\"link{i}\"/>\n"));
        out.push_str(&format!("    <child link=\"link{}\"/>\n", i + 1));
        out.push_str("    <axis xyz=\"0 0 1\"/>\n");
        out.push_str(&format!(
            "    <limit effort=\"{}\" lower=\"{}\" upper=\"{}\" velocity=\"{}\"/>\n",
            f(TAU_MAX[i]),
            f(Q_MIN[i]),
            f(Q_MAX[i]),
            f(DQ_MAX[i])
        ));
        out.push_str("  </joint>\n");
    }

    out.push_str("  <link name=\"link8\"/>\n");
    let (xyz, rpy) = JOINT_ORIGINS[DOF];
    out.push_str("  <joint name=\"joint8\" type=\"fixed\">\n");
    out.push_str(&format!(
        "    <origin rpy=\"{} {} {}\" xyz=\"{} {} {}\"/>\n",
        f(rpy[0]),
        f(rpy[1]),
        f(rpy[2]),
        f(xyz[0]),
        f(xyz[1]),
        f(xyz[2])
    ));
    out.push_str("    <parent link=\"link7\"/>\n");
    out.push_str("    <child link=\"link8\"/>\n");
    out.push_str("  </joint>\n");
    out.push_str("</robot>\n");
    out
}
