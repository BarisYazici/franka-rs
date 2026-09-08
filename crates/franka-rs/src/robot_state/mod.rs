//! Public robot state (mirrors `franka::RobotState`, all values `f64`).
//!
//! The conversion from the packed wire struct (`RobotState::from_wire`) lives in this file too
//! and is added together with the wire layer.

use crate::duration::Duration;
use crate::errors::Errors;

/// Robot mode (`franka::RobotMode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RobotMode {
    /// A mode this client does not model (`kOther`).
    Other,
    /// Ready, with no motion and no external controller running (`kIdle`).
    Idle,
    /// A motion generator and/or an external controller is running (`kMove`).
    Move,
    /// Hand-guiding mode: the brakes are open and the robot follows external forces
    /// (`kGuiding`).
    Guiding,
    /// A reflex (collision or limit violation) fired and the robot stopped (`kReflex`); clear it
    /// with [`crate::Robot::automatic_error_recovery`].
    Reflex,
    /// The user stop (the external enabling device) is pressed (`kUserStopped`).
    UserStopped,
    /// An [`crate::Robot::automatic_error_recovery`] is in progress
    /// (`kAutomaticErrorRecovery`).
    AutomaticErrorRecovery,
}

impl RobotMode {
    /// Parses the wire value (`research_interface::robot::RobotMode`).
    pub const fn from_u8(v: u8) -> Option<RobotMode> {
        Some(match v {
            0 => RobotMode::Other,
            1 => RobotMode::Idle,
            2 => RobotMode::Move,
            3 => RobotMode::Guiding,
            4 => RobotMode::Reflex,
            5 => RobotMode::UserStopped,
            6 => RobotMode::AutomaticErrorRecovery,
            _ => return None,
        })
    }
}

/// libfranka prints robot modes in the `commandNotPossibleMsg` text; keep the same spelling.
impl std::fmt::Display for RobotMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            RobotMode::Other => "Other",
            RobotMode::Idle => "Idle",
            RobotMode::Move => "Move",
            RobotMode::Guiding => "Guiding",
            RobotMode::Reflex => "Reflex",
            RobotMode::UserStopped => "User stopped",
            RobotMode::AutomaticErrorRecovery => "Automatic error recovery",
        };
        f.write_str(s)
    }
}

/// Identity 4x4 transform in column-major order.
pub const IDENTITY_TRANSFORM: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// Describes the robot state (field names and semantics identical to `franka::RobotState`;
/// see libfranka's `robot_state.h` for the documentation of every field).
///
/// With the `serde` feature the state serialises field by field under these names, the
/// arrays as arrays, `time` as milliseconds and the two [`Errors`] as lists of names.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RobotState {
    /// Measured end-effector pose in base frame, column-major 4x4 homogeneous transform (translation in m).
    pub O_T_EE: [f64; 16],
    /// Last *desired* end-effector pose in base frame, column-major 4x4 (m). Not for motion control -- see libfranka's warning.
    pub O_T_EE_d: [f64; 16],
    /// Flange-to-end-effector transform, column-major 4x4 (m); the product of `F_T_NE` and `NE_T_EE`.
    pub F_T_EE: [f64; 16],
    /// Flange-to-nominal-end-effector transform as configured in Desk, column-major 4x4 (m).
    pub F_T_NE: [f64; 16],
    /// Nominal-end-effector-to-end-effector transform set with [`crate::Robot::set_ee`], column-major 4x4 (m).
    pub NE_T_EE: [f64; 16],
    /// End-effector-to-stiffness-frame transform set with [`crate::Robot::set_k`], column-major 4x4 (m).
    pub EE_T_K: [f64; 16],
    /// Mass of the end effector, in kg.
    pub m_ee: f64,
    /// Inertia matrix of the end effector about its centre of mass, column-major 3x3, in kg*m^2.
    pub I_ee: [f64; 9],
    /// Centre of mass of the end effector in flange frame, in m.
    pub F_x_Cee: [f64; 3],
    /// Mass of the external load, in kg.
    pub m_load: f64,
    /// Inertia matrix of the external load about its centre of mass, column-major 3x3, in kg*m^2.
    pub I_load: [f64; 9],
    /// Centre of mass of the external load in flange frame, in m.
    pub F_x_Cload: [f64; 3],
    /// Combined mass of end effector and load, in kg.
    pub m_total: f64,
    /// Combined inertia matrix of end effector and load, column-major 3x3, in kg*m^2.
    pub I_total: [f64; 9],
    /// Combined centre of mass of end effector and load in flange frame, in m.
    pub F_x_Ctotal: [f64; 3],
    /// Measured elbow configuration: joint-3 position in rad and the sign (+1/-1) of joint 4.
    pub elbow: [f64; 2],
    /// Desired elbow configuration, same layout as [`RobotState::elbow`].
    pub elbow_d: [f64; 2],
    /// Commanded elbow configuration, same layout as [`RobotState::elbow`].
    pub elbow_c: [f64; 2],
    /// Commanded elbow velocity: joint-3 velocity in rad/s and 0 for the joint-4 sign.
    pub delbow_c: [f64; 2],
    /// Commanded elbow acceleration: joint-3 acceleration in rad/s^2 and 0 for the joint-4 sign.
    pub ddelbow_c: [f64; 2],
    /// Measured link-side joint torques, in Nm.
    pub tau_J: [f64; 7],
    /// Desired link-side joint torques without gravity, in Nm. Rate-limit torque commands against this.
    pub tau_J_d: [f64; 7],
    /// Derivative of the measured joint torques, in Nm/s.
    pub dtau_J: [f64; 7],
    /// Measured joint positions, in rad.
    pub q: [f64; 7],
    /// Desired joint positions, in rad.
    pub q_d: [f64; 7],
    /// Measured joint velocities, in rad/s.
    pub dq: [f64; 7],
    /// Desired joint velocities, in rad/s.
    pub dq_d: [f64; 7],
    /// Desired joint accelerations, in rad/s^2.
    pub ddq_d: [f64; 7],
    /// Per-joint contact indicator: 1.0 where a contact was detected, else 0.0.
    pub joint_contact: [f64; 7],
    /// Cartesian contact indicator in the stiffness frame (x, y, z, roll, pitch, yaw): 1.0 or 0.0.
    pub cartesian_contact: [f64; 6],
    /// Per-joint collision indicator: 1.0 where a collision was detected, else 0.0.
    pub joint_collision: [f64; 7],
    /// Cartesian collision indicator in the stiffness frame (x, y, z, roll, pitch, yaw): 1.0 or 0.0.
    pub cartesian_collision: [f64; 6],
    /// Filtered external torque estimate, in Nm.
    pub tau_ext_hat_filtered: [f64; 7],
    /// Estimated external wrench on the stiffness frame, expressed in base frame: forces in N, torques in Nm.
    pub O_F_ext_hat_K: [f64; 6],
    /// Estimated external wrench on the stiffness frame, expressed in the stiffness frame: forces in N, torques in Nm.
    pub K_F_ext_hat_K: [f64; 6],
    /// Desired end-effector twist in base frame: linear m/s then angular rad/s.
    pub O_dP_EE_d: [f64; 6],
    /// Gravity vector in base frame, in m/s^2 (the robot's own measurement of which way is down).
    pub O_ddP_O: [f64; 3],
    /// Last commanded end-effector pose in base frame, column-major 4x4 (m).
    pub O_T_EE_c: [f64; 16],
    /// Last commanded end-effector twist in base frame: linear m/s then angular rad/s.
    pub O_dP_EE_c: [f64; 6],
    /// Last commanded end-effector acceleration in base frame: linear m/s^2 then angular rad/s^2.
    pub O_ddP_EE_c: [f64; 6],
    /// Motor-side joint positions, in rad.
    pub theta: [f64; 7],
    /// Motor-side joint velocities, in rad/s.
    pub dtheta: [f64; 7],
    /// Raw readings of the six top accelerometers, in m/s^2 (FCI v10 only; zero on an FER).
    pub accelerometer_top: [[f64; 3]; 6],
    /// Raw readings of the six bottom accelerometers, in m/s^2 (FCI v10 only; zero on an FER).
    pub accelerometer_bottom: [[f64; 3]; 6],
    /// Errors currently active on the robot.
    pub current_errors: Errors,
    /// Errors that aborted the last motion (`reflex_reason`).
    pub last_motion_errors: Errors,
    /// Fraction of the last 100 cycles whose command the robot accepted, in [0, 1].
    pub control_command_success_rate: f64,
    /// Current robot mode.
    pub robot_mode: RobotMode,
    /// Strictly monotonic robot timestamp of this state.
    pub time: Duration,
}

impl Default for RobotState {
    /// Identity transforms, zero everything else, `robot_mode = Other`, gravity along -z in
    /// `O_ddP_O` as the robot reports it.
    fn default() -> Self {
        RobotState {
            O_T_EE: IDENTITY_TRANSFORM,
            O_T_EE_d: IDENTITY_TRANSFORM,
            F_T_EE: IDENTITY_TRANSFORM,
            F_T_NE: IDENTITY_TRANSFORM,
            NE_T_EE: IDENTITY_TRANSFORM,
            EE_T_K: IDENTITY_TRANSFORM,
            m_ee: 0.0,
            I_ee: [0.0; 9],
            F_x_Cee: [0.0; 3],
            m_load: 0.0,
            I_load: [0.0; 9],
            F_x_Cload: [0.0; 3],
            m_total: 0.0,
            I_total: [0.0; 9],
            F_x_Ctotal: [0.0; 3],
            elbow: [0.0; 2],
            elbow_d: [0.0; 2],
            elbow_c: [0.0; 2],
            delbow_c: [0.0; 2],
            ddelbow_c: [0.0; 2],
            tau_J: [0.0; 7],
            tau_J_d: [0.0; 7],
            dtau_J: [0.0; 7],
            q: [0.0; 7],
            q_d: [0.0; 7],
            dq: [0.0; 7],
            dq_d: [0.0; 7],
            ddq_d: [0.0; 7],
            joint_contact: [0.0; 7],
            cartesian_contact: [0.0; 6],
            joint_collision: [0.0; 7],
            cartesian_collision: [0.0; 6],
            tau_ext_hat_filtered: [0.0; 7],
            O_F_ext_hat_K: [0.0; 6],
            K_F_ext_hat_K: [0.0; 6],
            O_dP_EE_d: [0.0; 6],
            O_ddP_O: [0.0, 0.0, -9.81],
            O_T_EE_c: IDENTITY_TRANSFORM,
            O_dP_EE_c: [0.0; 6],
            O_ddP_EE_c: [0.0; 6],
            theta: [0.0; 7],
            dtheta: [0.0; 7],
            accelerometer_top: [[0.0; 3]; 6],
            accelerometer_bottom: [[0.0; 3]; 6],
            current_errors: Errors::default(),
            last_motion_errors: Errors::default(),
            control_command_success_rate: 0.0,
            robot_mode: RobotMode::Other,
            time: Duration::default(),
        }
    }
}

/// Combines the centres of mass of the end effector and the load.
///
/// Port of `franka::combineCenterOfMass` (`src/load_calculations.cpp`, libfranka 0.21.2): the
/// mass-weighted mean, or all zeros when the combined mass is not positive.
pub fn combine_center_of_mass(
    m_ee: f64,
    F_x_Cee: &[f64; 3],
    m_load: f64,
    F_x_Cload: &[f64; 3],
) -> [f64; 3] {
    let mut F_x_Ctotal = [0.0; 3];
    if (m_ee + m_load) > 0.0 {
        for i in 0..3 {
            F_x_Ctotal[i] = (m_ee * F_x_Cee[i] + m_load * F_x_Cload[i]) / (m_ee + m_load);
        }
    }
    F_x_Ctotal
}

/// Skew-symmetric matrix of a vector (`franka::skewSymmetricMatrixFromVector`).
fn skew_symmetric(v: &[f64; 3]) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0)
}

/// Combines the inertia tensors of the end effector and the load about the combined centre of
/// mass.
///
/// Port of `franka::combineInertiaTensor` (`src/load_calculations.cpp`, libfranka 0.21.2). Both
/// tensors are shifted to flange coordinates with the parallel-axis theorem, added, and shifted
/// back to the combined centre of mass. A body with zero mass contributes nothing, and a zero
/// total mass yields a zero tensor. All tensors are column-major `[f64; 9]`.
#[allow(clippy::too_many_arguments)]
pub fn combine_inertia_tensor(
    m_ee: f64,
    F_x_Cee: &[f64; 3],
    I_ee: &[f64; 9],
    m_load: f64,
    F_x_Cload: &[f64; 3],
    I_load: &[f64; 9],
    m_total: f64,
    F_x_Ctotal: &[f64; 3],
) -> [f64; 9] {
    if m_total == 0.0 {
        return [0.0; 9];
    }

    let mut inertia_ee = nalgebra::Matrix3::from_column_slice(I_ee);
    let mut inertia_load = nalgebra::Matrix3::from_column_slice(I_load);
    if m_ee == 0.0 {
        inertia_ee = nalgebra::Matrix3::zeros();
    }
    if m_load == 0.0 {
        inertia_load = nalgebra::Matrix3::zeros();
    }

    let skew_ee = skew_symmetric(F_x_Cee);
    let skew_load = skew_symmetric(F_x_Cload);
    let skew_total = skew_symmetric(F_x_Ctotal);

    let inertia_ee_flange = inertia_ee - m_ee * (skew_ee * skew_ee);
    let inertia_load_flange = inertia_load - m_load * (skew_load * skew_load);
    let inertia_total =
        inertia_ee_flange + inertia_load_flange + m_total * (skew_total * skew_total);

    let mut out = [0.0; 9];
    out.copy_from_slice(inertia_total.as_slice());
    out
}

impl RobotState {
    /// Widens a wire state to the public `f64` state.
    ///
    /// Port of `franka::convertRobotState` (`src/robot_impl.cpp`, libfranka 0.21.2): every
    /// `float` becomes a `double`, `m_total`/`F_x_Ctotal`/`I_total` are derived from the end
    /// effector and load properties, `time` is the `message_id` read as milliseconds, and the
    /// state's `errors`/`reflex_reason` become `current_errors`/`last_motion_errors`. An
    /// unknown `robot_mode` byte maps to [`RobotMode::Other`], matching the C++ switch whose
    /// result is pre-initialised to `kOther`.
    pub fn from_wire(w: &crate::wire::robot::RobotState) -> RobotState {
        use crate::wire::{accel_to_f64, f32s_to_f64};

        let m_ee = w.m_ee.get() as f64;
        let m_load = w.m_load.get() as f64;
        let I_ee = f32s_to_f64(&w.I_ee);
        let I_load = f32s_to_f64(&w.I_load);
        let F_x_Cee = f32s_to_f64(&w.F_x_Cee);
        let F_x_Cload = f32s_to_f64(&w.F_x_Cload);

        // `robot_impl.cpp:516` is `converted.m_total = robot_state.m_ee + robot_state.m_load;`
        // over two `float`s: the sum is computed in `float` and only then widened, which is not
        // the same value as adding the two widened doubles.
        let m_total = f64::from(w.m_ee.get() + w.m_load.get());
        let F_x_Ctotal = combine_center_of_mass(m_ee, &F_x_Cee, m_load, &F_x_Cload);
        let I_total = combine_inertia_tensor(
            m_ee,
            &F_x_Cee,
            &I_ee,
            m_load,
            &F_x_Cload,
            &I_load,
            m_total,
            &F_x_Ctotal,
        );

        RobotState {
            O_T_EE: f32s_to_f64(&w.O_T_EE),
            O_T_EE_d: f32s_to_f64(&w.O_T_EE_d),
            F_T_EE: f32s_to_f64(&w.F_T_EE),
            F_T_NE: f32s_to_f64(&w.F_T_NE),
            NE_T_EE: f32s_to_f64(&w.NE_T_EE),
            EE_T_K: f32s_to_f64(&w.EE_T_K),
            m_ee,
            I_ee,
            F_x_Cee,
            m_load,
            I_load,
            F_x_Cload,
            m_total,
            I_total,
            F_x_Ctotal,
            elbow: f32s_to_f64(&w.elbow),
            elbow_d: f32s_to_f64(&w.elbow_d),
            elbow_c: f32s_to_f64(&w.elbow_c),
            delbow_c: f32s_to_f64(&w.delbow_c),
            ddelbow_c: f32s_to_f64(&w.ddelbow_c),
            tau_J: f32s_to_f64(&w.tau_J),
            tau_J_d: f32s_to_f64(&w.tau_J_d),
            dtau_J: f32s_to_f64(&w.dtau_J),
            q: f32s_to_f64(&w.q),
            q_d: f32s_to_f64(&w.q_d),
            dq: f32s_to_f64(&w.dq),
            dq_d: f32s_to_f64(&w.dq_d),
            ddq_d: f32s_to_f64(&w.ddq_d),
            joint_contact: f32s_to_f64(&w.joint_contact),
            cartesian_contact: f32s_to_f64(&w.cartesian_contact),
            joint_collision: f32s_to_f64(&w.joint_collision),
            cartesian_collision: f32s_to_f64(&w.cartesian_collision),
            tau_ext_hat_filtered: f32s_to_f64(&w.tau_ext_hat_filtered),
            O_F_ext_hat_K: f32s_to_f64(&w.O_F_ext_hat_K),
            K_F_ext_hat_K: f32s_to_f64(&w.K_F_ext_hat_K),
            O_dP_EE_d: f32s_to_f64(&w.O_dP_EE_d),
            O_ddP_O: f32s_to_f64(&w.O_ddP_O),
            O_T_EE_c: f32s_to_f64(&w.O_T_EE_c),
            O_dP_EE_c: f32s_to_f64(&w.O_dP_EE_c),
            O_ddP_EE_c: f32s_to_f64(&w.O_ddP_EE_c),
            theta: f32s_to_f64(&w.theta),
            dtheta: f32s_to_f64(&w.dtheta),
            accelerometer_top: accel_to_f64(&w.accelerometer_top),
            accelerometer_bottom: accel_to_f64(&w.accelerometer_bottom),
            current_errors: Errors::from(w.errors),
            last_motion_errors: Errors::from(w.reflex_reason),
            control_command_success_rate: w.control_command_success_rate.get() as f64,
            robot_mode: RobotMode::from_u8(w.robot_mode).unwrap_or(RobotMode::Other),
            time: Duration::from_millis(w.message_id.get()),
        }
    }

    /// Widens an FCI v5 wire state to the public state.
    ///
    /// Port of `franka::convertRobotState` (`src/robot_impl.cpp`, libfranka 0.9.2). The FER
    /// streams `double` already, so nothing is widened; `m_total`/`F_x_Ctotal`/`I_total` are
    /// combined exactly as on FCI v10, and `time` is the `message_id` read as milliseconds.
    ///
    /// FCI v5 has no accelerometer arrays in its state (`rbk_types.h:38-85`, libfranka 0.9.2),
    /// so `accelerometer_top` and `accelerometer_bottom` are zero. An unknown `robot_mode` byte
    /// maps to [`RobotMode::Other`], matching the C++ switch whose result is pre-initialised to
    /// `kOther`.
    pub fn from_wire_v5(w: &crate::wire::robot::v5::RobotState) -> RobotState {
        use crate::wire::f64s_to_f64;

        let m_ee = w.m_ee.get();
        let m_load = w.m_load.get();
        let I_ee = f64s_to_f64(&w.I_ee);
        let I_load = f64s_to_f64(&w.I_load);
        let F_x_Cee = f64s_to_f64(&w.F_x_Cee);
        let F_x_Cload = f64s_to_f64(&w.F_x_Cload);

        let m_total = m_ee + m_load;
        let F_x_Ctotal = combine_center_of_mass(m_ee, &F_x_Cee, m_load, &F_x_Cload);
        let I_total = combine_inertia_tensor(
            m_ee,
            &F_x_Cee,
            &I_ee,
            m_load,
            &F_x_Cload,
            &I_load,
            m_total,
            &F_x_Ctotal,
        );

        RobotState {
            O_T_EE: f64s_to_f64(&w.O_T_EE),
            O_T_EE_d: f64s_to_f64(&w.O_T_EE_d),
            F_T_EE: f64s_to_f64(&w.F_T_EE),
            F_T_NE: f64s_to_f64(&w.F_T_NE),
            NE_T_EE: f64s_to_f64(&w.NE_T_EE),
            EE_T_K: f64s_to_f64(&w.EE_T_K),
            m_ee,
            I_ee,
            F_x_Cee,
            m_load,
            I_load,
            F_x_Cload,
            m_total,
            I_total,
            F_x_Ctotal,
            elbow: f64s_to_f64(&w.elbow),
            elbow_d: f64s_to_f64(&w.elbow_d),
            elbow_c: f64s_to_f64(&w.elbow_c),
            delbow_c: f64s_to_f64(&w.delbow_c),
            ddelbow_c: f64s_to_f64(&w.ddelbow_c),
            tau_J: f64s_to_f64(&w.tau_J),
            tau_J_d: f64s_to_f64(&w.tau_J_d),
            dtau_J: f64s_to_f64(&w.dtau_J),
            q: f64s_to_f64(&w.q),
            q_d: f64s_to_f64(&w.q_d),
            dq: f64s_to_f64(&w.dq),
            dq_d: f64s_to_f64(&w.dq_d),
            ddq_d: f64s_to_f64(&w.ddq_d),
            joint_contact: f64s_to_f64(&w.joint_contact),
            cartesian_contact: f64s_to_f64(&w.cartesian_contact),
            joint_collision: f64s_to_f64(&w.joint_collision),
            cartesian_collision: f64s_to_f64(&w.cartesian_collision),
            tau_ext_hat_filtered: f64s_to_f64(&w.tau_ext_hat_filtered),
            O_F_ext_hat_K: f64s_to_f64(&w.O_F_ext_hat_K),
            K_F_ext_hat_K: f64s_to_f64(&w.K_F_ext_hat_K),
            O_dP_EE_d: f64s_to_f64(&w.O_dP_EE_d),
            O_ddP_O: f64s_to_f64(&w.O_ddP_O),
            O_T_EE_c: f64s_to_f64(&w.O_T_EE_c),
            O_dP_EE_c: f64s_to_f64(&w.O_dP_EE_c),
            O_ddP_EE_c: f64s_to_f64(&w.O_ddP_EE_c),
            theta: f64s_to_f64(&w.theta),
            dtheta: f64s_to_f64(&w.dtheta),
            accelerometer_top: [[0.0; 3]; 6],
            accelerometer_bottom: [[0.0; 3]; 6],
            current_errors: Errors::from(w.errors),
            last_motion_errors: Errors::from(w.reflex_reason),
            control_command_success_rate: w.control_command_success_rate.get(),
            robot_mode: RobotMode::from_u8(w.robot_mode).unwrap_or(RobotMode::Other),
            time: Duration::from_millis(w.message_id.get()),
        }
    }
}

#[cfg(test)]
mod tests;
