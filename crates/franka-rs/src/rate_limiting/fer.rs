//! Rate limits of the Franka Emika Robot (FER, FCI v5), ported from libfranka 0.9.2
//! `include/franka/rate_limiting.h`.
//!
//! The constants in [the parent module](super) are the FR3 values of libfranka 0.21.2; the FER
//! uses different ones, most notably `kTolNumberPacketsLost = 3.0` (the FR3 expects no packet
//! loss and uses `0.0`), which shows up in every velocity limit. The limiting *functions*
//! themselves are unchanged — they take their bounds as arguments — so a v5 control loop calls
//! the same [`super::limit_rate_torques`], [`super::limit_rate_joint_velocities`], … with the constants from
//! this module.
//!
//! Unlike the FR3, the FER's joint velocity limits are flat: there is no position-dependent
//! envelope and no URDF to read one from, so [`MAX_JOINT_VELOCITY`] is used directly in
//! both directions.

/// Sample time constant. Port of `franka::kDeltaT` (`rate_limiting.h:18`). Same as on FR3.
pub const DELTA_T: f64 = 1e-3;
/// Epsilon value for checking limits. Port of `franka::kLimitEps` (`rate_limiting.h:22`).
/// Same as on FR3.
pub const LIMIT_EPS: f64 = 1e-3;
/// Epsilon value for limiting Cartesian accelerations/jerks or not. Port of
/// `franka::kNormEps` (`rate_limiting.h:26`). Same as on FR3.
pub const NORM_EPS: f64 = f64::EPSILON;
/// Number of packets lost considered for the definition of velocity limits. When a packet
/// is lost, FCI assumes a constant acceleration model. Port of
/// `franka::kTolNumberPacketsLost` (`rate_limiting.h:31`) — `3.0` on the FER, `0.0` on
/// the FR3.
pub const TOL_NUMBER_PACKETS_LOST: f64 = 3.0;
/// Factor for the definition of rotational limits using the Cartesian pose interface. Port
/// of `franka::kFactorCartesianRotationPoseInterface` (`rate_limiting.h:35`). Same as on
/// FR3.
pub const FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE: f64 = 0.99;

/// Maximum torque rate. Port of `franka::kMaxTorqueRate` (`rate_limiting.h:39-41`).
pub const MAX_TORQUE_RATE: [f64; 7] = [1000.0 - LIMIT_EPS; 7];
/// Maximum joint jerk. Port of `franka::kMaxJointJerk` (`rate_limiting.h:45-47`).
pub const MAX_JOINT_JERK: [f64; 7] = [
    7500.0 - LIMIT_EPS,
    3750.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    6250.0 - LIMIT_EPS,
    7500.0 - LIMIT_EPS,
    10000.0 - LIMIT_EPS,
    10000.0 - LIMIT_EPS,
];
/// Maximum joint acceleration. Port of `franka::kMaxJointAcceleration`
/// (`rate_limiting.h:51-53`).
pub const MAX_JOINT_ACCELERATION: [f64; 7] = [
    15.0000 - LIMIT_EPS,
    7.500 - LIMIT_EPS,
    10.0000 - LIMIT_EPS,
    12.5000 - LIMIT_EPS,
    15.0000 - LIMIT_EPS,
    20.0000 - LIMIT_EPS,
    20.0000 - LIMIT_EPS,
];
/// Maximum joint velocity. Port of `franka::kMaxJointVelocity` (`rate_limiting.h:57-64`).
///
/// Flat, unlike the FR3's position-dependent envelope: joints 1-4 start from 2.175 rad/s,
/// joints 5-7 from 2.610 rad/s, each reduced by `kLimitEps` and by the velocity the joint
/// could pick up over three lost packets.
pub const MAX_JOINT_VELOCITY: [f64; 7] = [
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[0],
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[1],
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[2],
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[3],
    2.6100 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[4],
    2.6100 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[5],
    2.6100 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[6],
];
/// Minimum joint velocity, i.e. `-MAX_JOINT_VELOCITY`.
///
/// libfranka 0.9.2 has no such array — its `limitRate` overloads take a single symmetric
/// `max_velocity` — but the crate's [`super::limit_rate_joint_velocities`] takes an upper
/// and a lower bound, so the negated array is what a v5 control loop passes as the lower
/// one.
pub const MIN_JOINT_VELOCITY: [f64; 7] = [
    -MAX_JOINT_VELOCITY[0],
    -MAX_JOINT_VELOCITY[1],
    -MAX_JOINT_VELOCITY[2],
    -MAX_JOINT_VELOCITY[3],
    -MAX_JOINT_VELOCITY[4],
    -MAX_JOINT_VELOCITY[5],
    -MAX_JOINT_VELOCITY[6],
];
/// Maximum translational jerk. Port of `franka::kMaxTranslationalJerk`
/// (`rate_limiting.h:68`).
pub const MAX_TRANSLATIONAL_JERK: f64 = 6500.0 - LIMIT_EPS;
/// Maximum translational acceleration. Port of `franka::kMaxTranslationalAcceleration`
/// (`rate_limiting.h:72`).
pub const MAX_TRANSLATIONAL_ACCELERATION: f64 = 13.0000 - LIMIT_EPS;
/// Maximum translational velocity. Port of `franka::kMaxTranslationalVelocity`
/// (`rate_limiting.h:76-77`).
pub const MAX_TRANSLATIONAL_VELOCITY: f64 =
    2.0000 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_TRANSLATIONAL_ACCELERATION;
/// Maximum rotational jerk. Port of `franka::kMaxRotationalJerk` (`rate_limiting.h:81`).
pub const MAX_ROTATIONAL_JERK: f64 = 12500.0 - LIMIT_EPS;
/// Maximum rotational acceleration. Port of `franka::kMaxRotationalAcceleration`
/// (`rate_limiting.h:85`).
pub const MAX_ROTATIONAL_ACCELERATION: f64 = 25.0000 - LIMIT_EPS;
/// Maximum rotational velocity. Port of `franka::kMaxRotationalVelocity`
/// (`rate_limiting.h:89-90`).
pub const MAX_ROTATIONAL_VELOCITY: f64 =
    2.5000 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_ROTATIONAL_ACCELERATION;
/// Maximum elbow jerk. Port of `franka::kMaxElbowJerk` (`rate_limiting.h:94`).
pub const MAX_ELBOW_JERK: f64 = 5000.0 - LIMIT_EPS;
/// Maximum elbow acceleration. Port of `franka::kMaxElbowAcceleration`
/// (`rate_limiting.h:98`).
pub const MAX_ELBOW_ACCELERATION: f64 = 10.0000 - LIMIT_EPS;
/// Maximum elbow velocity. Port of `franka::kMaxElbowVelocity` (`rate_limiting.h:102-103`).
pub const MAX_ELBOW_VELOCITY: f64 =
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_ELBOW_ACCELERATION;
/// Minimum elbow velocity, i.e. `-MAX_ELBOW_VELOCITY`.
///
/// libfranka 0.9.2 has no `kMinElbowVelocity`; its `limitRate(max_velocity, ...)` overload
/// is symmetric (`src/control_loop.cpp:254`). 0.21.2 spells the negation out as a constant
/// and the crate follows it, so the v5 module defines the same derived value.
pub const MIN_ELBOW_VELOCITY: f64 = -MAX_ELBOW_VELOCITY;
