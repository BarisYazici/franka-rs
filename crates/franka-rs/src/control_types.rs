//! Command types returned from control callbacks.
//!
//! Port of libfranka 0.21.2 `include/franka/control_types.h` / `src/control_types.cpp` plus the
//! validation helpers of `include/franka/control_tools.h` (`checkFinite`, `checkMatrix`,
//! `checkElbow`, `isValidElbow`, `isHomogeneousTransformation`).
//!
//! libfranka validates the commanded values inside the control loop
//! (`control_loop.cpp` / `robot_impl.cpp` call `checkFinite`/`checkMatrix`/`checkElbow` on the
//! values returned by the user callback) and throws `std::invalid_argument`. The Rust
//! constructors are infallible; the equivalent checks live in [`Finishable`] implementors'
//! `validate` methods, which the control loop calls for every command.

use crate::error::{FrankaError, FrankaResult};

/// Available controller modes for a `Robot`.
///
/// Port of `franka::ControllerMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerMode {
    /// Joint impedance controller.
    JointImpedance,
    /// Cartesian impedance controller.
    CartesianImpedance,
}

/// Helper trait for control and motion generation loops.
///
/// Port of `franka::Finishable`, which in C++ is a base struct carrying the public
/// `motion_finished` flag. In Rust the flag stays a public field on each command type and this
/// trait gives the control loop uniform access to it.
pub trait Finishable {
    /// Whether the motion should terminate after this command has been processed.
    fn is_finished(&self) -> bool;
    /// Sets the "terminate after this command" flag.
    fn set_finished(&mut self, finished: bool);
}

/// Which motion generator a command type drives.
///
/// The mapping onto the wire enum `research_interface::robot::Move::MotionGeneratorMode` is done
/// by the robot layer; this enum keeps `control_types` independent of the protocol structs
/// (libfranka does the same with `src/motion_generator_traits.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionGeneratorKind {
    /// Joint position motion generator.
    JointPosition,
    /// Joint velocity motion generator.
    JointVelocity,
    /// Cartesian pose motion generator.
    CartesianPosition,
    /// Cartesian velocity motion generator.
    CartesianVelocity,
}

/// Implemented by the four motion generator command types.
///
/// Port of libfranka's `MotionGeneratorTraits<T>` (`src/motion_generator_traits.h`).
/// [`Torques`] is a controller command and deliberately does *not* implement this trait.
pub trait MotionGenerator: Finishable + Copy {
    /// The motion generator this command type drives.
    const KIND: MotionGeneratorKind;
}

/// Helper method to indicate that a motion should stop after processing the given command.
///
/// Port of `franka::MotionFinished`.
pub fn motion_finished<T: Finishable>(mut command: T) -> T {
    command.set_finished(true);
    command
}

/// Determines whether the given elbow configuration is valid or not.
///
/// Port of `franka::isValidElbow`.
pub fn is_valid_elbow(elbow: &[f64; 2]) -> bool {
    elbow[1] == -1.0 || elbow[1] == 1.0
}

/// Determines whether the given array represents a valid homogeneous transformation matrix.
///
/// `transform` is a 4x4 matrix in column-major format. Port of
/// `franka::isHomogeneousTransformation` (orthonormality threshold `1e-5`).
pub fn is_homogeneous_transformation(transform: &[f64; 16]) -> bool {
    const ORTHONORMAL_THRESHOLD: f64 = 1e-5;

    if transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 || transform[15] != 1.0 {
        return false;
    }
    for j in 0..3 {
        // j .. column
        let norm = (transform[j * 4].powi(2)
            + transform[j * 4 + 1].powi(2)
            + transform[j * 4 + 2].powi(2))
        .sqrt();
        if (norm - 1.0).abs() > ORTHONORMAL_THRESHOLD {
            return false;
        }
    }
    for i in 0..3 {
        // i .. row
        let norm =
            (transform[i].powi(2) + transform[4 + i].powi(2) + transform[8 + i].powi(2)).sqrt();
        if (norm - 1.0).abs() > ORTHONORMAL_THRESHOLD {
            return false;
        }
    }
    true
}

/// Checks that all elements of the array have a finite value.
///
/// Port of `franka::checkFinite`.
pub fn check_finite(values: &[f64]) -> FrankaResult<()> {
    if values.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(FrankaError::InvalidArgument(
            "Commanding value is infinite or NaN.".to_string(),
        ))
    }
}

/// Checks that the transformation matrix is finite and a homogeneous transformation.
///
/// Port of `franka::checkMatrix`.
pub fn check_matrix(transform: &[f64; 16]) -> FrankaResult<()> {
    check_finite(transform)?;
    if !is_homogeneous_transformation(transform) {
        return Err(FrankaError::InvalidArgument(
            "libfranka: Attempt to set invalid transformation in motion generator. Has to be \
             column major!"
                .to_string(),
        ));
    }
    Ok(())
}

/// Checks that the elbow vector is finite and that the elbow configuration is valid.
///
/// Port of `franka::checkElbow`.
pub fn check_elbow(elbow: &[f64; 2]) -> FrankaResult<()> {
    check_finite(elbow)?;
    if !is_valid_elbow(elbow) {
        return Err(FrankaError::InvalidArgument(
            "Invalid elbow configuration given! Only +1 or -1 are allowed for the sign of the 4th \
             joint."
                .to_string(),
        ));
    }
    Ok(())
}

macro_rules! impl_finishable {
    ($t:ty) => {
        impl Finishable for $t {
            fn is_finished(&self) -> bool {
                self.motion_finished
            }
            fn set_finished(&mut self, finished: bool) {
                self.motion_finished = finished;
            }
        }
    };
}

/// Stores joint-level torque commands without gravity and friction.
///
/// Port of `franka::Torques`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Torques {
    /// Desired torques in \[Nm\].
    pub tau_J: [f64; 7],
    /// Determines whether to finish a currently running motion.
    pub motion_finished: bool,
}

impl Torques {
    /// Creates a new `Torques` instance from desired joint-level torques in \[Nm\].
    pub fn new(tau_J: [f64; 7]) -> Self {
        Torques {
            tau_J,
            motion_finished: false,
        }
    }

    /// Validates the command like libfranka's `control_loop.cpp` does before sending it
    /// (`checkFinite(control_command.tau_J_d)`).
    pub fn validate(&self) -> FrankaResult<()> {
        check_finite(&self.tau_J)
    }
}

impl_finishable!(Torques);

/// Stores values for joint position motion generation.
///
/// Port of `franka::JointPositions`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointPositions {
    /// Desired joint angles in \[rad\].
    pub q: [f64; 7],
    /// Determines whether to finish a currently running motion.
    pub motion_finished: bool,
}

impl JointPositions {
    /// Creates a new `JointPositions` instance from desired joint angles in \[rad\].
    pub fn new(q: [f64; 7]) -> Self {
        JointPositions {
            q,
            motion_finished: false,
        }
    }

    /// Validates the command like libfranka's `control_loop.cpp` does before sending it
    /// (`checkFinite(command.q_c)`).
    pub fn validate(&self) -> FrankaResult<()> {
        check_finite(&self.q)
    }
}

impl_finishable!(JointPositions);

impl MotionGenerator for JointPositions {
    const KIND: MotionGeneratorKind = MotionGeneratorKind::JointPosition;
}

/// Stores values for joint velocity motion generation.
///
/// Port of `franka::JointVelocities`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointVelocities {
    /// Desired joint velocities in \[rad/s\].
    pub dq: [f64; 7],
    /// Determines whether to finish a currently running motion.
    pub motion_finished: bool,
}

impl JointVelocities {
    /// Creates a new `JointVelocities` instance from desired joint velocities in \[rad/s\].
    pub fn new(dq: [f64; 7]) -> Self {
        JointVelocities {
            dq,
            motion_finished: false,
        }
    }

    /// Validates the command like libfranka's `control_loop.cpp` does before sending it
    /// (`checkFinite(command.dq_c)`).
    pub fn validate(&self) -> FrankaResult<()> {
        check_finite(&self.dq)
    }
}

impl_finishable!(JointVelocities);

impl MotionGenerator for JointVelocities {
    const KIND: MotionGeneratorKind = MotionGeneratorKind::JointVelocity;
}

/// Stores values for Cartesian pose motion generation.
///
/// Port of `franka::CartesianPose`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianPose {
    /// Homogeneous transformation `O_T_EE_d`, column major, that transforms from the end effector
    /// frame `EE` to base frame `O`.
    pub O_T_EE: [f64; 16],
    /// Elbow configuration: `elbow[0]` is the position of the 3rd joint in \[rad\],
    /// `elbow[1]` the flip direction of the elbow (4th joint), +1 or -1.
    pub elbow: [f64; 2],
    /// Whether there is a stored elbow configuration.
    ///
    /// libfranka's `CartesianPose::hasElbow()` reports `elbow != {0, 0}`; the constructors below
    /// reproduce that, so `with_elbow([.., ..], [0.0, 0.0])` yields `has_elbow == false`.
    pub has_elbow: bool,
    /// Determines whether to finish a currently running motion.
    pub motion_finished: bool,
}

impl CartesianPose {
    /// Creates a new `CartesianPose` without an elbow configuration.
    ///
    /// `O_T_EE` is a column-major homogeneous transformation matrix.
    pub fn new(O_T_EE: [f64; 16]) -> Self {
        CartesianPose {
            O_T_EE,
            elbow: [0.0; 2],
            has_elbow: false,
            motion_finished: false,
        }
    }

    /// Creates a new `CartesianPose` with an elbow configuration.
    pub fn with_elbow(O_T_EE: [f64; 16], elbow: [f64; 2]) -> Self {
        CartesianPose {
            O_T_EE,
            elbow,
            has_elbow: elbow != [0.0, 0.0],
            motion_finished: false,
        }
    }

    /// Validates the command like libfranka's `control_loop.cpp` does before sending it
    /// (`checkMatrix(command.O_T_EE_c)` and, if an elbow is set, `checkElbow(command.elbow_c)`).
    pub fn validate(&self) -> FrankaResult<()> {
        check_matrix(&self.O_T_EE)?;
        if self.has_elbow {
            check_elbow(&self.elbow)?;
        }
        Ok(())
    }
}

impl_finishable!(CartesianPose);

impl MotionGenerator for CartesianPose {
    const KIND: MotionGeneratorKind = MotionGeneratorKind::CartesianPosition;
}

/// Stores values for Cartesian velocity motion generation.
///
/// Port of `franka::CartesianVelocities`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianVelocities {
    /// Cartesian velocity with respect to the base frame `O`: `(dx, dy, dz)` in \[m/s\] and
    /// `(omega_x, omega_y, omega_z)` in \[rad/s\].
    pub O_dP_EE: [f64; 6],
    /// Elbow configuration, see [`CartesianPose::elbow`].
    pub elbow: [f64; 2],
    /// Whether there is a stored elbow configuration, see [`CartesianPose::has_elbow`].
    pub has_elbow: bool,
    /// Determines whether to finish a currently running motion.
    pub motion_finished: bool,
}

impl CartesianVelocities {
    /// Creates a new `CartesianVelocities` without an elbow configuration.
    pub fn new(O_dP_EE: [f64; 6]) -> Self {
        CartesianVelocities {
            O_dP_EE,
            elbow: [0.0; 2],
            has_elbow: false,
            motion_finished: false,
        }
    }

    /// Creates a new `CartesianVelocities` with an elbow configuration.
    pub fn with_elbow(O_dP_EE: [f64; 6], elbow: [f64; 2]) -> Self {
        CartesianVelocities {
            O_dP_EE,
            elbow,
            has_elbow: elbow != [0.0, 0.0],
            motion_finished: false,
        }
    }

    /// Validates the command like libfranka's `control_loop.cpp` does before sending it
    /// (`checkFinite(command.O_dP_EE_c)` and, if an elbow is set, `checkElbow(command.elbow_c)`).
    pub fn validate(&self) -> FrankaResult<()> {
        check_finite(&self.O_dP_EE)?;
        if self.has_elbow {
            check_elbow(&self.elbow)?;
        }
        Ok(())
    }
}

impl_finishable!(CartesianVelocities);

impl MotionGenerator for CartesianVelocities {
    const KIND: MotionGeneratorKind = MotionGeneratorKind::CartesianVelocity;
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDENTITY: [f64; 16] = [
        1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
    ];

    fn message(result: FrankaResult<()>) -> String {
        match result {
            Err(FrankaError::InvalidArgument(msg)) => msg,
            other => panic!("expected InvalidArgument, got {other:?}"),
        }
    }

    // Port of TEST(Torques, CanConstructFromArray).
    #[test]
    fn torques_can_construct_from_array() {
        let array = [0., 1., 2., 3., 4., 5., 6.];
        let t = Torques::new(array);
        assert_eq!(array, t.tau_J);
        assert!(!t.motion_finished);
        assert!(t.validate().is_ok());
    }

    // Port of TEST(JointPositions, CanConstructFromArray).
    #[test]
    fn joint_positions_can_construct_from_array() {
        let array = [0., 1., 2., 3., 4., 5., 6.];
        let jp = JointPositions::new(array);
        assert_eq!(array, jp.q);
        assert!(jp.validate().is_ok());
    }

    // Port of TEST(JointVelocities, CanConstructFromArray).
    #[test]
    fn joint_velocities_can_construct_from_array() {
        let array = [0., 1., 2., 3., 4., 5., 6.];
        let jv = JointVelocities::new(array);
        assert_eq!(array, jv.dq);
        assert!(jv.validate().is_ok());
    }

    // Port of TEST(CartesianPose, CanConstructFromArray).
    #[test]
    fn cartesian_pose_can_construct_from_array() {
        let p = CartesianPose::new(IDENTITY);
        assert_eq!(IDENTITY, p.O_T_EE);
        assert!(!p.has_elbow);
        assert!(p.validate().is_ok());
    }

    // Port of TEST(CartesianPose, CanConstructFromArrayWithElbow).
    #[test]
    fn cartesian_pose_can_construct_from_array_with_elbow() {
        let elbow = [0., -1.];
        let p = CartesianPose::with_elbow(IDENTITY, elbow);
        assert_eq!(IDENTITY, p.O_T_EE);
        assert_eq!(elbow, p.elbow);
        assert!(p.has_elbow);
        assert!(p.validate().is_ok());
    }

    // Port of TEST(CartesianVelocities, CanConstructFromArray).
    #[test]
    fn cartesian_velocities_can_construct_from_array() {
        let array = [0., 1., 2., 3., 4., 5.];
        let cv = CartesianVelocities::new(array);
        assert_eq!(array, cv.O_dP_EE);
        assert!(!cv.has_elbow);
        assert!(cv.validate().is_ok());
    }

    // Port of TEST(CartesianVelocities, CanConstructFromArrayWithElbow).
    #[test]
    fn cartesian_velocities_can_construct_from_array_with_elbow() {
        let array = [0., 1., 2., 3., 4., 5.];
        let elbow = [0., 1.];
        let cv = CartesianVelocities::with_elbow(array, elbow);
        assert_eq!(array, cv.O_dP_EE);
        assert_eq!(elbow, cv.elbow);
        assert!(cv.has_elbow);
        assert!(cv.validate().is_ok());
    }

    // libfranka's hasElbow() is `elbow != decltype(elbow)()`.
    #[test]
    fn zero_elbow_is_no_elbow() {
        assert!(!CartesianPose::with_elbow(IDENTITY, [0., 0.]).has_elbow);
        assert!(!CartesianVelocities::with_elbow([0.; 6], [0., 0.]).has_elbow);
    }

    #[test]
    fn motion_finished_sets_the_flag() {
        assert!(motion_finished(Torques::new([0.; 7])).motion_finished);
        assert!(motion_finished(JointPositions::new([0.; 7])).motion_finished);
        assert!(motion_finished(JointVelocities::new([0.; 7])).motion_finished);
        assert!(motion_finished(CartesianPose::new(IDENTITY)).motion_finished);
        assert!(motion_finished(CartesianVelocities::new([0.; 6])).motion_finished);
    }

    #[test]
    fn motion_generator_kinds() {
        assert_eq!(JointPositions::KIND, MotionGeneratorKind::JointPosition);
        assert_eq!(JointVelocities::KIND, MotionGeneratorKind::JointVelocity);
        assert_eq!(CartesianPose::KIND, MotionGeneratorKind::CartesianPosition);
        assert_eq!(
            CartesianVelocities::KIND,
            MotionGeneratorKind::CartesianVelocity
        );
    }

    // checkFinite
    #[test]
    fn non_finite_values_are_rejected() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut q = [0.; 7];
            q[3] = bad;
            assert_eq!(
                message(Torques::new(q).validate()),
                "Commanding value is infinite or NaN."
            );
            assert_eq!(
                message(JointPositions::new(q).validate()),
                "Commanding value is infinite or NaN."
            );
            assert_eq!(
                message(JointVelocities::new(q).validate()),
                "Commanding value is infinite or NaN."
            );
            let mut dx = [0.; 6];
            dx[2] = bad;
            assert_eq!(
                message(CartesianVelocities::new(dx).validate()),
                "Commanding value is infinite or NaN."
            );
            let mut pose = IDENTITY;
            pose[0] = bad;
            assert_eq!(
                message(CartesianPose::new(pose).validate()),
                "Commanding value is infinite or NaN."
            );
        }
    }

    // checkMatrix / isHomogeneousTransformation
    #[test]
    fn invalid_transformation_is_rejected() {
        assert!(is_homogeneous_transformation(&IDENTITY));

        // Last row not (0, 0, 0, 1).
        let mut pose = IDENTITY;
        pose[3] = 1.0;
        assert!(!is_homogeneous_transformation(&pose));
        assert_eq!(
            message(CartesianPose::new(pose).validate()),
            "libfranka: Attempt to set invalid transformation in motion generator. Has to be \
             column major!"
        );

        for idx in [7usize, 11] {
            let mut pose = IDENTITY;
            pose[idx] = 1.0;
            assert!(!is_homogeneous_transformation(&pose));
        }
        let mut pose = IDENTITY;
        pose[15] = 0.0;
        assert!(!is_homogeneous_transformation(&pose));

        // Non-unit column.
        let mut pose = IDENTITY;
        pose[0] = 2.0;
        assert!(!is_homogeneous_transformation(&pose));

        // Non-unit row (column norms stay 1, row 0 norm does not).
        let pose = [
            0., 1., 0., 0., 1., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        ];
        assert!(is_homogeneous_transformation(&pose));
        let pose = [
            0.6, 0.8, 0., 0., 0.8, -0.6, 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.,
        ];
        assert!(is_homogeneous_transformation(&pose));

        // Just inside / outside the 1e-5 orthonormality threshold.
        let mut pose = IDENTITY;
        pose[0] = 1.0 + 9e-6;
        assert!(is_homogeneous_transformation(&pose));
        pose[0] = 1.0 + 2e-5;
        assert!(!is_homogeneous_transformation(&pose));
    }

    // checkElbow / isValidElbow
    #[test]
    fn invalid_elbow_is_rejected() {
        assert!(is_valid_elbow(&[0.5, 1.0]));
        assert!(is_valid_elbow(&[0.5, -1.0]));
        assert!(!is_valid_elbow(&[0.5, 0.5]));

        let expected = "Invalid elbow configuration given! Only +1 or -1 are allowed for the sign \
                        of the 4th joint.";
        assert_eq!(
            message(CartesianPose::with_elbow(IDENTITY, [0.5, 0.5]).validate()),
            expected
        );
        assert_eq!(
            message(CartesianVelocities::with_elbow([0.; 6], [0.5, 0.5]).validate()),
            expected
        );
        // Non-finite elbow is caught by checkFinite first.
        assert_eq!(
            message(CartesianPose::with_elbow(IDENTITY, [f64::NAN, 1.0]).validate()),
            "Commanding value is infinite or NaN."
        );
        // Without an elbow configuration the elbow is not checked.
        assert!(CartesianPose::new(IDENTITY).validate().is_ok());
        assert!(CartesianVelocities::new([0.; 6]).validate().is_ok());
    }
}
