//! End-to-end checks of the gripper client against the franka-sim simulator's gripper server
//! (port 1338).
//!
//! Run with `FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-rs --test sim_gripper`.

mod common;

use franka::error::FrankaError;
use franka::gripper::Gripper;
use franka_sim_test::SimConfig;

/// Tolerance used throughout: the sim's kinematic gripper model applies commanded widths
/// instantly and exactly, but the tolerance keeps these checks robust to future physical
/// backends.
const WIDTH_TOL: f64 = 0.005;

/// Homing, moving across the full stroke, and grasping in thin air, against the nominal sim
/// configuration (no object between the fingers).
#[test]
fn nominal_gripper_workflow() {
    let sim = common::sim(SimConfig::nominal());
    let gripper = Gripper::new(sim.host()).expect("gripper handshake failed");

    assert!(gripper.homing().expect("homing failed"));

    let state = gripper.read_once().expect("read_once after homing failed");
    assert!(
        (state.max_width - 0.08).abs() < WIDTH_TOL,
        "max_width = {}, expected ~0.08",
        state.max_width
    );

    assert!(
        gripper
            .move_gripper(0.04, 0.05)
            .expect("move to 0.04 failed"),
        "move to 0.04 should succeed"
    );
    let state = gripper.read_once().expect("read_once after move failed");
    assert!(
        (state.width - 0.04).abs() < WIDTH_TOL,
        "width = {}, expected ~0.04",
        state.width
    );

    // The extremes of the gripper's stroke (0..0.08) are still within range and should succeed.
    assert!(
        gripper.move_gripper(0.0, 0.05).expect("move to 0.0 failed"),
        "move to 0.0 (fully closed) should succeed"
    );
    let state = gripper
        .read_once()
        .expect("read_once after move to 0.0 failed");
    assert!(
        state.width.abs() < WIDTH_TOL,
        "width = {}, expected ~0.0",
        state.width
    );

    assert!(
        gripper
            .move_gripper(0.08, 0.05)
            .expect("move to 0.08 failed"),
        "move to 0.08 (fully open) should succeed"
    );
    let state = gripper
        .read_once()
        .expect("read_once after move to 0.08 failed");
    assert!(
        (state.width - 0.08).abs() < WIDTH_TOL,
        "width = {}, expected ~0.08",
        state.width
    );

    // No object is configured, so grasping must be unsuccessful.
    let grasped = gripper
        .grasp(0.04, 0.05, 10.0, 0.005, 0.005)
        .expect("grasp in thin air failed unexpectedly");
    assert!(!grasped, "grasp in thin air should report false");
    let state = gripper.read_once().expect("read_once after grasp failed");
    assert!(!state.is_grasped, "is_grasped should be false");

    assert!(gripper.stop().expect("stop failed"));

    drop(gripper);
    drop(sim);
}

/// Grasping a virtual object placed between the fingers (`--gripper-object-width 0.04`).
///
/// Also exercises a grasp request beyond the gripper's stroke (0..0.08 m). Whether the sim
/// answers `kFail` (`Err(FrankaError::Command)`) or `kUnsuccessful` (`Ok(false)`) is not
/// specified anywhere, so only the behaviour actually observed is asserted. Running this against
/// `franka-sim:dev` (container logs, `franka_sim.gripper.server`) shows:
///
/// ```text
/// ERROR franka_sim.gripper.server: Gripper command 2 failed: grasp width 0.09 m is outside the
/// 0..0.08 m stroke
/// ```
///
/// i.e. the sim raises an exception validating the request width against the stroke *before*
/// running the grasp, which its TCP handler catches and reports as `kFail` -- not
/// `kUnsuccessful`. So this asserts `Err(FrankaError::Command("libfranka gripper: Command
/// failed!"))`.
#[test]
fn grasp_with_object_between_the_fingers() {
    let sim = common::sim(SimConfig::nominal().with_gripper_object(0.04));
    let gripper = Gripper::new(sim.host()).expect("gripper handshake failed");

    let grasped = gripper
        .grasp(0.04, 0.05, 10.0, 0.02, 0.02)
        .expect("grasp on a matching object failed unexpectedly");
    assert!(grasped, "grasp on a matching object should report true");

    let state = gripper.read_once().expect("read_once after grasp failed");
    assert!(state.is_grasped, "is_grasped should be true");
    assert!(
        (state.width - 0.04).abs() < WIDTH_TOL,
        "width = {}, expected ~0.04",
        state.width
    );

    // Beyond the gripper's stroke (0..0.08 m): observed as `kFail` (`Err(Command)`), not
    // `kUnsuccessful` -- see the doc comment above for how this was determined.
    let error = gripper
        .grasp(0.09, 0.05, 10.0, 0.005, 0.005)
        .expect_err("out-of-stroke grasp should fail, matching the observed kFail status");
    assert_eq!(error.to_string(), "libfranka gripper: Command failed!");
    assert!(matches!(error, FrankaError::Command(_)));

    drop(gripper);
    drop(sim);
}
