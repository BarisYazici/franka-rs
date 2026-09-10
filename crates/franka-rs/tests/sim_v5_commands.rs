//! The FCI v5 TCP command set, model library and gripper of a Franka Emika Robot (FER), against the
//! franka-sim simulator running `--protocol v5 --robot panda`.
//!
//! Run under the machine-wide simulator lock:
//!
//! ```text
//! flock .sim.lock cargo test --release -p franka-rs --test sim_v5_commands
//! ```
//!
//! The image comes from [`SimConfig::fer_v5`]: `$FRANKA_SIM_FER_IMAGE`, else
//! `franka-sim:panda-v5`. `FRANKA_SIM_IMAGE` names the **FR3** image and is deliberately
//! ignored here.
//!
//! The arm checks all share one container and one FCI session, because the simulator resets
//! its session state when the TCP client disconnects and only one FCI client slot exists. The
//! gripper checks get their own containers: the gripper server is a separate port (1338) but
//! the harness still starts one simulator per [`SimServer`].

mod common;

use franka::error::FrankaError;
use franka::gripper::Gripper;
use franka::{FciVersion, Frame, RobotMode};
use franka_sim_test::SimConfig;

/// A 4x4 column-major transform with a pure translation.
fn translation_transform(x: f64, y: f64, z: f64) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, z, 1.0,
    ]
}

/// Tolerance for the gripper width checks, as in `sim_gripper.rs`.
const WIDTH_TOL: f64 = 0.005;

#[test]
fn commands_and_model_against_the_fer_simulator() {
    let sim = common::sim(SimConfig::fer_v5());
    let robot = common::connect(&sim);
    assert_eq!(robot.fci_version(), FciVersion::V5);
    assert_eq!(robot.server_version(), 5);
    // Establish the start pose rather than assuming it.
    common::move_to_start_fer(&robot);

    // The simulator is idle in its home pose with a usable EE transform.
    let state = robot.read_once().expect("read_once failed");
    eprintln!(
        "fer sim: robot_mode {:?}, q {:?}",
        state.robot_mode, state.q
    );
    assert_eq!(state.robot_mode, RobotMode::Idle);
    for (index, (actual, expected)) in state
        .q
        .iter()
        .zip(common::FER_SIM_HOME_POSE.iter())
        .enumerate()
    {
        assert!(
            (actual - expected).abs() < 0.5,
            "q[{index}] = {actual}, expected about {expected}"
        );
    }
    assert!(
        state.F_T_EE[15] == 1.0
            && (state.F_T_EE[0].abs() + state.F_T_EE[5].abs() + state.F_T_EE[10].abs()) > 0.5,
        "F_T_EE is degenerate: {:?}",
        state.F_T_EE
    );

    // Every setter that exists on FCI v5 is accepted. The command ids differ from v10's (e.g.
    // SetCollisionBehavior is 4 here and 5 there); an `Ok(())` is therefore also evidence that
    // the v5 numbering reaches the right handler.
    robot
        .set_collision_behavior(
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        )
        .expect("set_collision_behavior");
    robot
        .set_collision_behavior_simple(
            [20.0; 7],
            [20.0; 7],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        )
        .expect("set_collision_behavior_simple");
    robot
        .set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])
        .expect("set_joint_impedance");
    robot
        .set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])
        .expect("set_cartesian_impedance");
    robot
        .set_guiding_mode([true, true, true, true, true, true], false)
        .expect("set_guiding_mode");
    robot.set_load(0.0, [0.0; 3], [0.0; 9]).expect("set_load");

    // set_k is echoed in EE_T_K.
    let ee_t_k = translation_transform(0.0, 0.0, 0.05);
    robot.set_k(ee_t_k).expect("set_k");
    let state = common::settled_state(&robot, 20);
    for (index, (actual, expected)) in state.EE_T_K.iter().zip(ee_t_k.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 1e-6,
            "EE_T_K[{index}] = {actual}, expected {expected}"
        );
    }

    // set_ee makes F_T_EE the product F_T_NE * NE_T_EE.
    let ne_t_ee = translation_transform(0.0, 0.0, 0.1);
    robot.set_ee(ne_t_ee).expect("set_ee");
    let state = common::settled_state(&robot, 20);
    let expected = common::matrix_multiply(&state.F_T_NE, &ne_t_ee);
    for (index, (actual, expected)) in state.F_T_EE.iter().zip(expected.iter()).enumerate() {
        assert!(
            (actual - expected).abs() < 1e-5,
            "F_T_EE[{index}] = {actual}, expected {expected}"
        );
    }
    // Restore the identity end effector for the model check below.
    robot
        .set_ee(translation_transform(0.0, 0.0, 0.0))
        .expect("set_ee (reset)");
    robot
        .set_k(translation_transform(0.0, 0.0, 0.0))
        .expect("set_k (reset)");

    // Error recovery is accepted while the robot is idle.
    robot
        .automatic_error_recovery()
        .expect("automatic_error_recovery");

    // `SetFilters` is v5-only and has no v10 counterpart at all.
    let filters = robot.set_filters(100.0, 100.0, 100.0, 100.0, 100.0);
    eprintln!("fer sim: set_filters answered {filters:?}");
    filters.expect("set_filters failed");

    // `GetCartesianLimit` is v5-only. The simulator implements the framing but not the
    // content: it answers `kSuccess` with an all-zero 154-byte body. These are therefore
    // *characterisation* assertions on a simulator stub, not on a real FER's virtual walls.
    let wall = robot
        .virtual_wall(1)
        .expect("virtual_wall(1) failed on FCI v5");
    eprintln!("fer sim: virtual_wall(1) answered {wall:?}");
    assert_eq!(wall.id, 1, "the id is echoed from the request");
    assert_eq!(
        wall.object_world_size, [0.0; 3],
        "franka-sim answers GetCartesianLimit with an all-zero payload; update this \
         characterisation assertion if the simulator grew real virtual walls"
    );
    assert_eq!(wall.p_frame, [0.0; 16], "the stub's object_frame is zero");
    assert!(!wall.active, "the stub's object_activation is zero");

    // `GetRobotModel` does not exist on FCI v5; the client rejects it without sending a byte.
    match robot.robot_model() {
        Err(FrankaError::InvalidOperation(message)) => assert_eq!(
            message,
            "libfranka: Get Robot Model is not available on FCI version 5."
        ),
        other => panic!("expected InvalidOperation, got {other:?}"),
    }

    // `LoadModelLibrary` downloads `libfcimodels.so`, which is dlopen'd and evaluated.
    let model = robot
        .load_model_from_robot()
        .expect("load_model_from_robot failed on FCI v5");
    let state = common::settled_state(&robot, 20);

    let pose = model.pose(Frame::EndEffector, &state);
    let joint7 = model.pose(Frame::Joint7, &state);
    let flange = model.pose(Frame::Flange, &state);
    eprintln!("fer sim: O_T_EE {:?}", state.O_T_EE);
    eprintln!("fer sim: model.pose(EndEffector) {pose:?}");
    eprintln!("fer sim: model.pose(Joint7)      {joint7:?}");
    eprintln!("fer sim: model.pose(Flange)      {flange:?}");

    // The orientation agrees with the pose the simulator publishes.
    for (index, (actual, expected)) in pose.iter().zip(state.O_T_EE.iter()).enumerate().take(12) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "model.pose(EndEffector)[{index}] = {actual}, O_T_EE[{index}] = {expected}"
        );
    }

    // The *origin* does not: exactly as in the FR3 simulator (see `sim_commands.rs`), the
    // `O_T_EE` franka-sim publishes sits at the joint-7 frame, short of the 0.107 m
    // joint7 -> flange offset along the tool z axis, with `F_T_EE` and `NE_T_EE` both the
    // identity. Characterisation assertions, pinning the simulator's behaviour:
    for (index, (actual, expected)) in joint7
        .iter()
        .zip(state.O_T_EE.iter())
        .enumerate()
        .skip(12)
        .take(3)
    {
        assert!(
            (actual - expected).abs() < 1e-3,
            "the simulator's O_T_EE origin is not the joint-7 origin: \
             model.pose(Joint7)[{index}] = {actual}, O_T_EE[{index}] = {expected}"
        );
    }
    let mut offset = 0.0;
    for index in 0..3 {
        offset += (pose[12 + index] - state.O_T_EE[12 + index]) * pose[8 + index];
    }
    eprintln!("fer sim: EndEffector - O_T_EE along the tool z axis = {offset} m");
    assert!(
        (offset.abs() - 0.107).abs() < 1e-3,
        "expected the same constant 0.107 m flange offset the FR3 simulator shows, \
         measured {offset} m"
    );

    // The dynamics terms evaluate at the home pose.
    let gravity = model.gravity(&state);
    let coriolis = model.coriolis(&state);
    let mass = model.mass(&state);
    eprintln!("fer sim: gravity {gravity:?}");
    eprintln!("fer sim: coriolis {coriolis:?}");
    assert!(
        gravity.iter().all(|value| value.is_finite()),
        "gravity is not finite: {gravity:?}"
    );
    assert!(
        coriolis.iter().all(|value| value.is_finite()),
        "coriolis is not finite: {coriolis:?}"
    );
    assert!(
        mass.iter().all(|value| value.is_finite()),
        "the mass matrix is not finite: {mass:?}"
    );
    // The arm is nearly at rest at the home pose, so the Coriolis/centrifugal term is tiny
    // while gravity is not: joints 2, 4 and 6 carry the arm's weight.
    assert!(
        coriolis.iter().all(|value| value.abs() < 1.0),
        "the arm is at rest, so coriolis should be near zero: {coriolis:?}"
    );
    // `Model::gravity(&state)` uses the state's `O_ddP_O` as Earth's gravity, and franka-sim
    // publishes that field as a hard-coded zero vector (`franka_sim/protocols/v5.py`,
    // `"O_ddP_O": [0.0] * 3` -- the v10 simulator does the same), where a real robot reports
    // about [0, 0, -9.81]. So the *simulator* makes this identically zero; that is a
    // characterisation assertion, and the model symbol itself is checked below with an
    // explicit gravity vector.
    assert_eq!(
        state.O_ddP_O, [0.0; 3],
        "franka-sim publishes O_ddP_O as zero; update this characterisation assertion if the \
         simulator started reporting real gravity"
    );
    assert_eq!(
        gravity, [0.0; 7],
        "with O_ddP_O = 0 the gravity torques must be zero: {gravity:?}"
    );

    let real_gravity = model.gravity_q(
        &state.q,
        state.m_total,
        &state.F_x_Ctotal,
        &[0.0, 0.0, -9.81],
    );
    eprintln!("fer sim: gravity_q with [0, 0, -9.81] = {real_gravity:?}");
    assert!(
        real_gravity.iter().all(|value| value.is_finite()),
        "gravity_q is not finite: {real_gravity:?}"
    );
    let worst_gravity = real_gravity
        .iter()
        .fold(0.0_f64, |worst, g| worst.max(g.abs()));
    assert!(
        worst_gravity > 1.0,
        "under real gravity the home pose must load the arm: {real_gravity:?}"
    );
    // The mass matrix is symmetric and positive on the diagonal.
    for row in 0..7 {
        assert!(
            mass[row * 7 + row] > 0.0,
            "M[{row}][{row}] = {} is not positive",
            mass[row * 7 + row]
        );
        for column in 0..7 {
            assert!(
                (mass[row * 7 + column] - mass[column * 7 + row]).abs() < 1e-9,
                "the mass matrix is not symmetric at ({row}, {column})"
            );
        }
    }
}

/// The gripper protocol is byte-identical on FCI v5 and v10 (version 3, port 1338), so this is
/// `sim_gripper.rs`'s `nominal_gripper_workflow` against the FER image.
///
/// The `franka-sim:panda-v5` image is an older snapshot of franka-sim than `franka-sim:dev`
/// and has neither the `--gripper-object-width` flag nor the
/// `FRANKA_SIM_GRIPPER_OBJECT_WIDTH` environment variable (verified by grepping
/// `run_server.py` inside both images), so [`SimConfig::with_gripper_object`] cannot be used
/// with it -- `docker run` would fail with `unrecognized arguments`. A successful grasp is
/// therefore only covered by `sim_gripper.rs` on the FR3 image; see `docs/book/src/reference/simulator-gaps.md`.
#[test]
fn gripper_against_the_fer_simulator() {
    let sim = common::sim(SimConfig::fer_v5());
    let gripper = Gripper::new(sim.host()).expect("gripper handshake failed");

    assert!(gripper.homing().expect("homing failed"));

    let state = gripper.read_once().expect("read_once after homing failed");
    eprintln!(
        "fer sim gripper: max_width {}, width {}",
        state.max_width, state.width
    );
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

    // No object is configured (and this image cannot configure one), so grasping must be
    // unsuccessful rather than an error.
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
