//! The TCP command set of [`franka::Robot`] against the franka-sim simulator.
//!
//! Run with `FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-rs --test sim_commands`.
//!
//! All checks share one container and one FCI session: the simulator resets its session state
//! when the TCP client disconnects, so the `set_k` / `set_ee` echoes are only observable while
//! the same `Robot` stays connected.

mod common;

use franka::{Frame, RobotMode};
use franka_sim_test::SimConfig;

/// A 4x4 column-major transform with a pure translation.
fn translation_transform(x: f64, y: f64, z: f64) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, z, 1.0,
    ]
}

#[test]
fn commands_and_model_against_the_simulator() {
    let sim = common::sim(SimConfig::nominal());
    let robot = common::connect(&sim);
    // Establish the start pose rather than assuming it: against a persistent server (CI's
    // `FRANKA_SIM_ADDR`) the arm is wherever the previous test left it.
    common::move_to_start(&robot);

    // read_once: the simulator starts idle in its home pose with a usable EE transform.
    // The pose is franka-sim's own `home` keyframe, not libfranka's ready pose (see
    // `common::SIM_HOME_POSE`).
    let state = robot.read_once().expect("read_once failed");
    assert_eq!(state.robot_mode, RobotMode::Idle);
    for (index, (actual, expected)) in state.q.iter().zip(common::SIM_HOME_POSE.iter()).enumerate()
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

    // Every setter is accepted.
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

    // The URDF names the FR3.
    let urdf = robot.robot_model().expect("robot_model");
    assert!(
        urdf.contains("fr3"),
        "the URDF does not mention fr3: {}",
        &urdf[..urdf.len().min(200)]
    );

    // The model's forward kinematics agree with the pose the simulator publishes.
    //
    // Two caveats, both observed against franka-sim:
    //  * the orientation matches to better than 1e-7, but the *origin* the simulator publishes
    //    as `O_T_EE` is the joint-7 frame, not the flange: it is short of the FR3's 0.107 m
    //    joint7 -> flange offset along the tool z-axis (which points down here). The raw
    //    element-wise difference between `model.pose(EndEffector)` and `O_T_EE` is therefore
    //    0.1070 m, with `F_T_EE` and `NE_T_EE` both the identity.
    //  * consequently the check below compares the rotation directly, the translation against
    //    `Frame::Joint7`, and the flange offset explicitly, instead of loosening the tolerance.
    let model = robot.load_model().expect("load_model");
    let state = common::settled_state(&robot, 20);
    let pose = model.pose(Frame::EndEffector, &state);
    for (index, (actual, expected)) in pose.iter().zip(state.O_T_EE.iter()).enumerate().take(12) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "model.pose(EndEffector)[{index}] = {actual}, O_T_EE[{index}] = {expected}"
        );
    }

    let joint7 = model.pose(Frame::Joint7, &state);
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

    // The whole disagreement is the constant flange offset along the tool z-axis.
    let mut offset = 0.0;
    for index in 0..3 {
        offset += (pose[12 + index] - state.O_T_EE[12 + index]) * pose[8 + index];
    }
    assert!(
        (offset.abs() - 0.107).abs() < 1e-3,
        "expected a constant 0.107 m flange offset, measured {offset} m"
    );
}
