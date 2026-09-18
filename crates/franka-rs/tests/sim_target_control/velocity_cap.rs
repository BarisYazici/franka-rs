//! The impedance backend's joint velocity cap and fade against franka-sim: the hand turned fast
//! about the direction joints 5 and 7 leave out once they are nearly aligned, where a turn asks
//! the most of them. The barrier does not engage in this run.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use franka::robot::target_control::max_joint_velocity;
use franka::{
    Backend, CartesianSent, Frame, ImpedanceGains, ImpedanceOptions, JointTargetControlOptions,
    Leash, OtgLimits, RobotMode, RobotState, TargetControlOptions,
};
use franka_sim_test::SimConfig;

use crate::{angle_between, common, note_max, rotation_about, wait_landed};

/// Joints 5 and 7 must start closer than this, degrees.
const ALIGNED_DEG: f64 = 25.0;
/// How far the hand turns and back, rad.
const TURN: f64 = 0.8;
/// The fewest cycles the cap must cut the goal in (88 measured on franka-sim 1.1.6 at the
/// default cap).
const CAPPED_CYCLES: u64 = 5;

/// Peaks from the realtime thread as `f64` bits, and the cycles the cap cut.
#[derive(Default)]
struct Peaks {
    measured_bits: AtomicU64,
    commanded_bits: AtomicU64,
    envelope_bits: AtomicU64,
    capped: AtomicU64,
}

fn peak(slot: &AtomicU64) -> f64 {
    f64::from_bits(slot.load(Ordering::Relaxed))
}

fn z_axis(pose: &[f64; 16]) -> [f64; 3] {
    [pose[8], pose[9], pose[10]]
}

fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn unit_cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    let c = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let norm = dot(&c, &c).sqrt();
    c.map(|x| x / norm)
}

/// 8. Impedance backend at the simulator's home pose with joint 6 at 2.8 rad, where joints 5
///    and 7 are within [`ALIGNED_DEG`], then the hand turned [`TURN`] rad about `z5 × z6` and
///    back under the teleop budgets. The goal never outruns the cap, the cap engages, every
///    measured joint stays under 0.85 of its limit (the law's push on an arm catching up on its
///    lag fades out from 0.7 of it), and the stop lands without an error.
#[test]
fn a_fast_wrist_turn_into_alignment_stays_under_the_velocity_limits() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    // The node's thresholds: a fast wrist in torque mode is not a collision.
    robot
        .set_collision_behavior_simple([40.0; 7], [40.0; 7], [40.0; 6], [40.0; 6])
        .expect("set_collision_behavior_simple failed");
    common::move_to_start(&robot);
    let model = robot.load_model().expect("load_model failed");

    let mut aligned = common::SIM_HOME_POSE;
    aligned[5] = 2.8;
    // Joint 6 travels 1.23 rad, past the default 1.0 rad deviation guard.
    let joints = robot
        .start_joint_target_control(JointTargetControlOptions::default().with_max_deviation(1.5))
        .expect("start_joint_target_control failed");
    joints.set_joints(aligned).expect("set_joints failed");
    let state = wait_landed(
        4.0,
        || joints.state(),
        |s| s.q.iter().zip(&aligned).all(|(a, b)| (a - b).abs() < 0.01),
    );
    joints.stop().expect("the joint session's stop failed");
    let z5 = z_axis(&model.pose(Frame::Joint5, &state));
    let z6 = z_axis(&model.pose(Frame::Joint6, &state));
    let z7 = z_axis(&model.pose(Frame::Joint7, &state));
    let apart = dot(&z5, &z7).abs().min(1.0).acos().to_degrees();
    eprintln!("wrist: joints 5 and 7 start {apart:.1} degrees apart");
    assert!(
        apart < ALIGNED_DEG,
        "the wrist is not aligned: {apart} degrees"
    );

    let preset = ImpedanceGains::CARTESIAN;
    let ratio: f64 = 1200.0 / preset.cartesian_stiffness[0];
    let gains = ImpedanceGains {
        cartesian_stiffness: preset.cartesian_stiffness.map(|k| k * ratio),
        cartesian_damping: preset.cartesian_damping.map(|d| d * ratio.sqrt()),
        ..preset
    };
    let leash = Leash {
        translation: 0.03,
        rotation: 0.25,
        ..Leash::default()
    };
    let impedance = ImpedanceOptions::cartesian()
        .with_gains(gains)
        .with_leash(leash);
    let limits = max_joint_velocity(robot.fci_version());
    let caps = limits.map(|v| v * impedance.joint_velocity_fraction);
    let peaks = Arc::new(Peaks::default());
    let observer = {
        let peaks = Arc::clone(&peaks);
        move |state: &RobotState, sent: &CartesianSent| {
            let measured = (0..7)
                .map(|i| state.dq[i].abs() / limits[i])
                .fold(0.0, f64::max);
            let commanded = (0..7)
                .map(|i| sent.dq_goal[i].abs() / caps[i])
                .fold(0.0, f64::max);
            let envelope = sent
                .tau_envelope
                .iter()
                .fold(0.0, |m: f64, t| m.max(t.abs()));
            note_max(&peaks.measured_bits, measured);
            note_max(&peaks.commanded_bits, commanded);
            note_max(&peaks.envelope_bits, envelope);
            if sent.cap_scale < 1.0 {
                peaks.capped.fetch_add(1, Ordering::Relaxed);
            }
        }
    };
    let options = TargetControlOptions::default()
        .with_limits(OtgLimits {
            max_velocity: 1.0,
            max_acceleration: 8.0,
            max_jerk: 400.0,
        })
        .with_rotation_limits(OtgLimits {
            max_velocity: 4.0,
            max_acceleration: 20.0,
            max_jerk: 500.0,
        })
        .with_max_deviation(0.6)
        .with_max_angular_deviation(2.5)
        .with_backend(Backend::Impedance(impedance))
        .with_observer(observer);
    let control = robot
        .start_cartesian_target_control(options)
        .expect("start_cartesian_target_control failed");
    let start = control.target_pose();
    let mut turned = common::matrix_multiply(&rotation_about(unit_cross(&z5, &z6), TURN), &start);
    turned[12..15].copy_from_slice(&control.target());
    control.set_pose(&turned).expect("set_pose failed");
    std::thread::sleep(Duration::from_millis(1500));
    control.set_pose(&start).expect("set_pose failed");
    std::thread::sleep(Duration::from_millis(2000));
    control.stop().expect("stop() returned the loop's error");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    assert!(
        !state.current_errors.any() && !state.last_motion_errors.any(),
        "error flags raised: {} / {}",
        state.current_errors,
        state.last_motion_errors
    );
    let back = angle_between(&model.pose(Frame::EndEffector, &state), &start);
    let (measured, commanded) = (peak(&peaks.measured_bits), peak(&peaks.commanded_bits));
    let (envelope, capped) = (
        peak(&peaks.envelope_bits),
        peaks.capped.load(Ordering::Relaxed),
    );
    eprintln!(
        "wrist: cap engaged in {capped} cycles, peak |dq_goal| / cap {commanded:.3}, peak \
         |dq| / limit {measured:.3}, peak |tau_envelope| {envelope:.2} Nm, back within \
         {back:.2e} rad"
    );
    assert!(
        capped >= CAPPED_CYCLES,
        "the turn reached the cap in {capped} cycles"
    );
    assert!(
        commanded <= 1.0 + 1e-9,
        "the goal outran the cap: {commanded}"
    );
    assert!(measured < 0.85, "a joint ran at {measured} of its limit");
    assert!(back < 1e-2, "the hand did not come back: {back} rad");
}
