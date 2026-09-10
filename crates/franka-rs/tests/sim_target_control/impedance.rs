//! The impedance backend (the default) of both target-control interfaces against franka-sim:
//! the arm is still at the start of a session, lands on every held target through the crate's
//! torques, and tracks a moving goal tighter with the velocity feedforward.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use franka::{
    Backend, CartesianSent, Frame, FrankaError, ImpedanceOptions, JointSent,
    JointTargetControlOptions, Model, RobotMode, RobotState, TargetControlOptions,
};
use franka_sim_test::SimConfig;

use crate::{add, angle_between, common, distance, norm, note_max, rotation_about, wait_landed};

fn peak_abs(v: &[f64; 7]) -> f64 {
    v.iter().fold(0.0, |peak, x| peak.max(x.abs()))
}

fn max_abs_difference(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

/// The measured position may land this far from a held target under the impedance backend, m:
/// the law's steady state against the simulator's joint friction (measured on franka-sim
/// 1.1.6: 0.2 to 0.5 mm).
const IMPEDANCE_LANDING: f64 = 5e-3;
/// ... and the measured orientation, rad. Measured on franka-sim 1.1.6: 5.0e-3, where the
/// DROID gains' 25 Nm/rad about the tool z (15 Cartesian, 10 on joint 7) stall against the
/// model's 0.25 Nm of Coulomb friction on joint 7.
const IMPEDANCE_ANGULAR_LANDING: f64 = 1e-2;
/// Cycles at the start of a session in which no joint may move more than [`STILL`] rad: the
/// first torque is that of rest, whatever `O_T_EE` and the model disagree by.
const STILL_CYCLES: u64 = 500;
const STILL: f64 = 2e-3;

/// The end-effector pose the impedance backend works in: the model's pose of the measured
/// configuration. franka-sim's `O_T_EE` is the joint-7 frame, 0.107 m short of it along the
/// tool z (see `docs/book/src/reference/simulator-gaps.md`); on a robot the two coincide.
fn measured_ee(model: &Model, state: &RobotState) -> [f64; 16] {
    model.pose(Frame::EndEffector, state)
}

/// Sleeps `hold` seconds for the generator to land and the arm to settle on it, then polls
/// like [`wait_landed`] so a loaded simulator gets its slack.
fn settle_then_wait(
    hold: f64,
    state: impl FnMut() -> RobotState,
    landed: impl Fn(&RobotState) -> bool,
) -> RobotState {
    std::thread::sleep(Duration::from_secs_f64(hold));
    wait_landed(0.0, state, landed)
}

/// What the impedance backend's observer tracks: the cycles, the largest excursion of any
/// joint from the first cycle's `q` within the first [`STILL_CYCLES`], the peak IK residual
/// and the peak torque, the peaks as `f64` bits.
#[derive(Default)]
struct TorqueCounters {
    cycles: AtomicU64,
    stillness_bits: AtomicU64,
    peak_ik_error_bits: AtomicU64,
    peak_torque_bits: AtomicU64,
}

impl TorqueCounters {
    /// One cycle; `q_start` is the observer's own memory of the first cycle's `q`.
    fn note(
        &self,
        state: &RobotState,
        q_start: &mut Option<[f64; 7]>,
        ik_error: f64,
        tau: &[f64; 7],
    ) {
        let cycle = self.cycles.fetch_add(1, Ordering::Relaxed);
        let start = *q_start.get_or_insert(state.q);
        if cycle < STILL_CYCLES {
            note_max(&self.stillness_bits, max_abs_difference(&state.q, &start));
        }
        note_max(&self.peak_ik_error_bits, ik_error);
        note_max(&self.peak_torque_bits, peak_abs(tau));
    }
    fn stillness(&self) -> f64 {
        f64::from_bits(self.stillness_bits.load(Ordering::Relaxed))
    }
    fn peak_ik_error(&self) -> f64 {
        f64::from_bits(self.peak_ik_error_bits.load(Ordering::Relaxed))
    }
    fn peak_torque(&self) -> f64 {
        f64::from_bits(self.peak_torque_bits.load(Ordering::Relaxed))
    }

    /// The first [`STILL_CYCLES`] have passed and the arm did not move.
    fn assert_still(&self, what: &str) {
        assert!(
            self.cycles.load(Ordering::Relaxed) > STILL_CYCLES,
            "{what}: the first target came before {STILL_CYCLES} cycles had passed"
        );
        let stillness = self.stillness();
        eprintln!("{what}: the arm moved {stillness:.2e} rad in the first {STILL_CYCLES} cycles");
        assert!(
            stillness < STILL,
            "{what}: the arm moved {stillness} rad in the first {STILL_CYCLES} cycles"
        );
    }
}

/// The time between the start of the session and the first target: [`STILL_CYCLES`] and the
/// slack a loaded simulator needs to publish that many states.
fn wait_still() {
    std::thread::sleep(Duration::from_millis(STILL_CYCLES + 200));
}

/// `stop()`, timed and printed.
fn timed_stop(what: &str, stop: impl FnOnce() -> franka::FrankaResult<()>) -> Duration {
    let stopped_at = Instant::now();
    stop().expect("stop() returned the loop's error");
    let stop_time = stopped_at.elapsed();
    eprintln!("{what}: stop() took {stop_time:?}");
    stop_time
}

/// 4. Cartesian, impedance backend (the default): still at the start, then the stepped
///    commander of test 1 followed with the crate's torques; the measured pose lands within
///    [`IMPEDANCE_LANDING`] of every held target, the IK residual stays small, `stop()` Ok.
#[test]
fn cartesian_impedance_backend_follows_a_stepped_commander() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let counters = Arc::new(TorqueCounters::default());
    let observer = {
        let counters = Arc::clone(&counters);
        let mut q_start = None;
        move |state: &RobotState, sent: &CartesianSent| {
            counters.note(state, &mut q_start, sent.ik_error, &sent.tau)
        }
    };
    let model = robot.load_model().expect("load_model failed");
    // A posture past a joint limit is refused at the start, naming the joint.
    let mut past = robot.read_once().expect("read_once failed").q;
    past[0] = 3.0;
    let refused =
        robot.start_cartesian_target_control(TargetControlOptions::default().with_backend(
            Backend::Impedance(ImpedanceOptions::cartesian().with_posture(Some(past))),
        ));
    assert!(
        matches!(refused, Err(FrankaError::InvalidArgument(ref m)) if m.contains("posture puts joint 1")),
        "a posture past joint 1's limit was not refused: {:?}",
        refused.map(|_| ())
    );
    let options = TargetControlOptions::default().with_observer(observer);
    assert!(matches!(options.backend, Backend::Impedance(_)));
    let control = robot
        .start_cartesian_target_control(options)
        .expect("start_cartesian_target_control failed");
    let start = control.target();
    let measured_start = common::translation(&measured_ee(&model, &control.state()));
    assert!(
        distance(&start, &measured_start) < 1e-6,
        "the start target is the model's pose of the first cycle's q: {start:?} vs \
         {measured_start:?}"
    );
    let ee = |s: &RobotState| common::translation(&measured_ee(&model, s));
    wait_still();
    counters.assert_still("cartesian");

    // Steps with holds long enough to land (0.85 s for 5 cm under the per-axis budget) and
    // for the arm to settle on the landed pose.
    let script: [([f64; 3], f64); 4] = [
        ([0.05, 0.0, 0.0], 1.5),
        ([0.05, 0.05, 0.0], 1.5),
        ([0.05, 0.05, 0.05], 1.5),
        ([0.0, 0.0, 0.0], 1.5),
    ];
    for (relative, hold) in script {
        let target = add(&start, &relative);
        control.set_position(target).expect("set_position failed");
        let state = settle_then_wait(
            hold,
            || control.state(),
            |s| distance(&ee(s), &target) < IMPEDANCE_LANDING,
        );
        let error = distance(&ee(&state), &target);
        eprintln!("cartesian: landed {:.2} mm from {relative:?}", error * 1e3);
        assert!(
            error < IMPEDANCE_LANDING,
            "the arm did not land on {relative:?}: {error} m off"
        );
        assert!(control.is_running(), "the loop ended early");
    }
    for i in 0..20 {
        let toggled = add(&start, &[0.05 * f64::from(i % 2 == 0), 0.0, 0.0]);
        control
            .set_position(toggled)
            .expect("burst set_position failed");
        std::thread::sleep(Duration::from_millis(5));
    }
    std::thread::sleep(Duration::from_millis(1500));
    assert!(control.is_running(), "the loop ended during the burst");
    control
        .set_position(start)
        .expect("final set_position failed");

    let stop_time = timed_stop("cartesian", || control.stop());
    assert!(
        stop_time < Duration::from_secs(4),
        "stop() took {stop_time:?}"
    );
    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    assert!(
        !state.current_errors.any() && !state.last_motion_errors.any(),
        "error flags raised: {} / {}",
        state.current_errors,
        state.last_motion_errors
    );
    let error = distance(&ee(&state), &start);
    eprintln!("cartesian: back at the start within {:.2} mm", error * 1e3);
    assert!(
        error < IMPEDANCE_LANDING,
        "the arm did not return to the start: {error} m off"
    );
    let cycles = counters.cycles.load(Ordering::Relaxed);
    assert!(cycles > 2000, "only {cycles} cycles observed");
    let (ik_error, torque) = (counters.peak_ik_error(), counters.peak_torque());
    eprintln!("cartesian: peak IK residual {ik_error:.2e}, peak torque {torque:.2} Nm");
    assert!(
        ik_error < 1e-3,
        "the IK fell behind the pose: residual {ik_error}"
    );
    assert!(torque > 0.1, "no torque was ever sent");
    robot.read_once().expect("read_once after stop");
}

/// 5. Joint, impedance backend (the default): still at the start, every target reached within
///    0.02 rad, `stop()` Ok, the arm back at the start.
#[test]
fn joint_impedance_backend_reaches_targets() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let counters = Arc::new(TorqueCounters::default());
    let observer = {
        let counters = Arc::clone(&counters);
        let mut q_start = None;
        move |state: &RobotState, sent: &JointSent| {
            counters.note(state, &mut q_start, 0.0, &sent.tau)
        }
    };
    let options = JointTargetControlOptions::default().with_observer(observer);
    assert!(matches!(options.backend, Backend::Impedance(_)));
    let control = robot
        .start_joint_target_control(options)
        .expect("start_joint_target_control failed");
    let start = control.target();
    assert!(
        max_abs_difference(&start, &control.state().q) < 1e-6,
        "the start target is the first cycle's measured q"
    );
    wait_still();
    counters.assert_still("joint");
    // A target past a joint limit is refused, naming the joint, and changes nothing.
    let mut past = start;
    past[3] = -3.2;
    let refused = control.set_joints(past);
    assert!(
        matches!(refused, Err(FrankaError::InvalidArgument(ref m)) if m.contains("target puts joint 4")),
        "a target past joint 4's limit was not refused: {refused:?}"
    );
    assert_eq!(control.target(), start);

    // Holds long enough for the generator to land and the joint PD to settle behind it: at
    // 1 rad/s joint 7's gains (50 Nm/rad, 15 Nm s/rad) lag the generator by a tenth of a
    // radian and converge with a 0.3 s time constant (measured on franka-sim 1.1.6: 19 mrad
    // left after 1.5 s).
    let mut target = start;
    let script: [(usize, f64, f64); 4] =
        [(0, 0.3, 1.5), (3, -0.3, 1.5), (6, 0.4, 3.0), (0, -0.3, 1.5)];
    for (joint, delta, hold) in script {
        target[joint] += delta;
        control.set_joints(target).expect("set_joints failed");
        let state = settle_then_wait(
            hold,
            || control.state(),
            |s| max_abs_difference(&s.q, &target) < 0.02,
        );
        let error = max_abs_difference(&state.q, &target);
        eprintln!("joint: joint {joint} step settled {error:.4} rad off");
        assert!(
            error < 0.02,
            "the arm did not reach the target: q {:?}, target {target:?}",
            state.q
        );
        assert!(control.is_running(), "the loop ended early");
    }
    // Back to the start with the same settle time before the stop.
    control.set_joints(start).expect("final set_joints failed");
    std::thread::sleep(Duration::from_millis(1500));
    timed_stop("joint", || control.stop());

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    let error = max_abs_difference(&state.q, &start);
    eprintln!("joint: back at the start within {error:.4} rad");
    assert!(
        error < 0.02,
        "the arm did not return to the start: {:?}",
        state.q
    );
    assert!(counters.cycles.load(Ordering::Relaxed) > 2000);
    assert!(counters.peak_torque() > 0.1);
    robot.read_once().expect("read_once after stop");
}

/// 6. Cartesian pose, impedance backend: the rotation of test 3 reached within
///    [`IMPEDANCE_ANGULAR_LANDING`] rad with the position held, a position step keeps the
///    orientation, the stop turns back, no error flag raised.
#[test]
fn impedance_backend_reaches_a_rotated_pose() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let counters = Arc::new(TorqueCounters::default());
    let observer = {
        let counters = Arc::clone(&counters);
        let mut q_start = None;
        move |state: &RobotState, sent: &CartesianSent| {
            counters.note(state, &mut q_start, sent.ik_error, &sent.tau)
        }
    };
    let model = robot.load_model().expect("load_model failed");
    let control = robot
        .start_cartesian_target_control(TargetControlOptions::default().with_observer(observer))
        .expect("start_cartesian_target_control failed");
    let start = control.target_pose();
    let start_position = control.target();
    let start_orientation = control.target_orientation();
    let ee = |s: &RobotState| measured_ee(&model, s);

    // R_target = Rx(10 deg) * R_start * Rz(20 deg), at the start position.
    let tool_z =
        common::matrix_multiply(&start, &rotation_about([0.0, 0.0, 1.0], 20f64.to_radians()));
    let mut target = common::matrix_multiply(
        &rotation_about([1.0, 0.0, 0.0], 10f64.to_radians()),
        &tool_z,
    );
    target[12..15].copy_from_slice(&start_position);
    control.set_pose(&target).expect("set_pose failed");
    let state = settle_then_wait(
        3.0,
        || control.state(),
        |s| angle_between(&ee(s), &target) < IMPEDANCE_ANGULAR_LANDING,
    );
    let (turned, drifted) = (
        angle_between(&ee(&state), &target),
        distance(&common::translation(&ee(&state)), &start_position),
    );
    eprintln!("rotation: landed {turned:.2e} rad off, position drifted {drifted:.2e} m");
    assert!(
        turned < IMPEDANCE_ANGULAR_LANDING,
        "the arm did not turn to the target: {turned} rad off"
    );
    assert!(
        drifted < IMPEDANCE_LANDING,
        "the position moved {drifted} m during a pure rotation"
    );

    let stepped = add(&start_position, &[0.03, 0.0, 0.0]);
    control.set_position(stepped).expect("set_position failed");
    let state = settle_then_wait(
        1.5,
        || control.state(),
        |s| distance(&common::translation(&ee(s)), &stepped) < IMPEDANCE_LANDING,
    );
    assert!(
        distance(&common::translation(&ee(&state)), &stepped) < IMPEDANCE_LANDING,
        "the arm did not follow the position step"
    );
    assert!(
        angle_between(&ee(&state), &target) < IMPEDANCE_ANGULAR_LANDING,
        "set_position turned the arm"
    );
    // Back to the start, settled before the stop (see the joint test).
    control
        .set_target(start_position, start_orientation)
        .expect("set_target failed");
    std::thread::sleep(Duration::from_millis(2500));
    timed_stop("rotation", || control.stop());

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    assert!(
        !state.current_errors.any() && !state.last_motion_errors.any(),
        "error flags raised: {} / {}",
        state.current_errors,
        state.last_motion_errors
    );
    let (turned, drifted) = (
        angle_between(&ee(&state), &start),
        distance(&common::translation(&ee(&state)), &start_position),
    );
    eprintln!("rotation: back at the start within {turned:.2e} rad and {drifted:.2e} m");
    assert!(
        turned < IMPEDANCE_ANGULAR_LANDING && drifted < IMPEDANCE_LANDING,
        "did not return to the start: {turned} rad, {drifted} m off"
    );
    let ik_error = counters.peak_ik_error();
    eprintln!("rotation: peak IK residual {ik_error:.2e}");
    assert!(
        ik_error < 1e-3,
        "the IK fell behind the pose: residual {ik_error}"
    );
}

/// 7. Cartesian, impedance backend: the same 5 cm step in two sessions, the velocity
///    feedforward on and off; the peak in-motion tracking error (the model's end effector
///    against the desired pose, from the observer) is smaller with it on.
#[test]
fn velocity_feedforward_tightens_the_tracking_of_a_moving_goal() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    let model = Arc::new(robot.load_model().expect("load_model failed"));

    // One session from the home pose: the step, the landing, the stop; the peak distance
    // between the model's end effector and the desired pose while the generator moves.
    let session = |feedforward: bool| -> f64 {
        common::move_to_start(&robot);
        let peak_lag = Arc::new(AtomicU64::new(0));
        let observer = {
            let (peak_lag, model) = (Arc::clone(&peak_lag), Arc::clone(&model));
            move |state: &RobotState, sent: &CartesianSent| {
                if norm(&sent.velocity) > 1e-3 {
                    let desired = common::translation(&sent.pose);
                    let measured = common::translation(&measured_ee(&model, state));
                    note_max(&peak_lag, distance(&measured, &desired));
                }
            }
        };
        let impedance = ImpedanceOptions::cartesian().with_velocity_feedforward(feedforward);
        let options = TargetControlOptions::default()
            .with_backend(Backend::Impedance(impedance))
            .with_observer(observer);
        let control = robot
            .start_cartesian_target_control(options)
            .expect("start_cartesian_target_control failed");
        let start = control.target();
        control
            .set_position(add(&start, &[0.05, 0.0, 0.0]))
            .expect("set_position failed");
        std::thread::sleep(Duration::from_millis(1500));
        let what = if feedforward {
            "feedforward on"
        } else {
            "feedforward off"
        };
        timed_stop(what, || control.stop());
        let lag = f64::from_bits(peak_lag.load(Ordering::Relaxed));
        eprintln!("{what}: peak in-motion tracking error {:.2} mm", lag * 1e3);
        lag
    };
    let with = session(true);
    let without = session(false);
    assert!(with > 0.0 && without > 0.0, "the arm never moved");
    assert!(
        with < without,
        "the feedforward did not tighten the tracking: {with} m with, {without} m without"
    );
}
