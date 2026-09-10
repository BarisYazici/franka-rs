//! Target control (`Robot::start_cartesian_target_control`, `start_joint_target_control`)
//! against franka-sim with motion limits enforced: a low-rate commander steps, bursts and
//! stalls its targets from the test thread while the crate's loop runs on its own. Tests 1-3
//! here stream through the robot's controller (`Backend::RobotController`); tests 4-7 in
//! [`impedance`] send the impedance backend's torques (the default).
//!
//! Run with `FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-rs --test sim_target_control
//! -- --test-threads=1`.

mod common;
#[path = "sim_target_control/impedance.rs"]
mod impedance;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use franka::robot::target_control::ENDED_MESSAGE;
use franka::{
    Backend, CartesianSent, FrankaError, JointSent, JointTargetControlOptions, RobotMode,
    RobotState, TargetControlOptions,
};
use franka_sim_test::SimConfig;

/// What the observer counts, from the realtime thread, without allocating.
#[derive(Default)]
struct Counters {
    cycles: AtomicU64,
    /// The peak of the sent command's speed and of the backstop's alteration, as `f64` bits.
    peak_speed_bits: AtomicU64,
    peak_alteration_bits: AtomicU64,
}

fn note_max(slot: &AtomicU64, value: f64) {
    if value > f64::from_bits(slot.load(Ordering::Relaxed)) {
        slot.store(value.to_bits(), Ordering::Relaxed);
    }
}

impl Counters {
    fn note(&self, speed: f64, alteration: f64) {
        self.cycles.fetch_add(1, Ordering::Relaxed);
        note_max(&self.peak_speed_bits, speed);
        note_max(&self.peak_alteration_bits, alteration);
    }
    fn peak_speed(&self) -> f64 {
        f64::from_bits(self.peak_speed_bits.load(Ordering::Relaxed))
    }
    fn peak_alteration(&self) -> f64 {
        f64::from_bits(self.peak_alteration_bits.load(Ordering::Relaxed))
    }
}

fn norm(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    norm(&[a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

/// Polls `state` every 20 ms until `landed` holds, for `hold` seconds plus the slack a loaded
/// simulator needs (it publishes below 1 kHz then, and the loop steps one cycle per state).
fn wait_landed(
    hold: f64,
    mut state: impl FnMut() -> RobotState,
    landed: impl Fn(&RobotState) -> bool,
) -> RobotState {
    let deadline = std::time::Instant::now() + Duration::from_secs_f64(hold + 3.0);
    loop {
        let current = state();
        if landed(&current) || std::time::Instant::now() > deadline {
            return current;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn add(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// 1. Cartesian: steps, a burst and a stall, every target reached, no reflex, `stop()` Ok.
#[test]
fn cartesian_target_control_follows_a_stepped_commander() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let counters = Arc::new(Counters::default());
    let observer = {
        let counters = Arc::clone(&counters);
        let mut last: Option<[f64; 3]> = None;
        move |state: &RobotState, sent: &CartesianSent| {
            let commanded = common::translation(&state.O_T_EE_c);
            let speed = last.map_or(0.0, |p| distance(&commanded, &p) / 1e-3);
            last = Some(commanded);
            counters.note(speed, sent.backstop_alteration);
        }
    };
    let options = TargetControlOptions::default()
        .with_backend(Backend::RobotController)
        .with_observer(observer);
    let budget = options.limits;
    let control = robot
        .start_cartesian_target_control(options)
        .expect("start_cartesian_target_control failed");
    assert!(control.is_running());
    let start = control.target();
    let commanded_start = common::translation(&control.state().O_T_EE_c);
    assert!(
        distance(&start, &commanded_start) < 1e-6,
        "the start target is the first cycle's echo: {start:?} vs {commanded_start:?}"
    );
    // A second control operation is refused while the loop holds the control lock.
    match robot.read(|_| false) {
        Err(FrankaError::InvalidOperation(_)) => {}
        other => panic!("expected the control lock to be held, got {other:?}"),
    }

    // Steps with holds long enough to land (0.66 s for 5 cm under the default budget).
    let script: [([f64; 3], f64); 4] = [
        ([0.05, 0.0, 0.0], 1.2),
        ([0.05, 0.05, 0.0], 1.2),
        ([0.05, 0.05, 0.05], 1.2),
        ([0.0, 0.0, 0.0], 1.5),
    ];
    for (relative, hold) in script {
        let target = add(&start, &relative);
        control.set_position(target).expect("set_position failed");
        let state = wait_landed(
            hold,
            || control.state(),
            |s| distance(&common::translation(&s.O_T_EE_c), &target) < 2e-3,
        );
        let commanded = common::translation(&state.O_T_EE_c);
        assert!(
            distance(&commanded, &target) < 2e-3,
            "the command did not land on {relative:?}: commanded {commanded:?}, target {target:?}"
        );
        let measured = common::translation(&state.O_T_EE);
        assert!(
            distance(&measured, &target) < 0.02,
            "the arm did not follow to {relative:?}: measured {measured:?}, target {target:?}"
        );
        assert!(control.is_running(), "the loop ended early");
    }
    // A burst: 20 targets 5 ms apart toggling x, then a stall of 1.5 s with nothing sent.
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

    // `stop()` settles on the last target before finishing.
    let stopped_at = std::time::Instant::now();
    control.stop().expect("stop() returned the loop's error");
    assert!(
        stopped_at.elapsed() < Duration::from_secs(4),
        "stop() took {:?}",
        stopped_at.elapsed()
    );
    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    let measured = common::translation(&state.O_T_EE);
    assert!(
        distance(&measured, &start) < 0.02,
        "the arm did not return to the start: {measured:?} vs {start:?}"
    );
    // Four 5 cm steps under the budget take at least two seconds of cycles, however slowly a
    // loaded simulator publishes them.
    let cycles = counters.cycles.load(Ordering::Relaxed);
    assert!(cycles > 2000, "only {cycles} cycles observed");
    let peak = counters.peak_speed();
    assert!(
        peak <= budget.max_velocity * 1.05,
        "the command's peak speed {peak} m/s exceeded the budget {}",
        budget.max_velocity
    );
    assert!(
        peak > 0.05,
        "the command never moved (peak speed {peak} m/s)"
    );
    // The backstop references the simulator's float32 echo of the command's velocity and
    // acceleration, so its jerk clamp fires at noise level and moves the command by
    // micrometres in most cycles (measured: up to 22 um); the generator re-anchors on the
    // echo every cycle, so nothing accumulates. Millimetres would mean it really bound.
    let alteration = counters.peak_alteration();
    assert!(
        alteration < 1e-4,
        "the backstop moved the command by up to {alteration} m; it is meant never to bind"
    );
    // The robot is free again.
    robot.read_once().expect("read_once after stop");
}

/// 2. Joint: the same commander shape on joint targets, and the handle after the loop ends.
#[test]
fn joint_target_control_follows_a_stepped_commander() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let cycles = Arc::new(AtomicU64::new(0));
    // Per-joint peak of the generator's speed, as `f64` bits.
    let peak_bits: Arc<[AtomicU64; 7]> = Arc::new(std::array::from_fn(|_| AtomicU64::new(0)));
    let observer = {
        let (cycles, peak_bits) = (Arc::clone(&cycles), Arc::clone(&peak_bits));
        move |_state: &RobotState, sent: &JointSent| {
            cycles.fetch_add(1, Ordering::Relaxed);
            for (peak, v) in peak_bits.iter().zip(&sent.velocity) {
                if v.abs() > f64::from_bits(peak.load(Ordering::Relaxed)) {
                    peak.store(v.abs().to_bits(), Ordering::Relaxed);
                }
            }
        }
    };
    let options = JointTargetControlOptions::default()
        .with_backend(Backend::RobotController)
        .with_observer(observer);
    let control = robot
        .start_joint_target_control(options)
        .expect("start_joint_target_control failed");
    let start = control.target();
    let q_d = control.state().q_d;
    assert!(
        start.iter().zip(&q_d).all(|(a, b)| (a - b).abs() < 1e-6),
        "the start target is the first cycle's echo: {start:?} vs {q_d:?}"
    );

    let mut target = start;
    let script: [(usize, f64, f64); 4] =
        [(0, 0.3, 1.5), (3, -0.3, 1.5), (6, 0.4, 1.5), (0, -0.3, 1.5)];
    for (joint, delta, hold) in script {
        target[joint] += delta;
        control.set_joints(target).expect("set_joints failed");
        let landed = |s: &RobotState| s.q_d.iter().zip(&target).all(|(a, b)| (a - b).abs() < 2e-3);
        let state = wait_landed(hold, || control.state(), landed);
        let worst = state
            .q_d
            .iter()
            .zip(&target)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(
            worst < 2e-3,
            "the command did not land: q_d {:?}, target {target:?}",
            state.q_d
        );
        let measured = state
            .q
            .iter()
            .zip(&target)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(
            measured < 0.03,
            "the arm did not follow: q {:?}, target {target:?}",
            state.q
        );
    }
    // A burst on joint 1, then a stall.
    for i in 0..20 {
        let mut toggled = target;
        toggled[0] += 0.2 * f64::from(i % 2 == 0);
        control
            .set_joints(toggled)
            .expect("burst set_joints failed");
        std::thread::sleep(Duration::from_millis(5));
    }
    std::thread::sleep(Duration::from_millis(1500));
    assert!(control.is_running());
    control.set_joints(start).expect("final set_joints failed");
    control.stop().expect("stop() returned the loop's error");

    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    let worst = state
        .q
        .iter()
        .zip(&start)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(
        worst < 0.03,
        "the arm did not return to the start: {:?}",
        state.q
    );
    assert!(cycles.load(Ordering::Relaxed) > 2000);
    let limits = JointTargetControlOptions::scaled_limits(robot.fci_version(), 0.2);
    let peaks = peak_bits
        .each_ref()
        .map(|p| f64::from_bits(p.load(Ordering::Relaxed)));
    for (joint, (peak, limit)) in peaks.iter().zip(&limits).enumerate() {
        assert!(
            *peak <= limit.max_velocity * 1.01,
            "joint {joint} peak speed {peak} rad/s against {}",
            limit.max_velocity
        );
    }
    assert!(
        peaks[0] > 0.05 && peaks[3] > 0.05 && peaks[6] > 0.05,
        "peaks {peaks:?}"
    );

    // A handle whose loop was preempted by `robot.stop()`: `set_*` refuses, `stop()` reports.
    let control = robot
        .start_joint_target_control(
            JointTargetControlOptions::default().with_backend(Backend::RobotController),
        )
        .expect("second start failed");
    robot.stop().expect("robot.stop() failed");
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while control.is_running() && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        !control.is_running(),
        "the loop must end after robot.stop()"
    );
    match control.set_joints(start) {
        Err(FrankaError::InvalidOperation(m)) => assert_eq!(m, ENDED_MESSAGE),
        other => panic!("expected InvalidOperation after the loop ended, got {other:?}"),
    }
    match control.stop() {
        Err(FrankaError::Control(e)) => assert_eq!(e.message, "libfranka: Move command preempted!"),
        other => panic!("expected the preemption, got {other:?}"),
    }
    common::move_to_start(&robot);
}

/// The angle, rad, between the rotation blocks of two column-major poses (quaternion-based:
/// accurate near zero, unlike `acos` of the trace).
fn angle_between(a: &[f64; 16], b: &[f64; 16]) -> f64 {
    let block = |p: &[f64; 16]| {
        nalgebra::Matrix4::from_column_slice(p)
            .fixed_view::<3, 3>(0, 0)
            .into_owned()
    };
    let relative = nalgebra::Rotation3::from_matrix_unchecked(block(a).transpose() * block(b));
    nalgebra::UnitQuaternion::from_rotation_matrix(&relative).angle()
}

/// A column-major pose rotating by `angle` about `axis`, no translation.
fn rotation_about(axis: [f64; 3], angle: f64) -> [f64; 16] {
    let r = nalgebra::Rotation3::from_axis_angle(
        &nalgebra::Unit::new_normalize(nalgebra::Vector3::from(axis)),
        angle,
    );
    let mut pose = [0.0; 16];
    for col in 0..3 {
        for row in 0..3 {
            pose[col * 4 + row] = r[(row, col)];
        }
    }
    pose[15] = 1.0;
    pose
}

/// 3. Cartesian pose: 20 degrees about the tool z and 10 about the base x under the
///    simulator's joint-side continuity check, landed within 1e-2 rad, no error flag raised.
#[test]
fn cartesian_target_control_reaches_a_rotated_pose() {
    let mut config = SimConfig::nominal().with_motion_limits();
    config.extra_args = vec!["--joint-discontinuity-scale".into(), "1.0".into()];
    let sim = common::sim(config);
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);

    let counters = Arc::new(Counters::default());
    let observer = {
        let counters = Arc::clone(&counters);
        let mut last: Option<[f64; 16]> = None;
        move |state: &RobotState, sent: &CartesianSent| {
            let rate = last.map_or(0.0, |p| angle_between(&p, &state.O_T_EE_c) / 1e-3);
            last = Some(state.O_T_EE_c);
            counters.note(rate, sent.backstop_angular_alteration);
        }
    };
    let options = TargetControlOptions::default()
        .with_backend(Backend::RobotController)
        .with_observer(observer);
    let budget = options.rotation_limits;
    let control = robot
        .start_cartesian_target_control(options)
        .expect("start_cartesian_target_control failed");
    let start = control.target_pose();
    let start_position = control.target();
    let start_orientation = control.target_orientation();
    assert!(
        angle_between(&start, &control.state().O_T_EE_c) < 1e-6,
        "the start target is the first cycle's echo"
    );

    // R_target = Rx(10 deg) * R_start * Rz(20 deg), at the start position.
    let tool_z =
        common::matrix_multiply(&start, &rotation_about([0.0, 0.0, 1.0], 20f64.to_radians()));
    let mut target = common::matrix_multiply(
        &rotation_about([1.0, 0.0, 0.0], 10f64.to_radians()),
        &tool_z,
    );
    target[12..15].copy_from_slice(&start_position);
    let mut skewed = target;
    skewed[0..3].iter_mut().for_each(|x| *x *= 1.05);
    match control.set_pose(&skewed) {
        Err(FrankaError::InvalidArgument(m)) => assert!(m.contains("orthonormal"), "{m}"),
        other => panic!("expected the skewed pose to be refused, got {other:?}"),
    }
    control.set_pose(&target).expect("set_pose failed");
    let state = wait_landed(
        3.0,
        || control.state(),
        |s| angle_between(&s.O_T_EE_c, &target) < 1e-3 && angle_between(&s.O_T_EE, &target) < 1e-2,
    );
    let commanded = angle_between(&state.O_T_EE_c, &target);
    assert!(
        commanded < 1e-3,
        "the command did not land: {commanded} rad off"
    );
    let measured = angle_between(&state.O_T_EE, &target);
    assert!(
        measured < 1e-2,
        "the arm did not follow: {measured} rad off"
    );
    assert!(
        distance(&common::translation(&state.O_T_EE_c), &start_position) < 1e-3,
        "the position moved during a pure rotation"
    );

    // A position step keeps the orientation; a target with a quaternion restores the start.
    control
        .set_position(add(&start_position, &[0.03, 0.0, 0.0]))
        .expect("set_position failed");
    let stepped = add(&start_position, &[0.03, 0.0, 0.0]);
    let state = wait_landed(
        1.5,
        || control.state(),
        |s| distance(&common::translation(&s.O_T_EE_c), &stepped) < 2e-3,
    );
    assert!(
        angle_between(&state.O_T_EE_c, &target) < 1e-3,
        "set_position turned the target"
    );
    assert!(
        distance(
            &common::translation(&state.O_T_EE_c),
            &add(&start_position, &[0.03, 0.0, 0.0])
        ) < 2e-3
    );
    control
        .set_target(start_position, start_orientation)
        .expect("set_target failed");
    control.stop().expect("stop() returned the loop's error");

    // The stop turned back from a standing start; the impedance-controlled arm lags the
    // landed command by a few mrad for a while.
    std::thread::sleep(Duration::from_millis(500));
    let state = common::settled_state(&robot, 5);
    assert_eq!(state.robot_mode, RobotMode::Idle);
    assert!(
        !state.current_errors.any() && !state.last_motion_errors.any(),
        "error flags raised: {} / {}",
        state.current_errors,
        state.last_motion_errors
    );
    let (measured, commanded) = (
        angle_between(&state.O_T_EE, &start),
        angle_between(&state.O_T_EE_c, &start),
    );
    assert!(
        measured < 1e-2,
        "did not turn back to the start: measured {measured} rad off, commanded {commanded}"
    );
    let peak = counters.peak_speed();
    assert!(
        peak <= budget.max_velocity * 1.05,
        "peak angular speed {peak} rad/s"
    );
    assert!(peak > 0.1, "the command never turned (peak {peak} rad/s)");
    let alteration = counters.peak_alteration();
    assert!(
        alteration < 1e-4,
        "the rotational backstop turned the command by {alteration} rad"
    );
}
