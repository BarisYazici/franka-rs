//! The impedance backend's joint position guard against franka-sim (>= 1.1.6): a joint target
//! at the margin above joint 4's lower limit is reached and held, the arm never inside the
//! position barrier's onset on the way, and a target inside the margin is refused.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use franka::robot::target_control::{joint_position_limits, POSITION_FADE_BAND};
use franka::{FrankaError, ImpedanceOptions, JointSent, JointTargetControlOptions, RobotState};
use franka_sim_test::SimConfig;

use crate::{common, wait_landed};

/// How close to the target joint 4 must settle, rad.
const SETTLED: f64 = 1e-3;

/// 9. Joint impedance backend from the simulator's home: joint 4 sent to its lower limit plus
///    the joint position margin lands there within [`SETTLED`] and holds without drifting past
///    it; joint 4 never comes closer to the limit than the barrier's onset; a target 0.04 rad
///    above the limit is refused and the loop runs on; `stop()` Ok.
#[test]
fn a_joint_target_at_the_margin_settles_and_one_inside_is_refused() {
    let sim = common::sim(SimConfig::nominal().with_motion_limits());
    let robot = Arc::new(common::connect(&sim));
    common::set_default_behavior(&robot);
    common::move_to_start(&robot);
    let lower = joint_position_limits(robot.fci_version()).0[3];
    let margin = ImpedanceOptions::joint().joint_position_margin;

    // The lowest joint 4 measured, as `f64` bits (below every start, so the first value wins).
    let lowest = Arc::new(AtomicU64::new(f64::INFINITY.to_bits()));
    let observer = {
        let lowest = Arc::clone(&lowest);
        move |state: &RobotState, _: &JointSent| {
            if state.q[3] < f64::from_bits(lowest.load(Ordering::Relaxed)) {
                lowest.store(state.q[3].to_bits(), Ordering::Relaxed);
            }
        }
    };
    // Joint 4 travels about 1.4 rad, past the default 1 rad deviation guard.
    let options = JointTargetControlOptions::default()
        .with_max_deviation(2.0)
        .with_observer(observer);
    let control = robot
        .start_joint_target_control(options)
        .expect("start_joint_target_control failed");
    let home = control.state().q;
    let mut target = home;
    target[3] = lower + margin;
    control
        .set_joints(target)
        .expect("a target at the margin was refused");
    let settled = |s: &RobotState| (s.q[3] - target[3]).abs() < SETTLED;
    let landed = wait_landed(8.0, || control.state(), settled);
    assert!(
        settled(&landed),
        "joint 4 did not settle at the margin: {} rad from the target",
        landed.q[3] - target[3]
    );
    std::thread::sleep(std::time::Duration::from_secs(2));
    let held = control.state().q[3];
    eprintln!(
        "position guard: joint 4 {:.2} mrad from the target after 2 s, lowest {:.2} mrad above \
         the limit",
        (held - target[3]) * 1e3,
        (f64::from_bits(lowest.load(Ordering::Relaxed)) - lower) * 1e3
    );
    assert!(
        (held - target[3]).abs() < SETTLED,
        "joint 4 drifted to {held}"
    );
    let closest = f64::from_bits(lowest.load(Ordering::Relaxed)) - lower;
    assert!(
        closest >= margin - POSITION_FADE_BAND,
        "joint 4 came {closest} rad from its limit, inside the barrier's onset"
    );

    let mut inside = target;
    inside[3] = lower + 0.04;
    let refused = control.set_joints(inside);
    assert!(
        matches!(refused, Err(FrankaError::InvalidArgument(ref m)) if m.contains("joint 4")),
        "a target inside the margin was not refused: {refused:?}"
    );
    assert!(control.is_running(), "the refusal ended the loop");

    control.set_joints(home).expect("the way home was refused");
    wait_landed(8.0, || control.state(), |s| (s.q[3] - home[3]).abs() < 0.01);
    control.stop().expect("stop() returned the loop's error");
}
