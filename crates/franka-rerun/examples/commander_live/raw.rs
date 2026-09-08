//! The `--raw` contrast of `commander_live.rs`: the commander on its own thread publishing
//! into a slot, and a plain `control_cartesian_pose` callback on the calling thread sending
//! `start pose + target` as is -- no generator, no limiting -- so the first step is refused.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use franka::robot::target_control::TargetSlot;
use franka::{
    CartesianPose, Duration, FrankaResult, Robot, RobotCommandLog, RobotState, MAX_CUTOFF_FREQUENCY,
};
use franka_rerun::{distance, Recorder};

use super::args::Args;
use super::script::{self, BOX};
use super::sink::Clock;
use super::{commander, translation, MAX_DEVIATION};

/// Returns the loop's result; a reflex is the expected outcome.
pub fn run(
    args: &Args,
    robot: &Robot,
    recorder: &Arc<Recorder>,
    clock: &Arc<Clock>,
) -> FrankaResult<()> {
    let start = translation(&robot.read_once()?.O_T_EE_c);
    println!("Start position: {start:.3?}; targets stay within +-{BOX} m of it.");
    let mut sink = commander(recorder, clock, start);
    let slot = Arc::new(TargetSlot::<3>::new([0.0; 3]));
    let finished = Arc::new(AtomicBool::new(false));
    let (commander_slot, commander_finished, from_stdin) =
        (Arc::clone(&slot), Arc::clone(&finished), args.from_stdin);
    std::thread::spawn(move || {
        franka_rerun::recorder::drop_realtime_priority();
        let mut publish = |target: [f64; 3]| {
            commander_slot.publish(target);
            true
        };
        match from_stdin {
            true => script::run_stdin(&mut publish, &mut sink),
            false => script::run_script(&mut publish, &mut sink),
        }
        commander_finished.store(true, Ordering::SeqCst);
    });
    // Only the control thread runs realtime; the recorder, the Rerun SDK's own threads and
    // the commander were spawned above and stay on the normal scheduler.
    if !franka_rerun::recorder::raise_realtime_priority(80) {
        eprintln!("warning: could not raise the control thread to SCHED_FIFO 80");
    }
    let (mut start_pose, mut target, mut settled, mut deviated) = (None, [0.0f64; 3], 0u32, false);
    robot.control_cartesian_pose(
        |state: &RobotState, _period: Duration| {
            clock
                .time_ms
                .store(state.time.as_millis(), Ordering::Relaxed);
            // Anchored on the first echo: on a real robot `O_T_EE_c` drifts by micrometres
            // between the `read_once` above and this cycle, and raw mode would send that
            // difference as a jump.
            let start_pose = *start_pose.get_or_insert(state.O_T_EE_c);
            let (start, commanded) = (translation(&start_pose), translation(&state.O_T_EE_c));
            if !deviated && distance(&translation(&state.O_T_EE), &start) > MAX_DEVIATION {
                deviated = true;
                target = std::array::from_fn(|i| commanded[i] - start[i]);
            } else if !deviated {
                slot.load(&mut target);
            }
            let goal: [f64; 3] = std::array::from_fn(|i| start[i] + target[i]);
            let mut pose = start_pose;
            pose[12..15].copy_from_slice(&goal);
            let command = RobotCommandLog {
                O_T_EE_c: pose,
                ..RobotCommandLog::default()
            };
            recorder.push(state, Some(command));
            let done = deviated || finished.load(Ordering::SeqCst);
            settled = if done && distance(&commanded, &goal) < 1e-3 {
                settled + 1
            } else {
                0
            };
            let mut output = CartesianPose::new(pose);
            output.motion_finished = settled >= 250;
            output
        },
        args.controller,
        false,
        MAX_CUTOFF_FREQUENCY,
    )
}
