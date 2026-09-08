//! Target control: a 1 kHz loop on its own thread that follows targets a low-rate commander
//! sets whenever it likes.
//!
//! Most programs that want to move a Franka are not 1 kHz programs: a planner, a vision
//! loop, a script over a socket, a hand on a keyboard. They produce *targets* -- at 10 Hz,
//! in bursts, with pauses -- each a step the robot must never see as a step. This is the
//! fourth way to control the robot, next to the callbacks, `ActiveControl` and the
//! read-only stream: [`Robot::start_cartesian_target_control`] and
//! [`Robot::start_joint_target_control`] spawn a named thread that runs the crate's own
//! control loop and hand back a handle whose `set_*` can be called from any thread, at any
//! rate, with any target. The handle's [`stop`](CartesianTargetControl::stop) brings the
//! command to rest on the last target, finishes the motion and returns the loop's result.
//!
//! # What the loop does every cycle
//! The three rules of the [`otg`](crate::otg) module, learnt on a real arm: the generator's
//! limits are **per axis** (a Cartesian budget is a norm, so it gets
//! [`OtgLimits::per_axis_for_norm`](crate::otg::OtgLimits::per_axis_for_norm)`(3)`), it steps **one nominal cycle** per command
//! ([`DELTA_T`](crate::rate_limiting::DELTA_T)) whatever the measured period, and it is
//! **re-anchored on the robot's echo** of the last command (`O_T_EE_c`, `q_d`) with
//! `set_position` before every re-plan. Then the
//! rate limiter under the same budget runs as the backstop that must never bind
//! (`limit_rate_cartesian_pose`, `limit_rate_joint_positions`), and the loop's own libfranka
//! limiter stays on behind it; the observer is told by how much the backstop moved the
//! command. The first setpoint of the motion is always the echo itself,
//! anchored in the first cycle -- on FCI v10 the first command is its own filter reference
//! and would otherwise go out as a jump -- and the start returns only once that cycle has
//! run, so [`target`](CartesianTargetControl::target) and
//! [`state`](CartesianTargetControl::state) are valid from the first call.
//!
//! The Cartesian target is a pose: three more axes of the same synchronised generator run
//! on the base-frame rotation vector of the orientation error, `log(R_target * R_echo^T)`,
//! re-anchored at zero every cycle and composed back as `exp(step) * R_echo`, under a
//! rotational norm budget of their own; translation and rotation arrive together.
//!
//! Two guards. If the *measured* pose strays more than `max_deviation` (m) or
//! `max_angular_deviation` (rad) from the start, the target freezes where the command is,
//! the generator brings it to rest, and the loop ends with [`FrankaError::Control`]. And a
//! stop never finishes on a moving command: once every axis of the generator has landed
//! (within [`Settle::tolerance`] of the target, slower than [`REST_VELOCITY`], accelerating
//! less than [`REST_ACCELERATION`] -- the orientation included, in rad) the loop stops
//! stepping it and sends the robot's echo of the last command -- continuous with what the
//! robot has by construction, whatever the backstop took off that command -- bit for bit and
//! past the backstop, for [`Settle::cycles`] cycles, then sets `motion_finished` on one more
//! of it: the sequence a robot accepts as "finished at rest". If the generator has not landed
//! within [`STOP_TIMEOUT_CYCLES`] the same hold starts from wherever the command is.
//!
//! # Threads
//! The commander side is a single-writer seqlock ([`TargetSlot`]) the loop polls without
//! blocking; `set_*` serialises its callers with a mutex on the user side only. The latest
//! [`RobotState`] is published with `Mutex::try_lock` from the realtime side and read with
//! `lock` on the user side. Nothing allocates on the realtime thread after the start. The
//! optional observer is the exception to "nothing of yours runs at 1 kHz": it is called on
//! the realtime thread every cycle with the state and what was sent, and must not allocate
//! or block -- copying into a preallocated ring, as `franka_rerun::Recorder::push` does, is
//! what it is for.
//!
//! The loop thread is raised to `SCHED_FIFO` like `Robot::new` raises its caller: to the
//! highest priority, or to `realtime_priority` when the options name one; a failure is fatal
//! under [`RealtimeConfig::Enforce`] and ignored under [`RealtimeConfig::Ignore`].
//!
//! Only one control or read operation may run on a `Robot` at a time, so while a target
//! control runs, `robot.read()` and the other loops fail with
//! [`FrankaError::InvalidOperation`], exactly as with a callback loop on another thread;
//! `robot.stop()` preempts it, and the handle's `stop` then returns the preemption as a
//! [`FrankaError::Control`].
//!
//! ```no_run
//! use std::sync::Arc;
//! use franka::{RealtimeConfig, Robot, TargetControlOptions};
//!
//! # fn main() -> franka::FrankaResult<()> {
//! let robot = Arc::new(Robot::new("192.168.0.1", RealtimeConfig::Enforce)?);
//! let control = robot.start_cartesian_target_control(TargetControlOptions::default())?;
//! let start = control.target();
//! for step in 1..=5 {
//!     // A planner, a socket, a keyboard: any thread, any rate, any target.
//!     control.set_position([start[0] + 0.01 * f64::from(step), start[1], start[2]])?;
//!     std::thread::sleep(std::time::Duration::from_millis(300));
//! }
//! // An orientation too: a unit quaternion in [x, y, z, w] order, or a pose as O_T_EE.
//! let yaw = std::f64::consts::FRAC_PI_8;
//! let orientation = [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()];
//! control.set_target(start, orientation)?;
//! control.stop()?; // settles on the last target, finishes the motion, joins the thread
//! # Ok(())
//! # }
//! ```

mod cartesian;
mod joint;
mod rotation;
mod runner;
mod slot;

pub use cartesian::{
    CartesianObserver, CartesianSent, CartesianTargetControl, TargetControlOptions,
};
pub use joint::{
    JointObserver, JointSent, JointTargetControl, JointTargetControlOptions, DEFAULT_LIMIT_FRACTION,
};
pub use rotation::{ORTHONORMAL_TOLERANCE, UNIT_QUATERNION_TOLERANCE};
pub use slot::TargetSlot;

use runner::Runner;
pub use runner::{REST_ACCELERATION, REST_VELOCITY};

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, SyncSender};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

use crate::error::{ControlException, FrankaError, FrankaResult};
use crate::realtime::{
    set_current_thread_scheduler_priority, set_current_thread_to_highest_scheduler_priority,
    RealtimeConfig,
};
use crate::robot::Robot;
use crate::robot_state::RobotState;

/// Cycles a stop waits for the generator to land before holding and finishing from wherever
/// the command is: five seconds.
pub const STOP_TIMEOUT_CYCLES: u32 = 5000;

/// The message of the [`FrankaError::InvalidOperation`] a `set_*` returns once the loop has
/// ended, for whatever reason; `stop()` has the reason.
pub const ENDED_MESSAGE: &str =
    "franka target control: the control loop has ended; stop() returns its result.";

/// The message of the [`FrankaError::Control`] the loop ends with after the deviation guard
/// froze the target.
pub const DEVIATION_MESSAGE: &str = "franka target control: the measured pose strayed \
     further than max_deviation (or max_angular_deviation) from the start; the target was \
     frozen where the command was and the motion finished from rest.";

/// How a stop ends: once the generator is within `tolerance` of the target on every axis and
/// at rest ([`REST_VELOCITY`], [`REST_ACCELERATION`]), the echo of the last command is held
/// for `cycles` cycles and `motion_finished` set on the next one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settle {
    /// Per-axis tolerance, in the interface's unit: m for a position, rad for a joint or for
    /// the orientation error of a pose. Default 1e-3.
    pub tolerance: f64,
    /// Cycles the landed command is held, identical, before `motion_finished`. Default 250.
    pub cycles: u32,
}

impl Default for Settle {
    fn default() -> Self {
        Settle {
            tolerance: 1e-3,
            cycles: 250,
        }
    }
}

/// Checks the options both interfaces share.
fn validate_common(
    max_deviation: f64,
    settle: Settle,
    realtime_priority: Option<i32>,
) -> FrankaResult<()> {
    let positive = |x: f64| x.is_finite() && x > 0.0;
    if !positive(max_deviation) {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: max_deviation must be finite and positive, got {max_deviation}"
        )));
    }
    if !positive(settle.tolerance) || settle.cycles == 0 {
        return Err(FrankaError::InvalidArgument(format!(
            "target control: settle needs a finite, positive tolerance and at least one \
             cycle, got {settle:?}"
        )));
    }
    if let Some(priority) = realtime_priority {
        if !(1..=99).contains(&priority) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: realtime_priority must be within 1..=99, got {priority}"
            )));
        }
    }
    Ok(())
}

/// What the user thread and the loop thread share.
struct Shared<const N: usize> {
    slot: TargetSlot<N>,
    stop: AtomicBool,
    running: AtomicBool,
    state: Mutex<RobotState>,
    /// Serialises the writers of `slot`.
    writer: Mutex<()>,
}

impl<const N: usize> Default for Shared<N> {
    fn default() -> Self {
        Shared {
            slot: TargetSlot::default(),
            stop: AtomicBool::new(false),
            running: AtomicBool::new(false),
            state: Mutex::new(RobotState::default()),
            writer: Mutex::new(()),
        }
    }
}

/// The user side, generic over the interface; the public handles wrap it.
struct Handle<const N: usize> {
    shared: Arc<Shared<N>>,
    thread: Option<JoinHandle<FrankaResult<()>>>,
}

impl<const N: usize> Handle<N> {
    fn set_target(&self, target: [f64; N]) -> FrankaResult<()> {
        self.modify_target(|current| *current = target)
    }

    /// Publishes `modify` of the latest target, under the writer lock so that two callers
    /// changing different parts of it never lose each other's part.
    fn modify_target(&self, modify: impl FnOnce(&mut [f64; N])) -> FrankaResult<()> {
        let _writer = self
            .shared
            .writer
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if !self.is_running() {
            return Err(FrankaError::InvalidOperation(ENDED_MESSAGE.to_string()));
        }
        let mut target = self.target();
        modify(&mut target);
        if target.iter().any(|v| !v.is_finite()) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: the target must be finite, got {target:?}"
            )));
        }
        self.shared.slot.publish(target);
        Ok(())
    }

    fn target(&self) -> [f64; N] {
        let mut target = [0.0; N];
        // Only a writer mid-update can make a load fail, and a write is a handful of stores.
        for _ in 0..1000 {
            if self.shared.slot.load(&mut target) {
                break;
            }
            std::hint::spin_loop();
        }
        target
    }

    fn state(&self) -> RobotState {
        *self
            .shared
            .state
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
    }

    fn is_running(&self) -> bool {
        self.shared.running.load(Ordering::SeqCst)
    }

    fn stop(mut self) -> FrankaResult<()> {
        self.shared.stop.store(true, Ordering::SeqCst);
        match self.thread.take() {
            Some(thread) => join(thread),
            None => Ok(()),
        }
    }
}

impl<const N: usize> Drop for Handle<N> {
    /// Requests the stop and detaches: the loop settles and finishes on its own, holding its
    /// `Arc<Robot>` until it has.
    fn drop(&mut self) {
        if self.thread.is_some() {
            self.shared.stop.store(true, Ordering::SeqCst);
        }
    }
}

fn join(thread: JoinHandle<FrankaResult<()>>) -> FrankaResult<()> {
    thread.join().unwrap_or_else(|_| {
        Err(FrankaError::Control(ControlException::new(
            "franka target control: the control thread panicked",
        )))
    })
}

/// `SCHED_FIFO` for the loop thread, the way `Robot::new` does it for its caller.
fn raise_priority(config: RealtimeConfig, priority: Option<i32>) -> FrankaResult<()> {
    let result = match priority {
        None => set_current_thread_to_highest_scheduler_priority(),
        Some(priority) => set_current_thread_scheduler_priority(priority),
    };
    match result {
        Err(message) if config == RealtimeConfig::Enforce => Err(FrankaError::Realtime(message)),
        _ => Ok(()),
    }
}

/// Spawns the loop thread and waits for its first cycle. `body` runs the crate's control
/// loop and must send on the channel from the first cycle; if it returns before that, its
/// error is what the start returns.
fn spawn<const N: usize, F>(
    name: &str,
    robot: &Arc<Robot>,
    shared: Arc<Shared<N>>,
    priority: Option<i32>,
    body: F,
) -> FrankaResult<Handle<N>>
where
    F: FnOnce(&Robot, SyncSender<()>) -> FrankaResult<()> + Send + 'static,
{
    let (started, first_cycle) = mpsc::sync_channel::<()>(1);
    let robot = Arc::clone(robot);
    let config = robot.robot.realtime_config();
    shared.running.store(true, Ordering::SeqCst);
    let thread_shared = Arc::clone(&shared);
    let thread = std::thread::Builder::new()
        .name(name.to_string())
        .spawn(move || {
            let result = raise_priority(config, priority).and_then(|()| body(&robot, started));
            thread_shared.running.store(false, Ordering::SeqCst);
            result
        })
        .map_err(|e| {
            FrankaError::InvalidOperation(format!(
                "franka target control: cannot spawn the control thread: {e}"
            ))
        })?;
    match first_cycle.recv() {
        Ok(()) => Ok(Handle {
            shared,
            thread: Some(thread),
        }),
        Err(_) => Err(join(thread).err().unwrap_or_else(|| {
            FrankaError::Control(ControlException::new(
                "franka target control: the control loop ended before its first cycle",
            ))
        })),
    }
}

impl Robot {
    /// Starts a Cartesian target control loop on its own thread and returns once its first
    /// cycle has anchored on the current commanded pose; see the
    /// [module documentation](self).
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] if the options are invalid,
    /// [`FrankaError::Realtime`] if the loop thread cannot be raised to `SCHED_FIFO` under
    /// [`RealtimeConfig::Enforce`], and whatever [`Robot::control_cartesian_pose`] fails with
    /// before its first cycle, [`FrankaError::InvalidOperation`] if another control or read
    /// operation is running among them.
    pub fn start_cartesian_target_control(
        self: &Arc<Self>,
        options: TargetControlOptions,
    ) -> FrankaResult<CartesianTargetControl> {
        cartesian::start(self, options)
    }

    /// Starts a joint target control loop on its own thread and returns once its first cycle
    /// has anchored on the current commanded joint positions; see the
    /// [module documentation](self).
    ///
    /// # Errors
    /// As [`Robot::start_cartesian_target_control`], with
    /// [`Robot::control_joint_positions`] as the loop.
    pub fn start_joint_target_control(
        self: &Arc<Self>,
        options: JointTargetControlOptions,
    ) -> FrankaResult<JointTargetControl> {
        joint::start(self, options)
    }
}

#[cfg(test)]
mod tests;
