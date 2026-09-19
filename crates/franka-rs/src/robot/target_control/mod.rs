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
//! # Two backends
//! The [`Backend`] of the options decides what the generator's output becomes. The default,
//! [`Backend::Impedance`], sends **torques**: the hybrid joint and Cartesian impedance law of
//! [`ImpedanceGains`] (`Kp = J^T Kx J + Kq`, `Kd = J^T Kxd J + Kqd`, the damping on the
//! measured velocity, or with [`ImpedanceOptions::velocity_feedforward`] on the velocity error
//! `dq_goal - dq`, plus the Coriolis term, clamped to the torque limits) tracks a joint target
//! `q_goal` -- the generator's own output on the joint interface, the solution of a
//! differential inverse kinematics following the generator's pose one cycle at a time on the
//! Cartesian one ([`IkOptions`]) -- through [`Robot::control_torques`] with the crate's
//! low-pass filter ([`ImpedanceOptions::cutoff_frequency`]) and torque rate limiter. The goal
//! never moves a joint faster than [`ImpedanceOptions::joint_velocity_fraction`] of the arm's limit
//! ([`max_joint_velocity`], which narrows toward the position limits on the FR3): a faster step is
//! scaled down as a whole and the generator re-anchored on what went out. A joint *measured* faster
//! than the cap, or than [`FADE_BAND`] of its limit under the barrier's onset if that is lower,
//! keeps less and less of the law's torque along its motion, none from
//! [`ImpedanceOptions::velocity_barrier_fraction`] of its limit, where it meets a damping of
//! [`VELOCITY_BARRIER_GAIN`] on the excess, added before the clamp. Toward a joint
//! position limit the goal brakes to stop [`ImpedanceOptions::joint_position_margin`] inside it; a
//! joint measured inside the margin keeps less of the law's torque toward the limit over
//! [`POSITION_FADE_BAND`], then meets a spring ([`POSITION_BARRIER_STIFFNESS`]). There is no echo of
//! a torque command, so the loop anchors on the *measured* configuration in its first
//! cycle (the Cartesian interface on the model's pose of it, where the IK's residual is zero), and
//! then, every cycle, on the measured state pulled toward the previous desired by at most the
//! [`Leash`]: exactly the previous desired while the arm follows, so the generator runs from its
//! own last output and its limits are the whole budget; a bounded distance ahead of an arm that is
//! held back, so the spring force is bounded by the stiffness times the leash and the generator
//! resumes from where the arm is once it is let go.
//! [`Backend::RobotController`] instead streams the generator's output as a pose or joint-position
//! command to the robot's own impedance controller (`controller_mode`); the rest of this page
//! describes that path where the two differ.
//!
//! # What the loop does every cycle
//! The three rules of the [`otg`](crate::otg) module: the generator's
//! limits are **per axis** (a Cartesian budget is a norm, so it gets
//! [`OtgLimits::per_axis_for_norm`](crate::otg::OtgLimits::per_axis_for_norm)`(3)`), it steps **one nominal cycle** per command
//! ([`DELTA_T`](crate::rate_limiting::DELTA_T)) whatever the measured period, and it is
//! **re-anchored on the last command** -- the robot's echo of it (`O_T_EE_c`, `q_d`) with
//! [`Backend::RobotController`], the leashed previous output with [`Backend::Impedance`] --
//! with `set_position` before every re-plan. With [`Backend::RobotController`] the
//! rate limiter under the same budget then runs as the backstop that must never bind
//! (`limit_rate_cartesian_pose`, `limit_rate_joint_positions`), and the loop's own libfranka
//! limiter stays on behind it; the observer is told by how much the backstop moved the
//! command. The first setpoint of the motion is always the anchor itself
//! -- on FCI v10 the first command is its own filter reference
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
//! stepping it and holds the last command -- with [`Backend::RobotController`] the robot's
//! echo of it, continuous with what the robot has by construction, whatever the backstop took
//! off that command, sent bit for bit and past the backstop; with [`Backend::Impedance`] the
//! landed desired pose or joint goal, whose torques are those of rest -- for
//! [`Settle::cycles`] cycles, then
//! sets `motion_finished` on one more of it: the sequence a robot accepts as "finished at
//! rest". If the generator has not landed within [`STOP_TIMEOUT_CYCLES`] the same hold starts
//! from wherever the command is. That hold settles the generator, not the arm: in torque mode
//! an arm still closing its lag would be handed to the robot's controller short of the goal,
//! so [`Backend::Impedance`] finishes only once every joint moves slower than
//! [`REST_JOINT_VELOCITY`], or after [`STOP_TIMEOUT_CYCLES`] more cycles, the law kept on the
//! held goal meanwhile.
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
//! highest priority, or to `realtime_priority` when the options name one; when they name a
//! `cpu`, it also pins itself to that core, whether or not the raise succeeded. A failure of
//! either is fatal under [`RealtimeConfig::Enforce`] and ignored under
//! [`RealtimeConfig::Ignore`].
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
//! let robot = Arc::new(Robot::new("172.16.0.2", RealtimeConfig::Enforce)?);
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
mod envelope;
mod ik;
mod impedance;
mod joint;
mod options;
mod position;
mod rotation;
mod runner;
mod slot;
mod torque;
mod tuning;
mod velocity;

pub use cartesian::{CartesianObserver, CartesianSent, CartesianTargetControl};
pub use ik::{IkOptions, MAX_POSTURE_RATE};
pub use impedance::{
    impedance_torques, Backend, ImpedanceGains, ImpedanceOptions, Leash, MIN_FEEDFORWARD_CUTOFF,
    RATED_TORQUES,
};
pub use joint::{JointObserver, JointSent, JointTargetControl};
pub use options::{JointTargetControlOptions, TargetControlOptions, DEFAULT_LIMIT_FRACTION};
pub use position::{POSITION_BARRIER_STIFFNESS, POSITION_FADE_BAND};
pub use rotation::{ORTHONORMAL_TOLERANCE, UNIT_QUATERNION_TOLERANCE};
pub use slot::TargetSlot;
pub use tuning::{
    FieldBound, LiveTuning, SlewGains, TuningDanger, TuningPolicy, TuningUpdate,
    MIN_JOINT_DAMPING_RATIO, SLEW_TAU,
};
pub use velocity::{FADE_BAND, VELOCITY_BARRIER_GAIN};

use runner::Runner;
pub use runner::{REST_ACCELERATION, REST_JOINT_VELOCITY, REST_VELOCITY};

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, SyncSender};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

use crate::error::{ControlException, FrankaError, FrankaResult};
use crate::rate_limiting;
use crate::realtime::{
    pin_current_thread_to_cpu, set_current_thread_scheduler_priority,
    set_current_thread_to_highest_scheduler_priority, RealtimeConfig,
};
use crate::robot::Robot;
use crate::robot_state::RobotState;
use crate::wire::robot::codec::FciVersion;

/// Cycles a stop waits for the generator to land before holding and finishing from wherever
/// the command is: five seconds.
pub const STOP_TIMEOUT_CYCLES: u32 = 5000;

/// The message of the [`FrankaError::InvalidOperation`] a `set_*` returns once the loop has
/// ended, for whatever reason; `stop()` has the reason.
pub const ENDED_MESSAGE: &str =
    "franka target control: the control loop has ended; stop() returns its result.";

/// The message of the [`FrankaError::InvalidOperation`] a live-tuning call returns on a session
/// whose interface or backend has none: [`LiveTuning`] describes the impedance law of the
/// Cartesian interface, and nothing else runs one.
pub const NO_TUNING_MESSAGE: &str = "franka target control: this session has no live tuning; \
     it is the Cartesian interface's on the impedance backend.";

/// The message of the [`FrankaError::InvalidOperation`] a live-tuning call returns on a Cartesian
/// impedance session whose gains [`LiveTuning::gains`] cannot rebuild from one Cartesian
/// stiffness -- [`ImpedanceGains::DROID`] is the preset this is true of. Such a session keeps the
/// gains it was started with and is given no tuning slot, rather than having the six derived
/// dampings rewritten in one cycle by the first change to any other field.
pub const DERIVED_GAINS_MESSAGE: &str = "franka target control: this session has no live tuning; \
     its Cartesian gains are not a scaled ImpedanceGains::CARTESIAN, which is all the one \
     tunable stiffness can rebuild.";

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

/// How the loop thread is scheduled: its `SCHED_FIFO` priority (`None`: the highest) and the
/// CPU it is pinned to, if any.
#[derive(Debug, Clone, Copy)]
struct Scheduling {
    priority: Option<i32>,
    cpu: Option<usize>,
}

/// Checks the options both interfaces share.
fn validate_common(
    max_deviation: f64,
    settle: Settle,
    scheduling: Scheduling,
    backend: &Backend,
) -> FrankaResult<()> {
    if let Backend::Impedance(impedance) = backend {
        impedance.validate()?;
    }
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
    if let Some(priority) = scheduling.priority {
        if !(1..=99).contains(&priority) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: realtime_priority must be within 1..=99, got {priority}"
            )));
        }
    }
    if let Some(cpu) = scheduling.cpu {
        if cpu >= libc::CPU_SETSIZE as usize {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: cpu must be below {}, got {cpu}",
                libc::CPU_SETSIZE
            )));
        }
    }
    Ok(())
}

/// The joint position limits (lower, upper), rad, of the arm speaking `version`: the FER's or
/// the FR3's `JOINT_POSITION_LIMITS`.
pub fn joint_position_limits(version: FciVersion) -> ([f64; 7], [f64; 7]) {
    match version {
        FciVersion::V5 => rate_limiting::fer::JOINT_POSITION_LIMITS,
        FciVersion::V10 => rate_limiting::JOINT_POSITION_LIMITS,
    }
}

/// The FR3's joint velocity caps, rad/s: the specifications page's `dq_max`, where its
/// position-dependent envelope saturates away from the joint limits; the `<limit velocity>` of
/// its URDF carries the same values. Nominal, where the V5 branch of [`max_joint_velocity`]
/// returns the FER's `MAX_JOINT_VELOCITY` already less libfranka's `LIMIT_EPS` and lost-packet
/// tolerance: each branch hands back the constant its arm's module defines, so the two differ by
/// that 1e-3 rad/s.
const FR3_MAX_JOINT_VELOCITY: [f64; 7] = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];

/// The joint velocity limits, rad/s, of the arm speaking `version` away from its joint position
/// limits: the FER's `MAX_JOINT_VELOCITY` or the FR3's caps. The FR3's limit narrows toward the
/// position limits, which the impedance backend follows; the FER's is flat. Its cap and barrier
/// ([`ImpedanceOptions::joint_velocity_fraction`], [`ImpedanceOptions::velocity_barrier_fraction`])
/// and [`JointTargetControlOptions::scaled_limits`] are fractions of these.
pub fn max_joint_velocity(version: FciVersion) -> [f64; 7] {
    match version {
        FciVersion::V5 => rate_limiting::fer::MAX_JOINT_VELOCITY,
        FciVersion::V10 => FR3_MAX_JOINT_VELOCITY,
    }
}

/// How far, rad, inside the negotiated version's joint position limits a joint target and an
/// impedance posture must lie: [`JointTargetControl::set_joints`] and both `start`s refuse a
/// configuration outside. [`Backend::Impedance`] refuses one inside
/// [`ImpedanceOptions::joint_position_margin`] where that is larger.
pub const JOINT_LIMIT_INSET: f64 = 0.02;

/// The inset, rad, targets and postures of `backend` are refused inside.
fn joint_limit_inset(backend: &Backend) -> f64 {
    match backend {
        Backend::Impedance(impedance) => JOINT_LIMIT_INSET.max(impedance.joint_position_margin),
        Backend::RobotController => JOINT_LIMIT_INSET,
    }
}

/// Refuses a `q` outside `limits` inset by `inset`, naming the joint and `what`.
fn check_joint_limits(
    q: &[f64; 7],
    limits: &([f64; 7], [f64; 7]),
    inset: f64,
    what: &str,
) -> FrankaResult<()> {
    for (i, value) in q.iter().enumerate() {
        let lower = limits.0[i] + inset;
        let upper = limits.1[i] - inset;
        if !(lower..=upper).contains(value) {
            return Err(FrankaError::InvalidArgument(format!(
                "target control: the {what} puts joint {} at {value} rad, outside \
                 [{lower}, {upper}] ({inset} rad inside the arm's limits)",
                i + 1
            )));
        }
    }
    Ok(())
}

/// The posture of an impedance backend, checked against the arm's limits at `start`.
fn check_posture(backend: &Backend, limits: &([f64; 7], [f64; 7])) -> FrankaResult<()> {
    match backend {
        Backend::Impedance(ImpedanceOptions {
            posture: Some(posture),
            ..
        }) => check_joint_limits(posture, limits, joint_limit_inset(backend), "posture"),
        _ => Ok(()),
    }
}

/// What the user thread and the loop thread share.
struct Shared<const N: usize> {
    slot: TargetSlot<N>,
    /// The live tuning, as [`LiveTuning::to_words`]: the same seqlock as `slot` and the same
    /// single writer, but the loop only ever reads it, and what it carries are targets the loop
    /// crosses to over [`SLEW_TAU`] rather than a command for the next cycle. A session without
    /// live tuning leaves it unwritten, which is what [`NO_TUNING_MESSAGE`] reports.
    tuning: TargetSlot<{ LiveTuning::WORDS }>,
    stop: AtomicBool,
    running: AtomicBool,
    state: Mutex<RobotState>,
    /// Serialises the writers of `slot` and of `tuning`.
    writer: Mutex<()>,
}

impl<const N: usize> Default for Shared<N> {
    fn default() -> Self {
        Shared {
            slot: TargetSlot::default(),
            tuning: TargetSlot::default(),
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

    /// Applies `update` to the published tuning and returns the bounds that clamped a value.
    ///
    /// The read, the gate and the publish are one critical section under the same `writer` lock
    /// `modify_target` takes, so two callers moving different sliders cannot lose each other's,
    /// and [`LiveTuning::apply_update`] is the only way into the slot: there is no setter that
    /// takes values past [`LiveTuning::BOUNDS`].
    fn tune(&self, update: &TuningUpdate) -> FrankaResult<Vec<&'static FieldBound>> {
        let _writer = self
            .shared
            .writer
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if !self.is_running() {
            return Err(FrankaError::InvalidOperation(ENDED_MESSAGE.to_string()));
        }
        let mut tuning = self.published_tuning()?;
        let clamped = tuning.apply_update(update)?;
        self.shared.tuning.publish(tuning.to_words());
        Ok(clamped)
    }

    /// The published tuning targets; the loop is somewhere between these and what it started
    /// with (see [`TuningPolicy::remaining`]).
    fn tuning(&self) -> FrankaResult<LiveTuning> {
        let _writer = self
            .shared
            .writer
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        self.published_tuning()
    }

    /// The tuning slot, read by a caller already holding `writer`: the slot's one writer is
    /// therefore excluded, so a load cannot tear and a failure means the slot was never seeded.
    fn published_tuning(&self) -> FrankaResult<LiveTuning> {
        let mut words = [0.0; LiveTuning::WORDS];
        if !self.shared.tuning.load(&mut words) {
            return Err(FrankaError::InvalidOperation(NO_TUNING_MESSAGE.to_string()));
        }
        Ok(LiveTuning::from_words(&words))
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

/// `SCHED_FIFO` for the loop thread, the way `Robot::new` does it for its caller, and the
/// pin to `scheduling.cpu` if there is one. Both are attempted (the pin needs no privilege
/// the raise may lack); the first failure is the one reported.
fn schedule(config: RealtimeConfig, scheduling: Scheduling) -> FrankaResult<()> {
    let priority = match scheduling.priority {
        None => set_current_thread_to_highest_scheduler_priority(),
        Some(priority) => set_current_thread_scheduler_priority(priority),
    };
    let pin = scheduling.cpu.map_or(Ok(()), pin_current_thread_to_cpu);
    match priority.and(pin) {
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
    scheduling: Scheduling,
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
            let result = schedule(config, scheduling).and_then(|()| body(&robot, started));
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
    /// cycle has anchored on the current pose (the model's pose of the measured configuration
    /// with [`Backend::Impedance`], the commanded one with [`Backend::RobotController`]); see
    /// the [module documentation](self).
    ///
    /// # Errors
    /// [`FrankaError::InvalidArgument`] if the options are invalid,
    /// [`FrankaError::Realtime`] if the loop thread cannot be raised to `SCHED_FIFO` or
    /// pinned to `cpu` under [`RealtimeConfig::Enforce`], whatever [`Robot::load_model`]
    /// fails with under [`Backend::Impedance`], and whatever [`Robot::control_torques`] or
    /// [`Robot::control_cartesian_pose`] fails with before its first cycle,
    /// [`FrankaError::InvalidOperation`] if another control or read operation is running
    /// among them.
    pub fn start_cartesian_target_control(
        self: &Arc<Self>,
        options: TargetControlOptions,
    ) -> FrankaResult<CartesianTargetControl> {
        cartesian::start(self, options)
    }

    /// Starts a joint target control loop on its own thread and returns once its first cycle
    /// has anchored on the current joint positions; see the [module documentation](self).
    ///
    /// # Errors
    /// As [`Robot::start_cartesian_target_control`], with
    /// [`Robot::control_joint_positions`] as the [`Backend::RobotController`] loop.
    pub fn start_joint_target_control(
        self: &Arc<Self>,
        options: JointTargetControlOptions,
    ) -> FrankaResult<JointTargetControl> {
        joint::start(self, options)
    }
}

#[cfg(test)]
mod tests;
