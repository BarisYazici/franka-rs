//! Shared helpers for the ported libfranka examples.
//!
//! Port of `examples/examples_common.h` / `examples/examples_common.cpp` (libfranka 0.21.2):
//! [`set_default_behavior`] and the point-to-point [`MotionGenerator`] that every
//! `generate_*_motion` example and `communication_test` use to first move the arm to a
//! suitable joint configuration. Each example pulls this file in with
//! `#[path = "common/mod.rs"] mod common;`, since cargo does not auto-share code between
//! `examples/*.rs` binaries.
//!
//! The simulator integration tests pull the very same file in as
//! `#[path = "../../examples/common/mod.rs"] pub mod examples_common;` from
//! `tests/common/mod.rs`, so that `common::move_to_start` homes the arm with exactly the
//! [`MotionGenerator`] the examples use rather than a second copy of it.
//!
//! Also home to [`realtime_config_from_env`], which is not part of libfranka's C++ examples
//! (they hardcode `RealtimeConfig::kEnforce` through the `franka::Robot` default constructor
//! argument). Every ported example here picks
//! [`franka::RealtimeConfig`] from the `FRANKA_REALTIME` environment variable instead, so the
//! same binary can run against real hardware (`enforce`, the default, matching libfranka) or
//! against franka-sim, which does not run under `PREEMPT_RT` (`ignore`).

#![allow(dead_code)] // Not every example uses every helper here.

/// Cartesian-impedance helpers, shared by the two `cartesian_impedance_*` examples.
pub mod cartesian;
/// The scripted commander of `nonrealtime_commander.rs`.
pub mod commander;

use franka::{
    Duration, FciVersion, FrankaResult, JointPositions, RealtimeConfig, Robot, RobotState,
};

/// libfranka's "ready" joint configuration, used by every `generate_*_motion.cpp` example and
/// `communication_test.cpp` as the target of the initial [`MotionGenerator`] move.
pub const READY_POSE: [f64; 7] = [
    0.0,
    -std::f64::consts::FRAC_PI_4,
    0.0,
    -3.0 * std::f64::consts::FRAC_PI_4,
    0.0,
    std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_4,
];

/// Picks [`RealtimeConfig`] from `FRANKA_REALTIME` (`"ignore"` or `"enforce"`, default
/// `"enforce"` like libfranka) and prints the choice. Every ported example here uses it.
pub fn realtime_config_from_env() -> RealtimeConfig {
    let config = match std::env::var("FRANKA_REALTIME") {
        Ok(value) if value == "ignore" => RealtimeConfig::Ignore,
        Ok(value) if value == "enforce" => RealtimeConfig::Enforce,
        Ok(other) => {
            eprintln!(
                "FRANKA_REALTIME={other:?} is neither \"ignore\" nor \"enforce\"; using \
                 \"enforce\"."
            );
            RealtimeConfig::Enforce
        }
        Err(_) => RealtimeConfig::Enforce,
    };
    println!(
        "RealtimeConfig: {}",
        match config {
            RealtimeConfig::Enforce => "enforce",
            RealtimeConfig::Ignore => "ignore",
        }
    );
    config
}

/// Sets a default collision behavior, joint impedance and Cartesian impedance.
///
/// Port of `setDefaultBehavior` (`examples/examples_common.cpp`), verbatim thresholds.
pub fn set_default_behavior(robot: &Robot) -> FrankaResult<()> {
    robot.set_collision_behavior(
        [20.0; 7], [20.0; 7], [10.0; 7], [10.0; 7], [20.0; 6], [20.0; 6], [10.0; 6], [10.0; 6],
    )?;
    robot.set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])?;
    robot.set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])?;
    Ok(())
}

/// Waits for the user to press Enter, like the examples' `std::cin.ignore()` after printing the
/// "this will move the robot" warning.
pub fn wait_for_enter() {
    println!(
        "WARNING: This example will move the robot! Please make sure to have the user stop \
         button at hand!"
    );
    println!("Press Enter to continue...");
    let mut discard = String::new();
    let _ = std::io::stdin().read_line(&mut discard);
}

/// Threshold below which a joint's motion is considered finished (`kDeltaQMotionFinished`).
const DELTA_Q_MOTION_FINISHED: f64 = 1e-6;

/// Eigen's `cwiseSign`: `0` maps to `0`, not `+1`/`-1` like [`f64::signum`].
fn sign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// A point-to-point joint motion generator to a goal configuration, following a
/// third-order (jerk-limited) synchronized trajectory.
///
/// Port of `MotionGenerator` (`examples/examples_common.h` / `.cpp`), adapted from: Wisama
/// Khalil and Etienne Dombre, *Modeling, Identification and Control of Robots* (Kogan Page
/// Science Paper edition, 2002). libfranka's `operator()` is the callable passed to
/// `robot.control(motion_generator)`; here that is [`MotionGenerator::step`], called from a
/// closure passed to [`Robot::control_joint_positions`].
pub struct MotionGenerator {
    /// Which libfranka the seed must imitate; see [`MotionGenerator::step`].
    version: FciVersion,
    q_goal: [f64; 7],

    q_start: [f64; 7],
    delta_q: [f64; 7],

    dq_max_sync: [f64; 7],
    t_1_sync: [f64; 7],
    t_2_sync: [f64; 7],
    t_f_sync: [f64; 7],
    q_1: [f64; 7],

    time: f64,

    dq_max: [f64; 7],
    ddq_max_start: [f64; 7],
    ddq_max_goal: [f64; 7],
}

impl MotionGenerator {
    /// Creates a new `MotionGenerator` for a target `q_goal`, with `speed_factor` in `[0, 1]`
    /// scaling libfranka's default per-joint velocity/acceleration limits.
    /// `version` selects which libfranka's seeding behaviour to reproduce; pass
    /// [`Robot::fci_version`]. It is not optional because getting it wrong trips a reflex on
    /// an FER -- see [`MotionGenerator::step`].
    pub fn new(version: FciVersion, speed_factor: f64, q_goal: [f64; 7]) -> MotionGenerator {
        let mut dq_max = [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5];
        let mut ddq_max_start = [5.0; 7];
        let mut ddq_max_goal = [5.0; 7];
        for v in &mut dq_max {
            *v *= speed_factor;
        }
        for v in &mut ddq_max_start {
            *v *= speed_factor;
        }
        for v in &mut ddq_max_goal {
            *v *= speed_factor;
        }
        MotionGenerator {
            version,
            q_goal,
            q_start: [0.0; 7],
            delta_q: [0.0; 7],
            dq_max_sync: [0.0; 7],
            t_1_sync: [0.0; 7],
            t_2_sync: [0.0; 7],
            t_f_sync: [0.0; 7],
            q_1: [0.0; 7],
            time: 0.0,
            dq_max,
            ddq_max_start,
            ddq_max_goal,
        }
    }

    /// Port of `MotionGenerator::calculateDesiredValues`.
    fn calculate_desired_values(&self, time: f64) -> ([f64; 7], bool) {
        let mut delta_q_d = [0.0; 7];
        let mut joint_motion_finished = [false; 7];
        for i in 0..7 {
            let sign_delta_q = sign(self.delta_q[i]);
            let t_d = self.t_2_sync[i] - self.t_1_sync[i];
            let delta_t_2_sync = self.t_f_sync[i] - self.t_2_sync[i];
            if self.delta_q[i].abs() < DELTA_Q_MOTION_FINISHED {
                delta_q_d[i] = 0.0;
                joint_motion_finished[i] = true;
            } else if time < self.t_1_sync[i] {
                delta_q_d[i] = -1.0 / self.t_1_sync[i].powi(3)
                    * self.dq_max_sync[i]
                    * sign_delta_q
                    * (0.5 * time - self.t_1_sync[i])
                    * time.powi(3);
            } else if time < self.t_2_sync[i] {
                delta_q_d[i] =
                    self.q_1[i] + (time - self.t_1_sync[i]) * self.dq_max_sync[i] * sign_delta_q;
            } else if time < self.t_f_sync[i] {
                delta_q_d[i] = self.delta_q[i]
                    + 0.5
                        * (1.0 / delta_t_2_sync.powi(3)
                            * (time - self.t_1_sync[i] - 2.0 * delta_t_2_sync - t_d)
                            * (time - self.t_1_sync[i] - t_d).powi(3)
                            + (2.0 * time - 2.0 * self.t_1_sync[i] - delta_t_2_sync - 2.0 * t_d))
                        * self.dq_max_sync[i]
                        * sign_delta_q;
            } else {
                delta_q_d[i] = self.delta_q[i];
                joint_motion_finished[i] = true;
            }
        }
        let finished = joint_motion_finished.iter().all(|&f| f);
        (delta_q_d, finished)
    }

    /// Port of `MotionGenerator::calculateSynchronizedValues`.
    fn calculate_synchronized_values(&mut self) {
        let mut dq_max_reach = self.dq_max;
        let mut t_f = [0.0; 7];
        let mut t_1 = [0.0; 7];
        let mut delta_t_2 = [0.0; 7];

        for i in 0..7 {
            if self.delta_q[i].abs() > DELTA_Q_MOTION_FINISHED {
                let sign_delta_q = sign(self.delta_q[i]);
                if self.delta_q[i].abs()
                    < (3.0 / 4.0 * (self.dq_max[i].powi(2) / self.ddq_max_start[i])
                        + 3.0 / 4.0 * (self.dq_max[i].powi(2) / self.ddq_max_goal[i]))
                {
                    dq_max_reach[i] = (4.0 / 3.0
                        * self.delta_q[i]
                        * sign_delta_q
                        * (self.ddq_max_start[i] * self.ddq_max_goal[i])
                        / (self.ddq_max_start[i] + self.ddq_max_goal[i]))
                        .sqrt();
                }
                t_1[i] = 1.5 * dq_max_reach[i] / self.ddq_max_start[i];
                delta_t_2[i] = 1.5 * dq_max_reach[i] / self.ddq_max_goal[i];
                t_f[i] =
                    t_1[i] / 2.0 + delta_t_2[i] / 2.0 + self.delta_q[i].abs() / dq_max_reach[i];
            }
        }

        let max_t_f = t_f.iter().copied().fold(0.0, f64::max);
        for i in 0..7 {
            if self.delta_q[i].abs() > DELTA_Q_MOTION_FINISHED {
                let sign_delta_q = sign(self.delta_q[i]);
                let a = 1.5 / 2.0 * (self.ddq_max_goal[i] + self.ddq_max_start[i]);
                let b = -max_t_f * self.ddq_max_goal[i] * self.ddq_max_start[i];
                let c = self.delta_q[i].abs() * self.ddq_max_goal[i] * self.ddq_max_start[i];
                let delta = (b * b - 4.0 * a * c).max(0.0);
                self.dq_max_sync[i] = (-b - delta.sqrt()) / (2.0 * a);
                self.t_1_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_start[i];
                let delta_t_2_sync = 1.5 * self.dq_max_sync[i] / self.ddq_max_goal[i];
                self.t_f_sync[i] = self.t_1_sync[i] / 2.0
                    + delta_t_2_sync / 2.0
                    + (self.delta_q[i] / self.dq_max_sync[i]).abs();
                self.t_2_sync[i] = self.t_f_sync[i] - delta_t_2_sync;
                self.q_1[i] = self.dq_max_sync[i] * sign_delta_q * (0.5 * self.t_1_sync[i]);
            }
        }
    }

    /// Computes the next joint position setpoint. Port of `MotionGenerator::operator()`; pass
    /// `|state, period| motion_generator.step(state, period)` to
    /// [`Robot::control_joint_positions`].
    pub fn step(&mut self, robot_state: &RobotState, period: Duration) -> JointPositions {
        self.time += period.as_secs_f64();

        if self.time == 0.0 {
            // libfranka seeds this differently per version, and on FCI v5 the difference is
            // not cosmetic. 0.9.2 `examples/examples_common.cpp:119` seeds from the
            // *commanded* `q_d`; 0.21.2 `:120` (and 0.20.4) seed from the *measured* `q`.
            //
            // Seeding from `q` is safe on v10 only because v10's `convertMotion` makes the
            // first command its own rate-limit reference, so it implies no commanded
            // velocity. libfranka 0.9.2 has no such case: it always references the first
            // command against the robot's own `q_d`, so a `q` seed emits a first command
            // that implies a non-zero commanded joint velocity (the impedance deflection over
            // one 1 kHz tick, clamped by the limiter but not to zero). When the arm already
            // sits at the goal the generator additionally reports `motion_finished` on that
            // very first callback, and ending a `Move` while a non-zero velocity is commanded
            // is a `joint_motion_generator_velocity_discontinuity` reflex.
            self.q_start = match self.version {
                FciVersion::V5 => robot_state.q_d,
                FciVersion::V10 => robot_state.q,
            };
            for i in 0..7 {
                self.delta_q[i] = self.q_goal[i] - self.q_start[i];
            }
            self.calculate_synchronized_values();
        }

        let (delta_q_d, motion_finished) = self.calculate_desired_values(self.time);
        let mut q = self.q_start;
        for i in 0..7 {
            q[i] += delta_q_d[i];
        }
        let mut output = JointPositions::new(q);
        output.motion_finished = motion_finished;
        output
    }
}
