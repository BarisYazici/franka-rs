//! The realtime side of [target control](super), generic over the interface: anchoring, the
//! slot, the generator under the three rules, the deviation guard, and the stop path --
//! land, hold the echo of the last command for the settle window, finish on one more of it.

use std::sync::atomic::Ordering;
use std::sync::mpsc::SyncSender;
use std::sync::Arc;

use super::{Settle, Shared, DEVIATION_MESSAGE, STOP_TIMEOUT_CYCLES};
use crate::error::{ControlException, FrankaError, FrankaResult};
use crate::otg::{MultiOtg, OtgLimits};
use crate::rate_limiting::DELTA_T;
use crate::robot_state::RobotState;

/// A generator within [`Settle::tolerance`] of its target moving slower than this, per axis,
/// counts as landed (m/s, rad/s). The hold then freezes a velocity step of at most this in
/// one cycle, a jerk of 100 per second cubed, which the joint side of a Cartesian command
/// amplifies threefold (1 mm/s froze as 3840 rad/s^3 on joint 2 in the simulator, over its
/// 3750). Not smaller: re-anchoring on a float32 echo (FCI v10) keeps a landed generator in
/// micro-profiles of a few 1e-8 that peak at about 2e-5 per second.
pub const REST_VELOCITY: f64 = 1e-4;
/// ... and accelerating less than this (m/s^2, rad/s^2); the micro-profiles above peak at
/// about 0.01.
pub const REST_ACCELERATION: f64 = 0.05;
/// An *arm* whose every joint moves slower than this, rad/s, is at rest: what a stop in torque
/// mode waits for before `motion_finished`, the hold having settled only the generator.
pub const REST_JOINT_VELOCITY: f64 = 0.01;

/// Maps the slot's target and the robot's echo (`S` values each) to where the generator is
/// re-anchored and where it is to go (`N` values each) for one cycle.
pub(super) type Chart<const N: usize, const S: usize> =
    fn(target: &[f64; S], commanded: &[f64; S]) -> ([f64; N], [f64; N]);

/// What one cycle produced.
pub(super) struct Step<const N: usize, const S: usize> {
    pub position: [f64; N],
    pub velocity: [f64; N],
    pub acceleration: [f64; N],
    /// The slot's target the generator was planning towards.
    pub target: [f64; S],
    /// The command is being held bit-identical; the backstop must not touch it.
    pub hold: bool,
    pub finished: bool,
}

/// The stop's final phase: the same position, sent `sent` times so far.
struct Hold<const N: usize> {
    position: [f64; N],
    sent: u32,
}

/// See the [module documentation](self).
pub(super) struct Runner<const N: usize, const S: usize> {
    shared: Arc<Shared<S>>,
    started: Option<SyncSender<()>>,
    otg: MultiOtg<N>,
    settle: Settle,
    chart: Chart<N, S>,
    target: [f64; S],
    anchored: bool,
    deviated: bool,
    stop_cycles: u32,
    hold: Option<Hold<N>>,
}

/// The chart of an interface whose slot and generator share their coordinates.
pub(super) fn identity<const N: usize>(
    target: &[f64; N],
    commanded: &[f64; N],
) -> ([f64; N], [f64; N]) {
    (*commanded, *target)
}

impl<const N: usize, const S: usize> Runner<N, S> {
    pub(super) fn new(
        shared: Arc<Shared<S>>,
        started: SyncSender<()>,
        limits: [OtgLimits; N],
        settle: Settle,
        chart: Chart<N, S>,
    ) -> FrankaResult<Self> {
        Ok(Runner {
            shared,
            started: Some(started),
            // Synchronised axes: a diagonal target moves along a straight line, and a pose
            // target's translation and rotation arrive together.
            otg: MultiOtg::with_limits([0.0; N], limits, true)?,
            settle,
            chart,
            target: [0.0; S],
            anchored: false,
            deviated: false,
            stop_cycles: 0,
            hold: None,
        })
    }

    /// One cycle: `commanded` is the robot's echo of the last command and `strayed` whether
    /// the deviation guard's threshold is crossed (`false` until anchored).
    pub(super) fn cycle(
        &mut self,
        state: &RobotState,
        commanded: [f64; S],
        strayed: bool,
    ) -> Step<N, S> {
        if let Ok(mut latest) = self.shared.state.try_lock() {
            *latest = *state;
        }
        if let Some(hold) = &mut self.hold {
            // Neither re-anchored nor stepped: the very same command again, and
            // `motion_finished` on the one after the settle window.
            hold.sent += 1;
            return Step {
                position: hold.position,
                velocity: [0.0; N],
                acceleration: [0.0; N],
                target: self.target,
                hold: true,
                finished: hold.sent > self.settle.cycles,
            };
        }
        // Read before the slot: a target published before `stop()` is then never missed.
        let stopping = self.deviated || self.shared.stop.load(Ordering::SeqCst);
        if !self.anchored {
            self.anchored = true;
            self.target = commanded;
            self.otg.reset((self.chart)(&commanded, &commanded).0);
            self.shared.slot.publish(commanded);
            if let Some(started) = self.started.take() {
                let _ = started.try_send(());
            }
        } else if !self.deviated && strayed {
            self.deviated = true;
            self.target = commanded;
        } else if !self.deviated {
            // A torn read keeps the previous target for this one cycle; a stop, which no
            // publish can follow, waits for the final one.
            while !self.shared.slot.load(&mut self.target) && stopping {}
        }
        // The echo and the target are finite, so neither setter fails.
        let (anchor, goal) = (self.chart)(&self.target, &commanded);
        let _ = self.otg.set_position(anchor);
        let _ = self.otg.set_target(goal);
        let position = self.otg.step(DELTA_T);
        let axes = self.otg.axes();
        let velocity = std::array::from_fn(|i| axes[i].velocity());
        let acceleration = std::array::from_fn(|i| axes[i].acceleration());

        if stopping || self.deviated {
            self.stop_cycles = self.stop_cycles.saturating_add(1);
            let landed = axes.iter().all(|a| {
                (a.position() - a.target()).abs() < self.settle.tolerance
                    && a.velocity().abs() < REST_VELOCITY
                    && a.acceleration().abs() < REST_ACCELERATION
            });
            // Landed, or out of patience: hold the robot's echo of the last command, which
            // is continuous with what the robot already has by construction (the generator's
            // own position may differ from it by whatever the backstop took off the last
            // command), and this is the first of the identical ones.
            if landed || self.stop_cycles >= STOP_TIMEOUT_CYCLES {
                let held = (self.chart)(&commanded, &commanded).0;
                self.hold = Some(Hold {
                    position: held,
                    sent: 1,
                });
                return Step {
                    position: held,
                    velocity: [0.0; N],
                    acceleration: [0.0; N],
                    target: self.target,
                    hold: true,
                    finished: false,
                };
            }
        }
        Step {
            position,
            velocity,
            acceleration,
            target: self.target,
            hold: false,
            finished: false,
        }
    }

    /// The loop's result: a regular end after the deviation guard fired is the error it is.
    pub(super) fn finish(&self, result: FrankaResult<()>) -> FrankaResult<()> {
        match result {
            Ok(()) if self.deviated => Err(FrankaError::Control(ControlException::new(
                DEVIATION_MESSAGE,
            ))),
            other => other,
        }
    }
}
