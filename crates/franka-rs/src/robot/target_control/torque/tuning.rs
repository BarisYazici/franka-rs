//! The loop's end of [live tuning](super::super::tuning): read the slot, cross to what it
//! holds, and write the result into the places the options were copied into at the session's
//! start.
//!
//! Every step is on the 1 kHz thread, so none of it allocates, locks, fails or loops without a
//! bound. The read is one atomic load in the common case and
//! [`WORDS`](LiveTuning::WORDS) more only on a cycle the writer moved; a read that tears is left
//! for the next cycle, which costs one cycle of a crossing that takes three hundred. The gains
//! are [`LiveTuning::slew_gains`], built once at construction, so no `exp` runs here.
//!
//! The fan-out is where the copies are undone. `ImpedanceOptions` is read afresh every cycle and
//! needs only to be written, but the IK's `λ` is read from two places in the same cycle -- the
//! law's nullspace projector out of the shared options, and the solver out of its own copy --
//! and a cycle in which they disagree is a projector built for a different solve. They are
//! written together here, from one value, and [`Tracker::retune`] is the only writer of the
//! tracker's copies.
//!
//! The budget is the one value that reaches something other than the law: the generator's
//! per-axis limits. What holds the clamp a narrower budget puts on the generator's stored state
//! to one cycle of the next order is the *crossing's rate*, not where in the cycle this runs --
//! see [`TuningPolicy::StepUpGateDown`](super::super::TuningPolicy). Running at the top of the
//! cycle buys two other things: the values arrive a cycle sooner, and everything the cycle then
//! does -- the anchor, the generator's step, the solve, the law -- runs on one set of them,
//! which is the same rule as the `λ` above, one step out.

use super::super::tuning::{LiveTuning, SlewGains};
use super::super::TargetSlot;
use super::{TorqueLoop, Tracker};
use crate::rate_limiting::DELTA_T;

#[cfg(test)]
use super::super::ik::IkOptions;
#[cfg(test)]
use super::super::ImpedanceOptions;
#[cfg(test)]
use super::PoseTracker;

/// The loop's live-tuning state: what the slot last said, what is in force, and how far one
/// cycle moves the second toward the first.
pub(super) struct Tuning {
    /// The sequence the last successful load came from, so an untouched slot costs one atomic
    /// load per cycle. Advanced only on a load that succeeded: a tear then leaves it behind and
    /// is retried next cycle instead of being skipped for good.
    sequence: u64,
    /// The load's landing place, a field rather than a local so that the read owns no stack
    /// buffer it has to build.
    words: [f64; LiveTuning::WORDS],
    target: LiveTuning,
    current: LiveTuning,
    gains: SlewGains,
}

impl Tuning {
    /// Seeded where the session's options already are, so `current` and `target` agree and
    /// nothing is applied until something is tuned.
    pub(super) fn new(seed: LiveTuning) -> Tuning {
        Tuning {
            sequence: 0,
            words: seed.to_words(),
            target: seed,
            current: seed,
            gains: LiveTuning::slew_gains(DELTA_T),
        }
    }

    /// Takes the slot's targets if they moved, then crosses `current` one cycle toward them;
    /// `true` when something moved and has to be put into effect.
    fn cycle(&mut self, slot: &TargetSlot<{ LiveTuning::WORDS }>) -> bool {
        let sequence = slot.sequence();
        if sequence != self.sequence && slot.load(&mut self.words) {
            // The sequence read before the load, so a write that lands between the two is
            // recorded as unread and re-read next cycle. The other order would record a
            // sequence the load did not see and lose that write.
            self.sequence = sequence;
            self.target = LiveTuning::from_words(&self.words);
        }
        self.current.slew_toward(&self.target, &self.gains)
    }
}

impl<const N: usize, const S: usize, T: Tracker<N, S>> TorqueLoop<N, S, T> {
    /// One cycle of live tuning, at the top of [`cycle`](Self::cycle) so that the law, the IK
    /// and the tracker all run this cycle on the same values. A session without a tuning slot
    /// does nothing at all, not even the load.
    pub(super) fn retune(&mut self) {
        let Some(tuning) = self.tuning.as_mut() else {
            return;
        };
        if !tuning.cycle(self.runner.tuning()) {
            return;
        }
        let current = tuning.current;
        self.apply(&current);
    }

    /// `tuning` into the law and the tracker's copies of it.
    fn apply(&mut self, tuning: &LiveTuning) {
        self.impedance.gains = tuning.gains();
        self.impedance.ik.damping = tuning.ik_damping;
        self.impedance.ik.nullspace_gain = tuning.ik_nullspace_gain;
        // The switch rides in the weight: zero is the law with the feedforward off, and a
        // weight can be crossed where a switch can only be thrown.
        self.impedance.velocity_feedforward = true;
        self.impedance.velocity_feedforward_gain = tuning.velocity_feedforward_gain;
        self.impedance.velocity_feedforward_cutoff = tuning.velocity_feedforward_cutoff;
        // In force for the step this cycle takes, not the next one's: what bounds the clamp is
        // the crossing's rate, but which cycle first plans under the new limits is this.
        if let Some(limits) = self.tracker.budget(tuning) {
            self.runner.set_limits(limits);
        }
        // The tracker's own copies, from the same values in the same cycle: the law's nullspace
        // projector and the tracker's solver must never run a cycle on different `λ`s.
        self.tracker.retune(tuning);
    }
}

#[cfg(test)]
impl TorqueLoop<6, 7, PoseTracker> {
    /// The two places a retune has to have reached in the same cycle: the shared options the
    /// law reads, and the solver's own copy.
    pub(in super::super) fn tuned(&self) -> (ImpedanceOptions, Option<IkOptions>) {
        (self.impedance, self.tracker.solver_ik())
    }

    /// The generator's per-axis velocity and the per-axis limits it is planning under: the
    /// budget's end of a retune, where the gate's promise about the command is checked.
    pub(in super::super) fn generator(&self) -> ([f64; 6], [crate::otg::OtgLimits; 6]) {
        let axes = self.runner.axes();
        (axes.map(|a| a.velocity()), axes.map(|a| a.limits()))
    }

    /// The tracker's landing dwell, and how often it has been derived; see
    /// [`PoseTracker::dwell`].
    pub(in super::super) fn dwell(&self) -> (u32, u32) {
        self.tracker.dwell()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::{sync_channel, TryRecvError};

    /// A tuning whose every word is `k`: the `k`-th publish into a fresh slot, whose sequence is
    /// then `2k`, so a set carries the version it came from.
    fn published(k: u64) -> LiveTuning {
        LiveTuning::from_words(&[k as f64; LiveTuning::WORDS])
    }

    /// The sequence is read *before* the load and recorded only for a load that succeeded, which
    /// makes it a lower bound on the version of the set in hand. Recorded after the load instead,
    /// it can name a set the loop never took: the loop then sees no change on every later cycle
    /// and holds a stale target for the rest of the session. The bound is asserted every cycle
    /// rather than at the end, because the window is a few instructions wide and the run that
    /// lands the *last* publish in it is rare.
    #[test]
    fn the_sequence_recorded_is_never_ahead_of_the_set_it_was_read_with() {
        const ROUNDS: u64 = 200;
        const WRITES_PER_ROUND: u64 = 1_000;
        let slot = TargetSlot::<{ LiveTuning::WORDS }>::default();
        let (done_tx, done_rx) = sync_channel(0);
        let (resume_tx, resume_rx) = sync_channel(0);
        let slot = &slot;
        std::thread::scope(move |threads| {
            threads.spawn(move || {
                for round in 1..=ROUNDS {
                    let last = round * WRITES_PER_ROUND;
                    for k in last - WRITES_PER_ROUND + 1..=last {
                        slot.publish(published(k).to_words());
                    }
                    // Pause after each batch so even a reader sharing one CPU with the writer
                    // observes every round. Disconnecting also releases the writer on failure.
                    if done_tx.send(last).is_err() || resume_rx.recv().is_err() {
                        return;
                    }
                }
            });
            let mut tuning = Tuning::new(published(0));
            for round in 1..=ROUNDS {
                loop {
                    tuning.cycle(slot);
                    let version = 2 * tuning.target.joint_stiffness[0] as u64;
                    assert!(
                        tuning.sequence <= version,
                        "round {round}: sequence {} names a newer set than the one in hand ({version})",
                        tuning.sequence
                    );
                    match done_rx.try_recv() {
                        Ok(last) => {
                            assert_eq!(last, round * WRITES_PER_ROUND);
                            break;
                        }
                        Err(TryRecvError::Empty) => {}
                        Err(TryRecvError::Disconnected) => panic!("writer stopped early"),
                    }
                }
                // With the writer paused, one cycle must recover a read left behind by a tear
                // or a write after the sequence read. It must never skip that update for good.
                tuning.cycle(slot);
                let last = round * WRITES_PER_ROUND;
                assert_eq!(
                    tuning.target.to_words().map(f64::to_bits),
                    published(last).to_words().map(f64::to_bits),
                    "round {round}: the loop ended on a set the writer had replaced"
                );
                assert_eq!(tuning.sequence, 2 * last);
                resume_tx.send(()).unwrap();
            }
        });
    }
}
