//! The lock-free mailbox between a commander and the realtime loop.

use std::sync::atomic::{AtomicU64, Ordering};

/// The latest target, `N` `f64`s, as a single-writer seqlock: the sequence number is odd
/// while a write is in progress and changes with every write, so a reader that sees the
/// same even number before and after loading the values has a consistent set. Reads never
/// block and never spin unboundedly, which is what lets the realtime loop poll it every
/// cycle; writes are wait-free too, but there must be only one writer at a time, which the
/// handles serialise with a mutex on the user side.
///
/// The [`Default`] slot holds zeros with sequence 0, which [`TargetSlot::load`] reports as
/// "never written" through its `false` return: the loop then keeps the target it has.
#[derive(Debug)]
pub struct TargetSlot<const N: usize> {
    sequence: AtomicU64,
    values: [AtomicU64; N],
}

impl<const N: usize> Default for TargetSlot<N> {
    fn default() -> Self {
        Self {
            sequence: AtomicU64::new(0),
            values: [const { AtomicU64::new(0) }; N],
        }
    }
}

impl<const N: usize> TargetSlot<N> {
    /// A slot holding `initial`, already published (sequence 2).
    pub fn new(initial: [f64; N]) -> Self {
        let slot = Self::default();
        slot.publish(initial);
        slot
    }

    /// Publishes `target`. The single writer; wait-free.
    pub fn publish(&self, target: [f64; N]) {
        self.sequence.fetch_add(1, Ordering::SeqCst);
        for (slot, value) in self.values.iter().zip(target) {
            slot.store(value.to_bits(), Ordering::SeqCst);
        }
        self.sequence.fetch_add(1, Ordering::SeqCst);
    }

    /// Copies the latest consistent target into `into` and returns `true`; `false`, with
    /// `into` untouched, if the writer was mid-update on every one of three tries or nothing
    /// has been published yet.
    pub fn load(&self, into: &mut [f64; N]) -> bool {
        for _ in 0..3 {
            let before = self.sequence.load(Ordering::SeqCst);
            if before == 0 || before & 1 == 1 {
                continue;
            }
            let candidate = self
                .values
                .each_ref()
                .map(|slot| f64::from_bits(slot.load(Ordering::SeqCst)));
            if self.sequence.load(Ordering::SeqCst) == before {
                *into = candidate;
                return true;
            }
        }
        false
    }

    /// The number of completed writes, times two (the sequence number).
    pub fn sequence(&self) -> u64 {
        self.sequence.load(Ordering::SeqCst)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::{sync_channel, TryRecvError};

    #[test]
    fn an_unwritten_slot_reports_nothing_and_leaves_the_output_alone() {
        let slot = TargetSlot::<3>::default();
        let mut into = [1.0, 2.0, 3.0];
        assert!(!slot.load(&mut into));
        assert_eq!(into, [1.0, 2.0, 3.0]);
        assert_eq!(slot.sequence(), 0);
    }

    #[test]
    fn publish_then_load_round_trips_every_bit_pattern() {
        let slot = TargetSlot::<4>::new([0.0, -0.0, f64::MIN_POSITIVE, -1e300]);
        let mut into = [f64::NAN; 4];
        assert!(slot.load(&mut into));
        assert_eq!(
            into.map(f64::to_bits),
            [0.0, -0.0, f64::MIN_POSITIVE, -1e300].map(f64::to_bits)
        );
        assert_eq!(slot.sequence(), 2);
        slot.publish([1.0, 2.0, 3.0, 4.0]);
        assert!(slot.load(&mut into));
        assert_eq!(into, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(slot.sequence(), 4);
    }

    #[test]
    fn a_reader_never_sees_a_torn_triple_under_a_busy_writer() {
        // Every published triple is `[k, 2k, 3k]`; a torn read would break that invariant.
        const ROUNDS: u32 = 200;
        const WRITES_PER_ROUND: u32 = 1_000;
        let slot = TargetSlot::<3>::new([0.0; 3]);
        let (done_tx, done_rx) = sync_channel(0);
        let (resume_tx, resume_rx) = sync_channel(0);
        let slot = &slot;
        std::thread::scope(move |threads| {
            threads.spawn(move || {
                for round in 1..=ROUNDS {
                    let last = round * WRITES_PER_ROUND;
                    for k in last - WRITES_PER_ROUND + 1..=last {
                        let k = f64::from(k);
                        slot.publish([k, 2.0 * k, 3.0 * k]);
                    }
                    // Guarantee observable updates without assuming how the scheduler divides
                    // time between the threads. A failed reader disconnects these channels.
                    if done_tx.send(last).is_err() || resume_rx.recv().is_err() {
                        return;
                    }
                }
            });
            let mut target = [0.0; 3];
            for round in 1..=ROUNDS {
                loop {
                    if slot.load(&mut target) {
                        assert_eq!(target[1], 2.0 * target[0], "torn read {target:?}");
                        assert_eq!(target[2], 3.0 * target[0], "torn read {target:?}");
                    }
                    match done_rx.try_recv() {
                        Ok(last) => {
                            assert_eq!(last, round * WRITES_PER_ROUND);
                            break;
                        }
                        Err(TryRecvError::Empty) => {}
                        Err(TryRecvError::Disconnected) => panic!("writer stopped early"),
                    }
                }
                // The paused writer gives every round a successful read, even on one CPU.
                assert!(slot.load(&mut target));
                let last = f64::from(round * WRITES_PER_ROUND);
                assert_eq!(target, [last, 2.0 * last, 3.0 * last]);
                resume_tx.send(()).unwrap();
            }
        });
    }
}
