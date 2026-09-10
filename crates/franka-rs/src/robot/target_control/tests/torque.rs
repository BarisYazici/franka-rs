//! The scaffolding of the torque-loop tests: the two loops on a fresh slot, the hold assertion.

use std::sync::mpsc;
use std::sync::Arc;

use super::super::cartesian;
use super::super::joint;
use super::super::torque::{JointTracker, PoseTracker, TorqueLoop};
use super::super::*;
use crate::model::Model;
use crate::wire::robot::codec::FciVersion;

pub(super) type PoseLoop = TorqueLoop<6, 7, PoseTracker>;
pub(super) type JointLoop = TorqueLoop<7, 7, JointTracker>;

/// A Cartesian torque loop with a fresh slot and a `started` receiver.
pub(super) fn cartesian_loop(
    options: TargetControlOptions,
    impedance: ImpedanceOptions,
) -> (PoseLoop, Arc<Shared<7>>, mpsc::Receiver<()>) {
    let shared = Arc::new(Shared::<7>::default());
    let (started, first_cycle) = mpsc::sync_channel(1);
    let model = Arc::new(Model::native_fer());
    let torque = cartesian::torque_loop(
        options,
        impedance,
        model,
        FciVersion::V5,
        Arc::clone(&shared),
        started,
    )
    .unwrap();
    (torque, shared, first_cycle)
}

/// A joint torque loop at the FER's default limits with a fresh slot.
pub(super) fn joint_loop(
    options: JointTargetControlOptions,
    impedance: ImpedanceOptions,
) -> (JointLoop, Arc<Shared<7>>, mpsc::Receiver<()>) {
    let shared = Arc::new(Shared::<7>::default());
    let (started, first_cycle) = mpsc::sync_channel(1);
    let limits = JointTargetControlOptions::scaled_limits(FciVersion::V5, DEFAULT_LIMIT_FRACTION);
    let model = Arc::new(Model::native_fer());
    let torque = joint::torque_loop(
        options,
        limits,
        impedance,
        model,
        Arc::clone(&shared),
        started,
    )
    .unwrap();
    (torque, shared, first_cycle)
}

pub(super) fn bits<const N: usize>(p: &[f64; N]) -> [u64; N] {
    p.map(f64::to_bits)
}

/// The last `Settle::cycles + 1` records are the hold: within `tolerance` of each other by
/// `key` (bit-identical at 0), begun at the window.
pub(super) fn assert_held<S, const N: usize>(
    records: &[S],
    settle: Settle,
    tolerance: f64,
    key: impl Fn(&S) -> [f64; N],
) {
    let window = settle.cycles as usize + 1;
    let held = &records[records.len() - window..];
    let first = key(&held[0]);
    let same = |r: &S| {
        let k = key(r);
        if tolerance == 0.0 {
            bits(&k) == bits(&first)
        } else {
            k.iter()
                .zip(&first)
                .all(|(a, b)| (a - b).abs() <= tolerance)
        }
    };
    assert!(held.iter().all(same), "the hold is not held");
    assert!(
        !same(&records[records.len() - window - 10]),
        "the hold started earlier than the settle window"
    );
}

pub(super) fn peak_abs(v: &[f64; 7]) -> f64 {
    v.iter().fold(0.0, |peak, x| peak.max(x.abs()))
}

/// Cycles `cycle` until it reports the finish, that cycle included.
pub(super) fn cycles_to_finish(mut cycle: impl FnMut() -> bool) -> u32 {
    let mut cycles = 0;
    loop {
        cycles += 1;
        if cycle() {
            return cycles;
        }
        assert!(cycles < 8000, "the loop never finished");
    }
}
