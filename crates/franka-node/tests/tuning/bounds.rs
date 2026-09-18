//! Reading `LiveTuning::BOUNDS` — the one table both binaries consult, so neither writes a
//! limit down.

// Shared by two test binaries: the asserting `sim_tuning` suite and the `sim_sweep` campaign
// script, which each use a different part of this module. Rust's dead-code analysis is per
// binary and has no view of the other one, so the parts one of them does not reach are not
// dead -- they are the other's. It is a blanket allow: something that went dead in *both*
// would be silent here, so a reader deleting from this module should check both binaries.
#![allow(dead_code)]

use franka::robot::target_control::{FieldBound, LiveTuning, TuningDanger};

/// The row of one word of the table, by name and element.
pub fn bound(name: &str, index: Option<usize>) -> &'static FieldBound {
    LiveTuning::BOUNDS
        .iter()
        .find(|b| b.name == name && b.index == index)
        .unwrap_or_else(|| panic!("{name}{index:?} has no bounds row"))
}

/// That word's offset into [`LiveTuning::to_words`], which is the table's own order.
pub fn word_of(name: &str, index: Option<usize>) -> usize {
    LiveTuning::BOUNDS
        .iter()
        .position(|b| b.name == name && b.index == index)
        .unwrap_or_else(|| panic!("{name}{index:?} has no bounds row"))
}

/// The value above which the table asks for a deliberate confirmation, or `None` where the word
/// carries no such flag.
pub fn confirm_above(name: &str, index: Option<usize>) -> Option<f64> {
    match bound(name, index).danger {
        Some(TuningDanger::ConfirmAbove(at)) => Some(at),
        _ => None,
    }
}

/// [`confirm_above`] where the caller requires the word to have one.
pub fn must_confirm_above(name: &str, index: Option<usize>) -> f64 {
    confirm_above(name, index)
        .unwrap_or_else(|| panic!("{name}{index:?} is not a ConfirmAbove field"))
}
