//! Helpers shared by the two model conformance suites, `model_conformance.rs`
//! (FR3, FCI v10) and `fer_model_conformance.rs` (FER, FCI v5).
//!
//! Both suites measure alternative model backends against a reference; the
//! max-absolute-error accounting is identical in both, so it lives here.

// Each suite uses a subset of these; the other suite's half is dead code in its
// binary.
#![allow(dead_code)]

/// The largest absolute error seen for one quantity, with the sample that
/// produced it.
#[derive(Default, Clone)]
pub struct MaxError {
    pub value: f64,
    pub detail: String,
}

impl MaxError {
    /// Keeps `error` if it is the largest so far, recording `detail()` with it.
    pub fn observe(&mut self, error: f64, detail: impl FnOnce() -> String) {
        if error > self.value {
            self.value = error;
            self.detail = detail();
        }
    }

    /// `Some(message)` when the worst error exceeds `tolerance`.
    pub fn check(&self, name: &str, tolerance: f64) -> Option<String> {
        if self.value > tolerance {
            Some(format!(
                "{name}: max |error| {:.3e} > {tolerance:.0e} at {}",
                self.value, self.detail
            ))
        } else {
            None
        }
    }
}

/// Feeds every element-wise difference between `actual` and `expected` into `max`.
pub fn compare(
    max: &mut MaxError,
    actual: &[f64],
    expected: &[f64],
    context: &str,
    sample: usize,
    config: usize,
) {
    assert_eq!(actual.len(), expected.len(), "{context}: length mismatch");
    for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
        max.observe((a - e).abs(), || {
            format!("sample {sample}, config {config}, {context}, entry {i} (got {a}, want {e})")
        });
    }
}
