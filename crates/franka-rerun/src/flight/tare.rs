//! The baseline of `tau_ext_hat_filtered` the contact estimate is taken against.

/// A running baseline of `tau_ext_hat_filtered` while nothing touches the arm, subtracted
/// before estimating: an exponential average with a time constant of [`Tare::RECORDS`]
/// records, updated only while no flag is set and every torque is within the noise floor of
/// the baseline, frozen otherwise (that is, through a contact).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Tare {
    /// The baseline so far, `None` before the first record.
    pub baseline: Option<[f64; 7]>,
}

impl Tare {
    /// Time constant of the average, records (a second at 1 kHz).
    pub const RECORDS: f64 = 1000.0;

    /// Folds `tau` in when `quiet` (no flag set) and it is within `floor` of the baseline;
    /// returns `tau` less the baseline.
    pub fn apply(&mut self, tau: &[f64; 7], quiet: bool, floor: f64) -> [f64; 7] {
        let baseline = self.baseline.get_or_insert(*tau);
        let within = tau
            .iter()
            .zip(baseline.iter())
            .all(|(t, b)| (t - b).abs() < floor);
        if quiet && within {
            for (b, t) in baseline.iter_mut().zip(tau) {
                *b += (t - *b) / Tare::RECORDS;
            }
        }
        std::array::from_fn(|k| tau[k] - baseline[k])
    }
}
