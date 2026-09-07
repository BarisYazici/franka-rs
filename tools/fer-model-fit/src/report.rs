//! Comparing two `Model`s (the shared object and a candidate URDF) across
//! every sample, load configuration and frame.

use franka::model::{Frame, Model};

use crate::fixture::{load_configs, Sample, GRAVITY_EARTH, GRAVITY_EARTH_ALT};

/// Per-quantity worst absolute difference, next to the magnitude of the
/// reference, so a difference can be read as a relative one.
#[derive(Default)]
pub struct Row {
    pub abs: f64,
    pub reference: f64,
}

impl Row {
    pub fn observe(&mut self, actual: &[f64], expected: &[f64]) {
        for (a, e) in actual.iter().zip(expected) {
            self.abs = self.abs.max((a - e).abs());
            self.reference = self.reference.max(e.abs());
        }
    }

    pub fn relative(&self) -> f64 {
        if self.reference > 0.0 {
            self.abs / self.reference
        } else {
            0.0
        }
    }
}

#[derive(Default)]
pub struct Report {
    pub pose: Row,
    pub body_jacobian: Row,
    pub zero_jacobian: Row,
    pub mass: Row,
    pub coriolis: Row,
    pub gravity: Row,
}

impl Report {
    fn rows(&self) -> [(&'static str, &Row); 6] {
        [
            ("pose (all 10 frames)", &self.pose),
            ("body_jacobian (all frames)", &self.body_jacobian),
            ("zero_jacobian (all frames)", &self.zero_jacobian),
            ("mass", &self.mass),
            ("coriolis", &self.coriolis),
            ("gravity", &self.gravity),
        ]
    }

    pub fn print(&self, title: &str) {
        println!("\n{title}");
        println!("| quantity | max abs diff | max |reference| | max abs / max ref |");
        println!("|---|---|---|---|");
        for (name, row) in self.rows() {
            println!(
                "| {name} | {:.3e} | {:.3e} | {:.3e} |",
                row.abs,
                row.reference,
                row.relative()
            );
        }
    }

    pub fn markdown(&self) -> String {
        let mut out = String::new();
        out.push_str(
            "| quantity | max abs difference | max \\|reference\\| | ratio |\n|---|---|---|---|\n",
        );
        for (name, row) in self.rows() {
            out.push_str(&format!(
                "| {name} | {:.3e} | {:.3e} | {:.3e} |\n",
                row.abs,
                row.reference,
                row.relative()
            ));
        }
        out
    }
}

/// Compares `candidate` against `reference` on every sample, load configuration
/// and frame.
pub fn compare(candidate: &Model, reference: &Model, samples: &[Sample]) -> Report {
    let mut report = Report::default();
    for sample in samples {
        for config in load_configs() {
            for frame in Frame::ALL {
                report.pose.observe(
                    &candidate.pose_q(frame, &sample.q, &config.F_T_EE, &config.EE_T_K),
                    &reference.pose_q(frame, &sample.q, &config.F_T_EE, &config.EE_T_K),
                );
                report.body_jacobian.observe(
                    &candidate.body_jacobian_q(frame, &sample.q, &config.F_T_EE, &config.EE_T_K),
                    &reference.body_jacobian_q(frame, &sample.q, &config.F_T_EE, &config.EE_T_K),
                );
                report.zero_jacobian.observe(
                    &candidate.zero_jacobian_q(frame, &sample.q, &config.F_T_EE, &config.EE_T_K),
                    &reference.zero_jacobian_q(frame, &sample.q, &config.F_T_EE, &config.EE_T_K),
                );
            }
            report.mass.observe(
                &candidate.mass_q(
                    &sample.q,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                ),
                &reference.mass_q(
                    &sample.q,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                ),
            );
            report.coriolis.observe(
                &candidate.coriolis_q(
                    &sample.q,
                    &sample.dq,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH,
                ),
                &reference.coriolis_q(
                    &sample.q,
                    &sample.dq,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH,
                ),
            );
            for g in [GRAVITY_EARTH, GRAVITY_EARTH_ALT] {
                report.gravity.observe(
                    &candidate.gravity_q(&sample.q, config.m_total, &config.F_x_Ctotal, &g),
                    &reference.gravity_q(&sample.q, config.m_total, &config.F_x_Ctotal, &g),
                );
            }
        }
    }
    report
}
