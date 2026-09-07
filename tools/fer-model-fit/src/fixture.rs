//! The joint configurations and load configurations every command in this tool
//! evaluates on, and the JSON reference fixture dumped from the shared object.

use crate::fer::{DQ_MAX, Q_MAX, Q_MIN};

/// Gravity used everywhere here, matching libfranka's own default.
pub const GRAVITY_EARTH: [f64; 3] = [0.0, 0.0, -9.81];

/// A second, deliberately non-vertical gravity vector, as in the FR3 fixture.
pub const GRAVITY_EARTH_ALT: [f64; 3] = [0.1, -0.2, -9.7];

pub const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// `splitmix64`, so the sample set is reproducible without a dependency.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform in `[low, high]`.
    fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.unit()
    }
}

/// One joint configuration to evaluate at.
pub struct Sample {
    pub index: usize,
    pub kind: &'static str,
    pub q: [f64; 7],
    pub dq: [f64; 7],
}

/// The fixed part of the sample set: the zero pose, the "ready" pose and six
/// corners of the joint box, exactly as `tests/fer_model_conformance.rs` uses.
pub fn fixed_samples() -> Vec<Sample> {
    use std::f64::consts::PI;

    let mut out = Vec::with_capacity(8);
    out.push(Sample {
        index: 0,
        kind: "zero",
        q: [0.0; 7],
        dq: [0.0; 7],
    });
    out.push(Sample {
        index: 1,
        kind: "ready",
        q: [
            0.0,
            -PI / 4.0,
            0.0,
            -3.0 * PI / 4.0,
            0.0,
            PI / 2.0,
            PI / 4.0,
        ],
        dq: [1.0; 7],
    });

    const MASKS: [u8; 6] = [
        0b1010101, 0b0101010, 0b1100110, 0b0011001, 0b1111000, 0b0000111,
    ];
    for mask in MASKS {
        let mut q = [0.0; 7];
        let mut dq = [0.0; 7];
        for j in 0..7 {
            let upper = mask & (1 << j) != 0;
            q[j] = if upper { Q_MAX[j] } else { Q_MIN[j] };
            dq[j] = if upper { 0.5 } else { -0.5 };
        }
        out.push(Sample {
            index: out.len(),
            kind: "limit_corner",
            q,
            dq,
        });
    }
    out
}

/// The fixed samples followed by `count` pseudo-random ones drawn uniformly
/// from the joint box, with velocities inside the velocity limits.
pub fn samples(count: usize, seed: u64) -> Vec<Sample> {
    let mut out = fixed_samples();
    let mut rng = Rng::new(seed);
    for _ in 0..count {
        let mut q = [0.0; 7];
        let mut dq = [0.0; 7];
        for j in 0..7 {
            q[j] = rng.range(Q_MIN[j], Q_MAX[j]);
            dq[j] = rng.range(-DQ_MAX[j], DQ_MAX[j]);
        }
        out.push(Sample {
            index: out.len(),
            kind: "random",
            q,
            dq,
        });
    }
    out
}

/// A load configuration: the two flange offsets and the combined inertial
/// properties `franka::Model` is called with.
#[allow(non_snake_case)]
pub struct LoadConfig {
    pub name: &'static str,
    pub F_T_EE: [f64; 16],
    pub EE_T_K: [f64; 16],
    pub m_total: f64,
    pub F_x_Ctotal: [f64; 3],
    pub I_total: [f64; 9],
}

/// The four load configurations of `tests/fer_model_conformance.rs`: no load,
/// the Franka Hand plus a 0.5 kg payload, and that with two stiffness frames.
pub fn load_configs() -> Vec<LoadConfig> {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    // Franka Hand: -45 degrees about z and 0.1034 m along z.
    let hand: [f64; 16] = [
        s, -s, 0.0, 0.0, s, s, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1034, 1.0,
    ];
    let (s30, c30) = (30.0_f64).to_radians().sin_cos();
    let stiffness: [f64; 16] = [
        c30, s30, 0.0, 0.0, -s30, c30, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 1.0,
    ];
    let tilted_stiffness: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, c30, s30, 0.0, 0.0, -s30, c30, 0.0, 0.02, 0.0, 0.05, 1.0,
    ];
    let hand_and_load = |ee_t_k: [f64; 16], name: &'static str| LoadConfig {
        name,
        F_T_EE: hand,
        EE_T_K: ee_t_k,
        m_total: 1.23,
        F_x_Ctotal: [-0.0018699187, 0.0081300813, 0.03],
        I_total: [0.0031, 0.0, 0.0, 0.0, 0.0046, 0.0, 0.0, 0.0, 0.0049],
    };

    vec![
        LoadConfig {
            name: "no_load",
            F_T_EE: IDENTITY,
            EE_T_K: IDENTITY,
            m_total: 0.0,
            F_x_Ctotal: [0.0; 3],
            I_total: [0.0; 9],
        },
        hand_and_load(IDENTITY, "hand_and_load"),
        hand_and_load(stiffness, "hand_load_and_stiffness"),
        hand_and_load(tilted_stiffness, "hand_load_and_tilted_stiffness"),
    ]
}
