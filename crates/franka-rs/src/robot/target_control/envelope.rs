//! The arms' joint velocity limits and the braking profile the guard brings a goal to rest on
//! its margin with.
//!
//! Franka's robot specifications give the FR3 a position-based velocity limit, per joint `i`:
//!
//! ```text
//! toward upper: min(dq_max, max(0, −dq_offset + sqrt(max(0, 2·ddq_dec·(q_max − q))))) − tolerance
//! toward lower: min(dq_max, max(0, −dq_offset + sqrt(max(0, 2·ddq_dec·(q − q_min))))) − tolerance
//! ```
//!
//! as speeds, never negative. The FER's specifications carry no such rows, and its recordings
//! rule the obvious assumption out (joint 5 ran at 2.26 times an envelope built on half its
//! acceleration limit, and the arm raised no reflex), so on the FER the robot's own check is the
//! published flat limit and that is what the guard uses. The guard's own braking toward a
//! position limit, [`Braking`], applies to both arms. Everything is read through
//! [`VelocityLimit`].

use crate::math_utils::{cmax, cmin};
use crate::rate_limiting::{fer, JOINT_VELOCITY_LIMITS_TOLERANCE};
use crate::wire::robot::codec::FciVersion;

/// One arm's position-dependent velocity envelope, per joint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Envelope {
    /// rad: the position limits the envelope is measured from.
    pub q_min: [f64; 7],
    pub q_max: [f64; 7],
    /// rad/s: the saturation away from the limits.
    pub dq_max: [f64; 7],
    pub dq_offset: [f64; 7],
    /// rad/s²
    pub ddq_dec: [f64; 7],
    /// rad/s taken off: libfranka's margin for rounding and lost packets.
    pub tolerance: [f64; 7],
}

/// The FR3, from the robot specifications page. libfranka's deprecated
/// `compute_*_limits_joint_velocity` (and its test URDF) use other offsets and position limits;
/// the recorded FR3 faults follow these.
pub(super) const FR3: Envelope = Envelope {
    q_min: [-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508],
    q_max: [2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508],
    dq_max: [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],
    dq_offset: [0.6599, 0.2517, 0.2000, 0.3533, 0.5757, 0.4878, 0.4628],
    ddq_dec: [6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0],
    tolerance: JOINT_VELOCITY_LIMITS_TOLERANCE,
};

/// The FER's nominal joint acceleration limits, rad/s², from the robot specifications page.
const FER_DDQ_MAX: [f64; 7] = [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0];

/// The guard's braking profile per joint, `E_i(x) = min(flat_i, sqrt(k_i·x + c_i²) − c_i)` rad/s
/// at `x` rad before the goal must be at rest. It is the guard's own bound, not a limit the
/// robot checks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Braking {
    /// `2a`, `a` rad/s² the deceleration the profile assumes: `E ≤ sqrt(2a·x)`, the speed a joint
    /// decelerating at `a` still comes to rest from in `x`.
    k: [f64; 7],
    /// rad/s: keeps the slope at the stop finite (`k/2c`) and the profile under `sqrt(2a·x)`
    /// everywhere. libfranka's FR3 rate limiter offsets, used on both arms.
    c: [f64; 7],
}

const C: [f64; 7] = [0.30, 0.20, 0.20, 0.30, 0.35, 0.35, 0.35];

const fn twice(a: [f64; 7]) -> [f64; 7] {
    let mut k = [0.0; 7];
    let mut i = 0;
    while i < 7 {
        k[i] = 2.0 * a[i];
        i += 1;
    }
    k
}

/// The FR3: `a` is the specifications' `ddq_dec`, 2.585 to 17 rad/s², the deceleration its own
/// envelope is built on, so `(k, c)` are libfranka's FR3 braking constants unchanged.
const FR3_BRAKING: Braking = Braking {
    k: twice(FR3.ddq_dec),
    c: C,
};

/// The FER: nothing is published to derive `a` from, so it is half the specifications' joint
/// acceleration limit, 7.5 / 3.75 / 5 / 6.25 / 7.5 / 10 / 10 rad/s² — the goal gives up half of
/// what the joint is rated to decelerate at and leaves the rest to the arm following it — which
/// makes `k` the acceleration limit itself. Against the FR3 constants used before, this brakes
/// joints 1 to 4 and 6 from a fifth to two thirds faster (`k` 12 → 15 on joint 1) and halves
/// the wrists, where the FR3's `a` of 17 rad/s² was above the FER's own 15 rad/s² rating on
/// joint 5: at 10 mrad before the stop, 0.19 rad/s against 0.16 on joint 1 and 0.17 against 0.33
/// on joint 5.
const FER_BRAKING: Braking = Braking {
    k: FER_DDQ_MAX,
    c: C,
};

impl Braking {
    /// `E_i(x)`, rad/s, never above `flat`; 0 at or past the stop and for NaN.
    fn speed(&self, i: usize, x: f64, flat: f64) -> f64 {
        if x > 0.0 {
            let c = self.c[i];
            flat.min((self.k[i] * x + c * c).sqrt() - c)
        } else {
            0.0
        }
    }
}

impl Envelope {
    /// The speed, rad/s, joint `i` may move at `distance` rad before its limit; 0 for NaN.
    fn speed(&self, i: usize, distance: f64) -> f64 {
        let rise = cmax(0.0, 2.0 * self.ddq_dec[i] * distance).sqrt() - self.dq_offset[i];
        (cmin(self.dq_max[i], cmax(0.0, rise)) - self.tolerance[i]).max(0.0)
    }

    /// The speeds joint `q` may move at toward its lower and toward its upper limit.
    pub(super) fn at(&self, q: &[f64; 7]) -> ([f64; 7], [f64; 7]) {
        (
            std::array::from_fn(|i| self.speed(i, q[i] - self.q_min[i])),
            std::array::from_fn(|i| self.speed(i, self.q_max[i] - q[i])),
        )
    }

    /// The saturation less the tolerance, rad/s.
    pub(super) fn flat(&self) -> [f64; 7] {
        std::array::from_fn(|i| self.dq_max[i] - self.tolerance[i])
    }
}

/// What the robot checks a joint's velocity against.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Cap {
    /// The same limit, rad/s, whatever the position.
    Flat([f64; 7]),
    /// A position-dependent envelope, which falls to zero at or a few mrad inside each limit.
    Envelope(&'static Envelope),
}

/// An arm's joint velocity limit: what the robot checks, and the braking profile the guard
/// approaches a position limit with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct VelocityLimit {
    cap: Cap,
    braking: &'static Braking,
}

impl VelocityLimit {
    /// The limit of the arm speaking `version`. The FER's envelope is unpublished: give it one
    /// here, as `Cap::Envelope`, when Franka publishes its coefficients.
    pub(super) fn of(version: FciVersion) -> Self {
        match version {
            FciVersion::V5 => VelocityLimit {
                cap: Cap::Flat(fer::MAX_JOINT_VELOCITY),
                braking: &FER_BRAKING,
            },
            FciVersion::V10 => VelocityLimit {
                cap: Cap::Envelope(&FR3),
                braking: &FR3_BRAKING,
            },
        }
    }

    /// Whether the limit narrows toward the position limits.
    pub(super) fn position_dependent(&self) -> bool {
        matches!(self.cap, Cap::Envelope(_))
    }

    /// The limit away from the position limits, rad/s.
    pub(super) fn flat(&self) -> [f64; 7] {
        match self.cap {
            Cap::Flat(v) => v,
            Cap::Envelope(e) => e.flat(),
        }
    }

    /// The speeds, rad/s and never negative, joint `q` may move at toward its lower and toward
    /// its upper limit.
    pub(super) fn at(&self, q: &[f64; 7]) -> ([f64; 7], [f64; 7]) {
        match self.cap {
            Cap::Flat(v) => (v, v),
            Cap::Envelope(e) => e.at(q),
        }
    }

    /// The braking profile `E_i(x)`, rad/s, at `x` rad before joint `i` must be at rest, capped
    /// at the joint's limit away from the position limits ([`Self::flat`], hoisted by callers).
    pub(super) fn braking(&self, i: usize, x: f64, flat: f64) -> f64 {
        self.braking.speed(i, x, flat)
    }
}

/// A flat limit with the FR3's braking profile: the rigs of the tests and of the replays' as-run
/// loop, which run with the position limits out of the way.
impl From<[f64; 7]> for VelocityLimit {
    fn from(flat: [f64; 7]) -> Self {
        VelocityLimit {
            cap: Cap::Flat(flat),
            braking: &FR3_BRAKING,
        }
    }
}
