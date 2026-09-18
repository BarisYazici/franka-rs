//! The stateful target gate between the transport and `set_target`: pure, no I/O. A gate is
//! Cartesian ([`Guard::new`]) or joint-space ([`Guard::joints`]) and refuses the other kind.
//!
//! A Cartesian target is measured against two references: the previous accepted target, which
//! `max_step` keeps it near so the commanded stream stays continuous, and where the arm is,
//! which [`GuardOptions::max_lead`] keeps it near so the arm can never coast further than that
//! after the commander stops. [`crate::msg::TARGET_ANCHOR`] swaps the first reference for the
//! second on one message, which is how a commander whose stream has jumped gets back in; with
//! the lead limits off the swap would be unbounded, so the flag is then ignored.

use std::fmt;

use franka::robot::target_control::JOINT_LIMIT_INSET;

use crate::msg::{Kind, TargetMsg};

/// How far a quaternion's norm may be from one.
pub const UNIT_QUATERNION_TOLERANCE: f64 = 1e-3;

/// The arm's joint position limits (lower, upper), rad, as a joint-space gate checks them
/// ([`JOINT_LIMIT_INSET`] inside).
pub type JointLimits = ([f64; 7], [f64; 7]);

/// The gate's limits; [`Default`] is the spec's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuardOptions {
    /// Largest translation step from the previous accepted target, m. Default 0.05.
    pub max_step: f64,
    /// Largest rotation step from the previous accepted target, rad. Default 0.26.
    pub max_step_rotation: f64,
    /// Largest per-joint step from the previous accepted target, rad. Default 0.2.
    pub max_step_joint: f64,
    /// How far, as a norm in m, an accepted target may lead the reference [`Guard::check`] is
    /// given -- where the arm itself is, not the previous target. Default 0.05, `0` disables.
    ///
    /// `max_step` bounds a distance per message, so a commander faster than the generator
    /// walks the target ahead of the arm without limit and, when it stops sending, the arm
    /// coasts the whole accumulated lead instead of stopping where the commander stopped.
    /// This is the bound that makes that impossible: the target can never be further than
    /// `max_lead` from the arm, so the coast is that plus the brake distance.
    ///
    /// 0.05 equals `max_step`, which keeps the two checks coherent -- a commander whose steps
    /// the arm keeps up with is accepted by both -- and bounds the coast at about the leash
    /// plus 5 cm. Tighter is the backend's own 0.025 m leash, which the tracking error of a
    /// 100 Hz stream (7 to 11 mm) plus jitter would leave almost no margin under.
    pub max_lead: f64,
    /// The same on the orientation, rad, as the angle between the quaternions. Default 0.26,
    /// `0` disables.
    ///
    /// There is a rotational companion because the orientation coasts exactly as the position
    /// does -- the rotational budget is 0.5 rad/s, so a lead of 0.26 rad is another 0.9 s of
    /// motion after the commander stops, and at arm's length that is a hand's breadth at the
    /// fingers -- and because the backend's leash bounds rotation too. 0.26 equals
    /// `max_step_rotation` for the same coherence as above.
    pub max_lead_rotation: f64,
    /// Workspace box in the base frame, m, inclusive. Default x 0.2..0.8, y -0.5..0.5, z 0..0.8.
    pub workspace_min: [f64; 3],
    pub workspace_max: [f64; 3],
    /// Sustained target rate per client; the token bucket holds twice this. Default 250.
    pub rate_hz: f64,
}

impl Default for GuardOptions {
    fn default() -> Self {
        GuardOptions {
            max_step: 0.05,
            max_step_rotation: 0.26,
            max_step_joint: 0.2,
            max_lead: 0.05,
            max_lead_rotation: 0.26,
            workspace_min: [0.2, -0.5, 0.0],
            workspace_max: [0.8, 0.5, 0.8],
            rate_hz: 250.0,
        }
    }
}

/// An axis of the base frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];
}

impl fmt::Display for Axis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        })
    }
}

/// Why a target was refused; `Display` is the reply and log text.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reason {
    /// The kind byte is not a [`Kind`].
    Kind(u8),
    /// The target's kind is not the session's.
    WrongKind { got: Kind, mode: Kind },
    /// The sender is not the lease holder (or is 0, which is never one).
    NotHolder(u32),
    /// `seq` did not increase.
    Seq { last: u64, got: u64 },
    /// A component is NaN or infinite.
    NotFinite,
    /// The quaternion's norm is off by more than [`UNIT_QUATERNION_TOLERANCE`].
    NotUnit(f64),
    /// The translation step, m, exceeds `max_step`.
    Step(f64),
    /// The rotation step, rad, exceeds `max_step_rotation`.
    RotationStep(f64),
    /// The target leads the arm by more than `max_lead`, m.
    Lead(f64),
    /// Its orientation leads the arm's by more than `max_lead_rotation`, rad.
    LeadRotation(f64),
    /// The position lies outside the workspace box on this axis.
    Workspace(Axis),
    /// The step, rad, of this joint (1-based) exceeds `max_step_joint`.
    JointStep { joint: usize, rad: f64 },
    /// This joint (1-based) would be at `rad`, outside the inset limits.
    JointLimit { joint: usize, rad: f64 },
    /// The client's token bucket is empty.
    Rate,
}

impl Reason {
    /// Whether the target this refused came from the session's own commander: the session's
    /// kind, from the lease holder, in order, and refused only on its content or its rate.
    ///
    /// The target watchdog uses this as the sign of life it measures from, because a commander
    /// whose targets are all refused -- parked at the workspace wall, leading the arm, pushing
    /// against a hand -- is talking, not gone, and stopping its session tells it nothing and
    /// costs it the loop.
    pub fn from_the_commander(self) -> bool {
        !matches!(
            self,
            Reason::Kind(_) | Reason::WrongKind { .. } | Reason::NotHolder(_) | Reason::Seq { .. }
        )
    }
}

impl fmt::Display for Reason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reason::Kind(k) => write!(f, "unsupported kind {k}"),
            Reason::WrongKind { got, mode } => write!(f, "{got} target in {mode} mode"),
            Reason::NotHolder(id) => write!(f, "client {id} is not the holder"),
            Reason::Seq { last, got } => write!(f, "seq {got} not after {last}"),
            Reason::NotFinite => f.write_str("not finite"),
            Reason::NotUnit(norm) => write!(f, "quaternion norm {norm:.4} not unit"),
            Reason::Step(m) => write!(f, "step {m:.3} m too large"),
            Reason::RotationStep(rad) => write!(f, "rotation step {rad:.3} rad too large"),
            Reason::Lead(m) => write!(f, "lead {m:.3} m too large"),
            Reason::LeadRotation(rad) => write!(f, "lead rotation {rad:.3} rad too large"),
            Reason::Workspace(axis) => write!(f, "outside workspace on {axis}"),
            Reason::JointStep { joint, rad } => {
                write!(f, "joint {joint} step {rad:.3} rad too large")
            }
            Reason::JointLimit { joint, rad } => {
                write!(f, "joint {joint} at {rad:.3} rad outside limits")
            }
            Reason::Rate => f.write_str("rate limit"),
        }
    }
}

/// The outcome of [`Guard::check`].
#[must_use]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    Accept,
    Refuse(Reason),
}

/// Gates targets for one arm: the session's kind only, from the lease holder, in order,
/// finite, small steps, no further ahead of the arm than [`GuardOptions::max_lead`], within
/// bounds (the box and a unit quaternion, or the joint limits), under the rate.
#[derive(Debug, Clone)]
pub struct Guard {
    options: GuardOptions,
    /// `Some` in joints mode.
    joints: Option<JointLimits>,
    holder: u32,
    last_seq: u64,
    previous: [f64; 7],
    tokens: f64,
    refilled_ns: u64,
}

impl Guard {
    /// A Cartesian gate with no holder, stepping from `initial_target` (position xyz,
    /// quaternion xyzw).
    ///
    /// # Errors
    /// [`Reason::NotFinite`] or [`Reason::NotUnit`] when the initial target is neither: the
    /// step checks would compare against NaN and pass everything.
    pub fn new(options: GuardOptions, initial_target: [f64; 7]) -> Result<Self, Reason> {
        finite_and_unit(&initial_target)?;
        Ok(Guard::with_kind(options, None, initial_target))
    }

    /// A joint-space gate with no holder under the arm's `limits`, stepping from `initial_q`,
    /// rad.
    ///
    /// # Errors
    /// [`Reason::NotFinite`] when `initial_q` is not.
    pub fn joints(
        options: GuardOptions,
        limits: JointLimits,
        initial_q: [f64; 7],
    ) -> Result<Self, Reason> {
        if initial_q.iter().any(|x| !x.is_finite()) {
            return Err(Reason::NotFinite);
        }
        Ok(Guard::with_kind(options, Some(limits), initial_q))
    }

    fn with_kind(options: GuardOptions, joints: Option<JointLimits>, previous: [f64; 7]) -> Self {
        Guard {
            options,
            joints,
            holder: 0,
            last_seq: 0,
            previous,
            tokens: 2.0 * options.rate_hz,
            refilled_ns: 0,
        }
    }

    /// Whether both lead limits are on, which is what bounds an anchored target.
    pub fn lead_bounded(&self) -> bool {
        self.options.max_lead > 0.0 && self.options.max_lead_rotation > 0.0
    }

    /// The kind this gate accepts.
    pub fn kind(&self) -> Kind {
        if self.joints.is_some() {
            Kind::Joints
        } else {
            Kind::Cartesian
        }
    }

    /// Makes `client_id` the only accepted sender and restarts its sequence and bucket.
    pub fn set_holder(&mut self, client_id: u32, now_ns: u64) {
        self.holder = client_id;
        self.last_seq = 0;
        self.tokens = 2.0 * self.options.rate_hz;
        self.refilled_ns = now_ns;
    }

    /// The accepted sender, 0 if none.
    pub fn holder(&self) -> u32 {
        self.holder
    }

    /// The last accepted target (the initial one until something is accepted).
    pub fn previous(&self) -> [f64; 7] {
        self.previous
    }

    /// The last accepted `seq`, 0 if none.
    pub fn last_seq(&self) -> u64 {
        self.last_seq
    }

    /// Checks `msg` at `now_ns` against `reference`, where the arm is (position xyz,
    /// quaternion xyzw), and on `Accept` records it as the previous target.
    ///
    /// `reference` is the caller's: the gate holds no robot and no model. `None` -- a joints
    /// session, or a Cartesian one before the arm's pose is known -- skips the lead checks and
    /// makes [`crate::msg::TARGET_ANCHOR`] a no-op, since an anchor has nothing to anchor on.
    pub fn check(&mut self, msg: &TargetMsg, now_ns: u64, reference: Option<&[f64; 7]>) -> Verdict {
        match self.judge(msg, now_ns, reference) {
            Ok(()) => {
                self.tokens -= 1.0;
                self.last_seq = msg.seq.get();
                self.previous = msg.data();
                Verdict::Accept
            }
            Err(reason) => Verdict::Refuse(reason),
        }
    }

    fn judge(
        &mut self,
        msg: &TargetMsg,
        now_ns: u64,
        reference: Option<&[f64; 7]>,
    ) -> Result<(), Reason> {
        let got = msg.kind().ok_or(Reason::Kind(msg.kind))?;
        let mode = self.kind();
        if got != mode {
            return Err(Reason::WrongKind { got, mode });
        }
        let client = msg.client_id.get();
        if client == 0 || client != self.holder {
            return Err(Reason::NotHolder(client));
        }
        let seq = msg.seq.get();
        if seq <= self.last_seq {
            return Err(Reason::Seq {
                last: self.last_seq,
                got: seq,
            });
        }
        let data = msg.data();
        match self.joints {
            Some(limits) => self.judge_joints(&data, &limits)?,
            None => self.judge_cartesian(&data, reference, msg.anchor())?,
        }
        self.refill(now_ns);
        if self.tokens < 1.0 {
            return Err(Reason::Rate);
        }
        Ok(())
    }

    /// The Cartesian rules. An `anchored` target with a `reference` steps from the arm instead
    /// of from the previous accepted target, which is the whole of the anchor: it lifts no
    /// limit, so an anchored target is bounded by `min(max_step, max_lead)` of where the arm
    /// is -- tighter than the ordinary path, whose previous target may itself lead the arm, and
    /// so not worth abusing. What it dissolves is the lockout: a refusal leaves `previous`
    /// behind, so a stream that jumps (a re-latched clutch, a shifted tracking frame, an arm a
    /// hand has pushed off the target it holds) is otherwise refused for ever.
    ///
    /// The anchor is honoured only while both lead limits are on ([`Guard::lead_bounded`]).
    /// With them off, the second of those two bounds is gone and the anchor would be the one
    /// way to command a jump of any size: `max_step` from a measured pose that may be anywhere.
    fn judge_cartesian(
        &self,
        data: &[f64; 7],
        reference: Option<&[f64; 7]>,
        anchored: bool,
    ) -> Result<(), Reason> {
        finite_and_unit(data)?;
        let from = match reference {
            Some(reference) if anchored && self.lead_bounded() => reference,
            _ => &self.previous,
        };
        let step = distance(data, from);
        // `!is_finite()` refuses a NaN that a `>` would let through.
        if !step.is_finite() || step > self.options.max_step {
            return Err(Reason::Step(step));
        }
        let angle = angle_between(&from[3..], &data[3..]);
        if !angle.is_finite() || angle > self.options.max_step_rotation {
            return Err(Reason::RotationStep(angle));
        }
        if let Some(reference) = reference {
            let lead = distance(data, reference);
            if self.options.max_lead > 0.0 && (!lead.is_finite() || lead > self.options.max_lead) {
                return Err(Reason::Lead(lead));
            }
            let lead = angle_between(&reference[3..], &data[3..]);
            if self.options.max_lead_rotation > 0.0
                && (!lead.is_finite() || lead > self.options.max_lead_rotation)
            {
                return Err(Reason::LeadRotation(lead));
            }
        }
        for (i, axis) in Axis::ALL.into_iter().enumerate() {
            let (lo, hi) = (self.options.workspace_min[i], self.options.workspace_max[i]);
            if data[i] < lo || data[i] > hi {
                return Err(Reason::Workspace(axis));
            }
        }
        Ok(())
    }

    fn judge_joints(&self, q: &[f64; 7], (lower, upper): &JointLimits) -> Result<(), Reason> {
        if q.iter().any(|x| !x.is_finite()) {
            return Err(Reason::NotFinite);
        }
        for (i, value) in q.iter().enumerate() {
            let joint = i + 1;
            let step = (value - self.previous[i]).abs();
            if step > self.options.max_step_joint {
                return Err(Reason::JointStep { joint, rad: step });
            }
            let lower = lower[i] + JOINT_LIMIT_INSET;
            let upper = upper[i] - JOINT_LIMIT_INSET;
            if !(lower..=upper).contains(value) {
                return Err(Reason::JointLimit { joint, rad: *value });
            }
        }
        Ok(())
    }

    fn refill(&mut self, now_ns: u64) {
        let elapsed = now_ns.saturating_sub(self.refilled_ns) as f64 * 1e-9;
        let depth = 2.0 * self.options.rate_hz;
        self.tokens = (self.tokens + elapsed * self.options.rate_hz).min(depth);
        self.refilled_ns = now_ns.max(self.refilled_ns);
    }
}

/// Every component finite and the quaternion within [`UNIT_QUATERNION_TOLERANCE`] of unit.
fn finite_and_unit(target: &[f64; 7]) -> Result<(), Reason> {
    if target.iter().any(|x| !x.is_finite()) {
        return Err(Reason::NotFinite);
    }
    let quaternion_norm = norm(&target[3..]);
    if (quaternion_norm - 1.0).abs() > UNIT_QUATERNION_TOLERANCE {
        return Err(Reason::NotUnit(quaternion_norm));
    }
    Ok(())
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// The distance, m, between the positions of two targets.
fn distance(a: &[f64; 7], b: &[f64; 7]) -> f64 {
    norm(&[a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

/// The angle, rad, between two near-unit quaternions, sign-insensitive; NaN when either has
/// no direction (`f64::min` would turn a NaN cosine into an angle of zero).
fn angle_between(a: &[f64], b: &[f64]) -> f64 {
    let dot: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum();
    let cos_half = dot.abs() / (norm(a) * norm(b));
    if !cos_half.is_finite() {
        return f64::NAN;
    }
    2.0 * cos_half.min(1.0).acos()
}

#[cfg(test)]
mod tests;
