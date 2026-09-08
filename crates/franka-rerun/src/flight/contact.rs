//! Where on the arm an external force acts, from the external joint torques.
//!
//! A force `F` at a point `c` on link `i` (the body that joint `i` moves) loads joints `1..=i`
//! with `tau_k = z_k . ((c - o_k) x F)` -- `o_k` and `z_k` being the origin and axis of joint
//! `k`'s frame, from [`Model::pose_q`] -- and leaves joints `i+1..=7` alone; in matrix form
//! `tau = J_c^T F` with the point Jacobian `J_c[:, k] = z_k x (c - o_k)` for `k <= i` and zero
//! beyond. [`estimate`] tries a grid of candidate points along the arm, solves the 7x3 least
//! squares for `F` at each, and reports the candidate with the smallest residual together with
//! the best residual of every other link, so an ambiguous fit shows as an alternative close to
//! the winner. Two ambiguities are inherent: a force is a line vector, so sliding `c` along
//! `F`'s line of action changes nothing (a push that goes through a joint axis therefore ties
//! the links on both sides of that joint), and a force along a link's own direction cannot be
//! located along that link.
//!
//! Candidates on a Franka arm: for every link `i` the skeleton segment from `o_i` to
//! `o_{i+1}` (link 7 runs to the flange and on to the end effector), every
//! [`ContactOptions::step`], and for links 1, 3 and 5 -- whose castings run along their own
//! joint axis rather than towards the next origin (joint 2 sits on joint 1's origin, joint 6
//! on joint 5's) -- the joint axis over the casting's extent as measured on franka_description's
//! visual meshes ([`AXIS_EXTENT`]). Candidates stay on those lines by default; with
//! [`ContactOptions::lateral`] each also gets eight neighbours displaced perpendicular to the
//! line. There is no estimate while every torque is below [`ContactOptions::noise_floor`];
//! the fit is damped ([`ContactOptions::damping`]) so that a candidate next to a joint origin,
//! where the lever arms vanish, cannot explain anything with an enormous force. What `i`
//! loaded joints can tell: three force components and a position along a line need four, so
//! on links 1 to 3 every point of the line fits and the estimate shows that as a
//! [`ContactEstimate::span`] covering the link; telling two lines apart needs a fifth, so a
//! contact on the forearm axis reads as link 4 (the chord from the elbow) and link 5 (the
//! axis) fitting alike, a few centimetres apart, whichever the noise favours winning and the
//! other showing in [`ContactEstimate::next_best`].
//! [`super::Tare`] removes the slowly varying part of `tau_ext_hat_filtered` -- the model
//! error that reads as a tenth of a newton-metre or more on some joints with nothing touching
//! the arm.

use std::fmt;

use franka::Model;
use nalgebra::{Matrix3, SVector, Vector3};

pub use super::skeleton::AXIS_EXTENT;
use super::skeleton::{Jt, Skeleton};

type Tau = SVector<f64, 7>;

/// How the estimator samples and filters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactOptions {
    /// A joint torque below this is noise and counts as zero, Nm.
    pub noise_floor: f64,
    /// Spacing of the candidate points along the arm, m.
    pub step: f64,
    /// Largest lateral displacement of a candidate from the arm's lines, m; 0 keeps to them.
    pub lateral: f64,
    /// Tikhonov weight on the force, m: `|tau - J^T F|^2 + damping^2 |F|^2` is minimised, so
    /// a force costs as much as it would on a lever this long.
    pub damping: f64,
    /// Zero every torque below the floor before fitting, rather than only gating on them.
    /// A joint next to the contact sees a small torque that is still information, so this is
    /// off by default.
    pub zero_below_floor: bool,
    /// Residuals within this of the best count as fitting equally well, Nm; along the
    /// winner's line they define the [`ContactEstimate::span`].
    pub tolerance: f64,
}

impl Default for ContactOptions {
    fn default() -> Self {
        ContactOptions {
            noise_floor: 1.0,
            step: 0.01,
            lateral: 0.0,
            damping: 0.002,
            zero_below_floor: false,
            tolerance: 0.3,
        }
    }
}

/// The best fit: which link, where on it, and the force.
#[derive(Debug, Clone, PartialEq)]
pub struct ContactEstimate {
    /// The link, 1-based: the body joint `link` moves.
    pub link: usize,
    /// Distance of the point from joint `link`'s origin, m; signed along the axis when
    /// `on_joint_axis` (negative towards the previous joint).
    pub distance: f64,
    /// Whether the point lies on joint `link`'s own axis rather than on the segment towards the
    /// next joint.
    pub on_joint_axis: bool,
    /// Lateral displacement from that line, m (0 without [`ContactOptions::lateral`]).
    pub offset: f64,
    /// The distances along the same line between which candidates fit within
    /// [`ContactOptions::tolerance`] of the best: how well the point is located. A span the
    /// length of the link means the location along it is not observable.
    pub span: [f64; 2],
    /// The point in the base frame, m.
    pub point: [f64; 3],
    /// The force in the base frame, N.
    pub force: [f64; 3],
    /// `|tau - J_c^T F|` at the winner, Nm.
    pub residual: f64,
    /// The smallest residual on each link (index `link - 1`), the winner's included.
    pub link_residuals: [f64; 7],
}

impl ContactEstimate {
    /// `|F|`, N.
    pub fn magnitude(&self) -> f64 {
        crate::norm(&self.force)
    }

    /// The best link other than the winner and its residual: the ambiguity.
    pub fn next_best(&self) -> Option<(usize, f64)> {
        self.link_residuals
            .iter()
            .enumerate()
            .filter(|(i, r)| i + 1 != self.link && r.is_finite())
            .map(|(i, r)| (i + 1, *r))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
}

impl fmt::Display for ContactEstimate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let along = if self.on_joint_axis {
            "along its axis"
        } else {
            "towards the next joint"
        };
        write!(
            f,
            "contact on link {}, {:.2} m from joint {} {along} ({:.2}..{:.2} m fit as well), \
             {:.1} N; residual {:.2} Nm",
            self.link,
            self.distance,
            self.link,
            self.span[0],
            self.span[1],
            self.magnitude(),
            self.residual
        )?;
        if let Some((link, residual)) = self.next_best() {
            write!(f, "; next best link {link} residual {residual:.2} Nm")?;
        }
        Ok(())
    }
}

/// Damped least-squares `F` for `tau = J_c^T F` and the residual norm.
fn fit(jt: &Jt, tau: &Tau, damping: f64) -> (Vector3<f64>, f64) {
    let j = jt.transpose();
    let normal = j * jt + Matrix3::identity() * damping * damping;
    let force = normal
        .cholesky()
        .map(|c| c.solve(&(j * tau)))
        .unwrap_or_else(Vector3::zeros);
    let residual = (tau - jt * force).norm();
    (force, residual)
}

/// The contact that best explains `tau_ext` (`tau_ext_hat_filtered`, less any baseline) at
/// `q`; `None` when every torque is below the noise floor.
pub fn estimate(
    model: &Model,
    q: &[f64; 7],
    tau_ext: &[f64; 7],
    f_t_ee: &[f64; 16],
    options: &ContactOptions,
) -> Option<ContactEstimate> {
    let floor = options.noise_floor.max(0.0);
    if tau_ext.iter().all(|t| t.abs() < floor) {
        return None;
    }
    let tau = Tau::from_iterator(tau_ext.iter().map(|&t| {
        if options.zero_below_floor && t.abs() < floor {
            0.0
        } else {
            t
        }
    }));
    let skeleton = Skeleton::at(model, q, f_t_ee);
    let candidates = skeleton.candidates(options);
    let fits: Vec<(Vector3<f64>, f64)> = candidates
        .iter()
        .map(|c| {
            fit(
                &skeleton.jacobian_t(c.link, &c.point),
                &tau,
                options.damping,
            )
        })
        .collect();
    // The smallest residual wins. Exact ties (a force line through a joint axis, or displaced
    // neighbours that fit as well) go to the lower link -- a distal joint reading no torque is
    // explained structurally by the lower link and only by a coincidence, a point right on
    // that joint's axis, by the higher one -- then to the smaller offset.
    let winner = (0..candidates.len()).min_by(|&a, &b| {
        let (ra, rb) = (fits[a].1, fits[b].1);
        if (ra - rb).abs() <= 1e-12 {
            (candidates[a].link, candidates[a].offset)
                .partial_cmp(&(candidates[b].link, candidates[b].offset))
                .expect("finite offsets")
        } else {
            ra.total_cmp(&rb)
        }
    })?;
    let (best, residual) = (&candidates[winner], fits[winner].1);
    let mut link_residuals = [f64::INFINITY; 7];
    let mut span = [best.distance; 2];
    for (candidate, (_, r)) in candidates.iter().zip(&fits) {
        let slot = &mut link_residuals[candidate.link - 1];
        *slot = slot.min(*r);
        let same_line = candidate.link == best.link
            && candidate.on_joint_axis == best.on_joint_axis
            && candidate.offset == 0.0;
        if same_line && *r <= residual + options.tolerance {
            span = [
                span[0].min(candidate.distance),
                span[1].max(candidate.distance),
            ];
        }
    }
    Some(ContactEstimate {
        link: best.link,
        distance: best.distance,
        on_joint_axis: best.on_joint_axis,
        offset: best.offset,
        span,
        point: best.point.into(),
        force: fits[winner].0.into(),
        residual,
        link_residuals,
    })
}

/// The torques a force `force` at `point` on `link` produces at `q`: the forward model, for
/// tests and for checking an estimate against the measurement.
pub fn torques(
    model: &Model,
    q: &[f64; 7],
    f_t_ee: &[f64; 16],
    link: usize,
    point: &[f64; 3],
    force: &[f64; 3],
) -> [f64; 7] {
    let skeleton = Skeleton::at(model, q, f_t_ee);
    let jt = skeleton.jacobian_t(link.clamp(1, 7), &Vector3::from(*point));
    (jt * Vector3::from(*force)).into()
}

/// A point on `link` for building test cases, in the base frame: `distance` m from joint
/// `link`'s origin towards the next joint's (for link 7: towards the flange and on to the end
/// effector), or with `on_joint_axis` the signed distance along joint `link`'s own axis.
pub fn point_on_link(
    model: &Model,
    q: &[f64; 7],
    f_t_ee: &[f64; 16],
    link: usize,
    distance: f64,
    on_joint_axis: bool,
) -> [f64; 3] {
    let skeleton = Skeleton::at(model, q, f_t_ee);
    let i = link.clamp(1, 7) - 1;
    if on_joint_axis {
        return (skeleton.origin[i] + skeleton.axis[i] * distance).into();
    }
    let (_, line) = skeleton.lines(i + 1).swap_remove(0);
    let mut left = distance;
    for (a, b) in line.iter().zip(line.iter().skip(1)) {
        let length = (b - a).norm();
        if left <= length || length <= 1e-9 {
            let t = if length > 1e-9 { left / length } else { 0.0 };
            return (a + (b - a) * t.min(1.0)).into();
        }
        left -= length;
    }
    (*line.last().expect("a line has points")).into()
}
