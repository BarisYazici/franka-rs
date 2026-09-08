//! The candidate geometry of the contact estimator: the joint origins and axes at `q`, the
//! lines the candidates are sampled on, and the point Jacobian of a candidate.

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::{Frame, Model};
use nalgebra::{SMatrix, Vector3};

use super::contact::ContactOptions;

pub(super) type Jt = SMatrix<f64, 7, 3>;

/// Extent of each link's casting along its own joint axis, m from the joint origin, measured
/// on franka_description's visual meshes (the same for FER and FR3); only links 1, 3 and 5 are
/// sampled along it, the others lie along the skeleton segments.
pub const AXIS_EXTENT: [(f64, f64); 7] = [
    (-0.192, 0.055),
    (0.0, 0.0),
    (-0.121, 0.055),
    (0.0, 0.0),
    (-0.259, 0.052),
    (0.0, 0.0),
    (0.0, 0.0),
];

/// One candidate point and what it belongs to.
pub(super) struct Candidate {
    pub(super) link: usize,
    pub(super) distance: f64,
    pub(super) on_joint_axis: bool,
    pub(super) offset: f64,
    pub(super) point: Vector3<f64>,
}

/// The joint origins and axes, the flange and the end effector origins at `q`.
pub(super) struct Skeleton {
    pub(super) origin: [Vector3<f64>; 7],
    pub(super) axis: [Vector3<f64>; 7],
    flange: Vector3<f64>,
    ee: Vector3<f64>,
}

impl Skeleton {
    pub(super) fn at(model: &Model, q: &[f64; 7], f_t_ee: &[f64; 16]) -> Skeleton {
        let pose = |frame| model.pose_q(frame, q, f_t_ee, &IDENTITY_TRANSFORM);
        let translation = |p: &[f64; 16]| Vector3::new(p[12], p[13], p[14]);
        let mut origin = [Vector3::zeros(); 7];
        let mut axis = [Vector3::z(); 7];
        for (k, frame) in Frame::ALL[..7].iter().enumerate() {
            let p = pose(*frame);
            origin[k] = translation(&p);
            axis[k] = Vector3::new(p[8], p[9], p[10]);
        }
        Skeleton {
            origin,
            axis,
            flange: translation(&pose(Frame::Flange)),
            ee: translation(&pose(Frame::EndEffector)),
        }
    }

    /// `J_c^T` for a point on `link`: row `k` is `(z_k x (c - o_k))^T` for `k < link`, zero after.
    pub(super) fn jacobian_t(&self, link: usize, c: &Vector3<f64>) -> Jt {
        let mut jt = Jt::zeros();
        for k in 0..link {
            let column = self.axis[k].cross(&(c - self.origin[k]));
            jt.set_row(k, &column.transpose());
        }
        jt
    }

    /// The polylines candidates are sampled on, per link: the segment to the next origin, and
    /// for links with an [`AXIS_EXTENT`] the joint axis over it.
    pub(super) fn lines(&self, link: usize) -> Vec<(bool, Vec<Vector3<f64>>)> {
        let i = link - 1;
        let mut lines = Vec::with_capacity(2);
        let segment = if link < 7 {
            vec![self.origin[i], self.origin[i + 1]]
        } else {
            vec![self.origin[i], self.flange, self.ee]
        };
        lines.push((false, segment));
        let (lo, hi) = AXIS_EXTENT[i];
        if hi > lo {
            let o = self.origin[i];
            lines.push((true, vec![o + self.axis[i] * lo, o + self.axis[i] * hi]));
        }
        lines
    }

    pub(super) fn candidates(&self, options: &ContactOptions) -> Vec<Candidate> {
        let step = options.step.max(1e-4);
        let mut out = Vec::new();
        for link in 1..=7 {
            for (on_joint_axis, line) in self.lines(link) {
                let mut travelled = 0.0;
                for (a, b) in line.iter().zip(line.iter().skip(1)) {
                    let length = (b - a).norm();
                    let n = (length / step).ceil().max(1.0) as usize;
                    for j in 0..=n {
                        if j == 0 && travelled > 0.0 {
                            continue;
                        }
                        let t = j as f64 / n as f64;
                        let point = a + (b - a) * t;
                        let direction = if length > 1e-9 {
                            (b - a) / length
                        } else {
                            self.axis[link - 1]
                        };
                        let from_origin = point - self.origin[link - 1];
                        let distance = if on_joint_axis {
                            from_origin.dot(&self.axis[link - 1])
                        } else {
                            from_origin.norm()
                        };
                        let base = Candidate {
                            link,
                            distance,
                            on_joint_axis,
                            offset: 0.0,
                            point,
                        };
                        out.extend(lateral(base, &direction, options.lateral));
                        if length <= 1e-9 {
                            break;
                        }
                    }
                    travelled += length;
                }
            }
        }
        out
    }
}

/// `base` and, with a positive `lateral`, eight neighbours perpendicular to `direction`.
fn lateral(base: Candidate, direction: &Vector3<f64>, lateral: f64) -> Vec<Candidate> {
    if lateral <= 0.0 {
        return vec![base];
    }
    let helper = if direction.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = direction.cross(&helper).normalize();
    let v = direction.cross(&u);
    let mut out = Vec::with_capacity(9);
    for r in [lateral / 2.0, lateral] {
        for d in [u, -u, v, -v] {
            out.push(Candidate {
                link: base.link,
                distance: base.distance,
                on_joint_axis: base.on_joint_axis,
                offset: r,
                point: base.point + d * r,
            });
        }
    }
    out.push(base);
    out
}
