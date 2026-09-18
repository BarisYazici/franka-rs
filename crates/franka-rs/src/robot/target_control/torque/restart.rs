//! The generator's restart after a Cartesian goal fell short of its step, per block
//! (translation, rotation). Held at a position limit, the velocity and acceleration into the wall
//! the pinned joints push against are cut to what went out along it, so the rest slides on and
//! nothing winds up against it, and that rest is cut again to the speed the goal's step carried,
//! so the generator does not run ahead of a goal a wall holds back; on a velocity pin each block
//! is cut to the fraction of its step that went out; otherwise the velocity is cut by the
//! backstop's scale. [`ahead`] says where the generator restarts, [`landed`] when a stop held at
//! a wall has got as near the target as it will.

use nalgebra::Vector3;

use super::super::cartesian::Placement;
use super::super::ik::stall::{Stall, Wall, STALL_QUIET_CYCLES};
use super::super::rotation::{exp, from_quaternion, log};
use super::super::runner::{braking, Step, REST_VELOCITY};
use crate::rate_limiting::DELTA_T;

/// A deficit or a step along a wall shorter than this, m or rad at the generator, is rounding.
const NEGLIGIBLE: f64 = 1e-9;
/// A generator step shorter than this, m or rad, moves slower than at rest: a block planning less
/// is left as it is.
const REST_STEP: f64 = REST_VELOCITY * DELTA_T;

/// The walls a goal is held against, per block (translation, rotation), each with the cycles
/// since it was last pushed on.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(in super::super) struct Walls {
    pub blocks: [Option<(Wall, u32)>; 2],
}

impl Walls {
    /// Each block's wall follows its own pushes and goes `STALL_QUIET_CYCLES` after the last,
    /// which bridges a pin the exact solve lets go for a cycle; whether the goal is held: stalled,
    /// or against a wall.
    pub(in super::super) fn update(&mut self, stall: &Stall) -> bool {
        for (block, withheld) in self.blocks.iter_mut().zip(stall.withheld) {
            *block = match (withheld, *block) {
                (Some(wall), _) => Some((wall, 0)),
                (None, Some((wall, off))) if off < STALL_QUIET_CYCLES => Some((wall, off + 1)),
                _ => None,
            };
        }
        stall.stalled || self.blocks.iter().any(Option::is_some)
    }
}

/// What the goal fell short against, which decides how the generator restarts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in super::super) enum Pressure {
    /// Nothing: a backstop cut.
    None,
    /// A velocity pin, not held at a position limit.
    Velocity,
    /// Held at a position limit, against the walls; per block how far its pins' goals rest on
    /// their margins, 0 to 1, where nothing more goes out along the wall.
    Position { rest: [f64; 2] },
}

/// The generator's restart after a step that fell short: its end-of-cycle velocity and
/// acceleration, and per block the fraction of its step that went out, the backstop's `cut`
/// included. `anchor` is where the step started, `from` the last goal's pose and
/// `out` this one's: what went out is the goal's own step, so the IK's lag behind the generator
/// is not taken for a shortfall.
pub(in super::super) fn restart(
    step: &Step<6, 7>,
    (anchor, from): (&Placement, &Placement),
    out: &Placement,
    cut: f64,
    pressure: Pressure,
    walls: &Walls,
) -> ([f64; 6], [f64; 6], [f64; 2]) {
    let (p, v, a) = (step.position, step.velocity, step.acceleration);
    let turn = log(&(out.1 * from.1.transpose()));
    let (mut velocity, mut acceleration, mut fractions) = ([0.0; 6], [0.0; 6], [1.0; 2]);
    for k in [0, 3] {
        let (plan, went): (Vector3<f64>, Vector3<f64>) = if k == 0 {
            (
                Vector3::from_fn(|i, _| p[i] - anchor.0[i]),
                Vector3::from_fn(|i, _| out.0[i] - from.0[i]),
            )
        } else {
            (Vector3::new(p[3], p[4], p[5]), Vector3::from(turn))
        };
        let block = |x: &[f64; 6]| Vector3::new(x[k], x[k + 1], x[k + 2]);
        let wall = match pressure {
            Pressure::Position { rest } => {
                walls.blocks[k / 3].map(|(w, _)| (w.normal, rest[k / 3]))
            }
            _ => None,
        };
        let restarted = restart_block(block(&v), block(&a), plan, went, cut, pressure, wall);
        velocity[k..k + 3].copy_from_slice(restarted.0.as_slice());
        acceleration[k..k + 3].copy_from_slice(restarted.1.as_slice());
        fractions[k / 3] = cut * restarted.2;
    }
    (velocity, acceleration, fractions)
}

/// Where the generator restarts: its anchor advanced by the goal's step, the IK's lag behind it so
/// kept, but per block never further than its own planned step, so a goal closing a lag or moving
/// off the plan does not drag the desired along; held against a wall, less the lead into it.
pub(in super::super) fn ahead(
    step: &Step<6, 7>,
    (anchor, from): (&Placement, &Placement),
    out: &Placement,
    walls: Option<&Walls>,
) -> Placement {
    let p = &step.position;
    let advance = |went: Vector3<f64>, plan: Vector3<f64>| {
        let (w, l) = (went.norm(), plan.norm());
        if w > l {
            went * (l / w)
        } else {
            went
        }
    };
    let lead = |k: usize, lead: Vector3<f64>| match walls.and_then(|w| w.blocks[k]) {
        Some((w, _)) => lead - w.normal * lead.dot(&w.normal).max(0.0),
        None => lead,
    };
    let went = Vector3::from_fn(|i, _| out.0[i] - from.0[i]);
    let plan = Vector3::from_fn(|i, _| p[i] - anchor.0[i]);
    let moved = advance(went, plan);
    let position = lead(
        0,
        Vector3::from_fn(|i, _| anchor.0[i] + moved[i] - out.0[i]),
    );
    let turn = advance(
        Vector3::from(log(&(out.1 * from.1.transpose()))),
        Vector3::new(p[3], p[4], p[5]),
    );
    let desired = exp(&turn.into()) * anchor.1;
    let rotation = lead(1, Vector3::from(log(&(desired * out.1.transpose()))));
    (
        std::array::from_fn(|i| out.0[i] + position[i]),
        exp(&rotation.into()) * out.1,
    )
}

/// Cycles a goal must have come no [`Settle::tolerance`](super::super::Settle::tolerance) closer
/// to its target in before a stop lands on a wall: `(12 · tolerance / jerk)^(1/3)`, the time a
/// generator takes from rest to cover twice a tolerance (101 cycles at the Cartesian defaults), so
/// a stop that has only just started, whose generator is still at its first cycles of jerk, is
/// never taken for a goal a wall holds.
pub(in super::super) fn landing_dwell(jerks: &[f64; 6], tolerance: f64) -> u32 {
    let cycles = |jerk: f64| ((12.0 * tolerance / jerk).cbrt() / DELTA_T).ceil() as u32;
    jerks.iter().map(|jerk| cycles(*jerk)).max().unwrap_or(0)
}

/// What is left of `target` at the goal's pose `out`, per generator axis: the translation, m, then
/// the rotation, rad.
pub(in super::super) fn left(target: &[f64; 7], out: &Placement) -> [f64; 6] {
    let rotation = from_quaternion(&[target[3], target[4], target[5], target[6]]);
    let turn = log(&(rotation * out.1.transpose()));
    std::array::from_fn(|i| {
        if i < 3 {
            target[i] - out.0[i]
        } else {
            turn[i - 3]
        }
    })
}

/// A block whose restarted speed, m/s or rad/s, is over this is still landing, not landed: the
/// ordinary path finishes at [`REST_VELOCITY`], and a wall may excuse the last of a slide but not
/// the speed it is taken at. Measured on the plant stops: 1e-4 to 3e-4 held against a wall.
const LANDING_SPEED: f64 = 10.0 * REST_VELOCITY;
/// The share of a tolerance of what is left that a wall's normal must hold back before the block
/// is judged against it rather than against the tolerance: a wall nearly orthogonal to the
/// remainder holds none of it, and without a floor decides on rounding.
const BEYOND_FLOOR: f64 = 1e-3;

/// Whether a stop toward `target` (the slot's values) lands on the goal's pose `out`, restarted at
/// `speed`, held at the `walls`: per block, within
/// [`Settle::tolerance`](super::super::Settle::tolerance) of the target on every generator axis,
/// or slow and beyond a wall, and at least one block slow at a wall at all. Judged on the target
/// rather than on the wall's own axes: the synchronised generator would otherwise stretch the last
/// of a slide along a wall to the creep into it, and a wall tilted off the axes would never land.
/// The caller adds the dwell: a goal that has come no tolerance closer for [`landing_dwell`]
/// cycles.
pub(in super::super) fn landed(
    target: &[f64; 7],
    out: &Placement,
    walls: &Walls,
    tolerance: f64,
    speed: &[f64; 6],
) -> bool {
    let left = left(target, out);
    let mut held = false;
    for (k, wall) in walls.blocks.iter().enumerate() {
        let d = Vector3::from_fn(|i, _| left[3 * k + i]);
        let slow = Vector3::from_fn(|i, _| speed[3 * k + i]).norm() <= LANDING_SPEED;
        let beyond = wall.is_some_and(|(w, _)| slow && w.normal.dot(&d) > BEYOND_FLOOR * tolerance);
        held |= wall.is_some() && slow;
        if !beyond && (0..3).any(|i| d[i].abs() >= tolerance) {
            return false;
        }
    }
    held
}

/// One block of [`restart`]: `plan` the generator's step, `went` the goal's, `wall` the
/// direction held against and how far its pins rest on their margins.
fn restart_block(
    v: Vector3<f64>,
    a: Vector3<f64>,
    plan: Vector3<f64>,
    went: Vector3<f64>,
    cut: f64,
    pressure: Pressure,
    wall: Option<(Vector3<f64>, f64)>,
) -> (Vector3<f64>, Vector3<f64>, f64) {
    let planned = plan * cut;
    let moving = plan.norm() > REST_STEP && cut > 0.0;
    let phi = if moving {
        (went.dot(&planned) / planned.norm_squared()).clamp(0.0, 1.0)
    } else {
        1.0
    };
    let scaled = v * cut;
    let kept = |v: Vector3<f64>| Vector3::from(braking(&v.into(), &a.into()));
    let withheld = (planned - went).norm() >= NEGLIGIBLE;
    match pressure {
        Pressure::Position { .. } => {
            let Some((n, rest)) = wall.filter(|_| withheld) else {
                // No wall in this block, and part of the plan did not go out: at most what did.
                let v = if withheld {
                    went_out(scaled, a, went)
                } else {
                    scaled
                };
                return if cut < 1.0 {
                    (v, kept(v), phi)
                } else {
                    (v, a, phi)
                };
            };
            // Into the wall, only what went out along it: the envelope's fraction while a joint
            // still brakes toward it, ramped to nothing as it comes to rest on its margin.
            // One-sided, so a reversal and the motion along the wall are left alone.
            let (p, w) = (planned.dot(&n), went.dot(&n));
            let kept_along = if p > NEGLIGIBLE {
                (w / p).clamp(0.0, 1.0) * (1.0 - rest)
            } else {
                0.0
            };
            let into = |x: Vector3<f64>| n * ((1.0 - kept_along) * x.dot(&n).max(0.0));
            let (v, a) = (scaled - into(scaled), a - into(a));
            // What is left of the plan went out only in part: the velocity is cut to that part,
            // so the generator does not run ahead of a goal the wall holds back.
            let v = went_out(v, a, went);
            // What is left of the acceleration drives along or away from the wall; only a cut
            // brakes it as the cap does.
            let a = if cut < 1.0 {
                braking(&v.into(), &a.into()).into()
            } else {
                a
            };
            (v, a, phi)
        }
        Pressure::Velocity => (scaled * phi, kept(scaled * phi), phi),
        Pressure::None => (scaled, kept(scaled), phi),
    }
}

/// The velocity `v` of a block that carried `went` out, restarted on that step: no faster along
/// it than the step's own speed and across it than the goal stood still, but for one cycle of the
/// acceleration `a` either way (a step is a cycle's mean, the velocity its end). A goal that made
/// only part of the plan, or none of it, so takes the generator's velocity down with it; a
/// reversal is left alone.
fn went_out(v: Vector3<f64>, a: Vector3<f64>, went: Vector3<f64>) -> Vector3<f64> {
    let lead = a.norm() * DELTA_T;
    let capped = |x: Vector3<f64>, most: f64| {
        let n = x.norm();
        if n > most {
            x * (most / n)
        } else {
            x
        }
    };
    let length = went.norm();
    if length <= NEGLIGIBLE {
        return capped(v, lead);
    }
    let unit = went / length;
    let ahead = v.dot(&unit);
    let along = ahead.min(length / DELTA_T + lead);
    unit * along + capped(v - unit * ahead, lead)
}
