//! The stall flag: how hard the residual pushes the pinned joints into their bounds, a hysteresis
//! on the position pins' pressure that says the goal is held at a position limit, and the motion
//! those pins withhold that the free joints cannot make up.

use nalgebra::{Matrix6, Vector3};

use super::active_set::{Jacobian, Task};

/// Position pressure, weighted task units (m) per cycle, above which the flag is set at once.
const STALL_ON: f64 = 2e-5;
/// Quiet cycles that clear the flag, and the cycles they are counted over; also how long a wall
/// outlives its last push.
pub(in super::super) const STALL_QUIET_CYCLES: u32 = 20;
/// Cycles the quiet ones are counted over. A run in a row would never clear: a pin resting on its
/// margin is pushed on now and then, however lightly, and one such cycle restarted the run.
const STALL_QUIET_WINDOW: u32 = 40;
/// The share of what the pins withhold, weighted, that a block's remainder must exceed to hold a
/// wall, and of that wall a pin's remainder along it to be one of its pins.
const WALL_SHARE: f64 = 0.1;
/// Weighted m: withheld motion, or a pin's push, under this is rounding, far under the cycle of
/// jerk (2e-9 m at 20 m/s³) a wall leaves the generator at rest.
const WALL_FLOOR: f64 = 1e-12;

/// A block's wall: the direction of the motion withheld, and the pins (∓1) withholding it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in super::super) struct Wall {
    pub normal: Vector3<f64>,
    pub joints: [i8; 7],
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(in super::super) struct Stall {
    /// The largest pressure into a position bound this cycle.
    pub position: f64,
    /// The largest pressure into a velocity bound this cycle.
    pub velocity: f64,
    pub stalled: bool,
    /// Per block (translation, rotation, base frame), the wall of the motion the position pins
    /// withhold and the free joints cannot make up; `None` when there is none.
    pub withheld: [Option<Wall>; 2],
    /// One bit per cycle, newest last: whether it was quiet.
    quiet: u64,
}

impl Stall {
    /// Updates the flag on this cycle's `pinned` (0 free, ±1 position, ±2 velocity), `j` the
    /// weighted Jacobian, `r` the weighted residual at the goal and `damping` the solver's λ. A
    /// pin's pressure is the residual along its unit column, `side · J̃_iᵀ r / ‖J̃_i‖`, counted only
    /// into its bound and taken over the whole task or its translation rows alone, whichever
    /// pushes harder: the same units whatever the tool and the rotation weight, and a flag that
    /// does not drop where position first has the orientation cancel the translation's push. What a pin pushing
    /// withholds, `ĵ_i ĵ_iᵀ r`, is left to the free joints' damped least squares; the remainders
    /// `λ² (J̃_F J̃_Fᵀ + λ² I)⁻¹ ĵ_i ĵ_iᵀ r` sum to the wall. Any push counts, however small: at rest
    /// on a wall the generator plans a cycle of jerk into it, which leaves a residual under the
    /// solver's tolerance (a floor there lets a parked fold creep at 3.6 mm/s).
    pub(in super::super) fn update(
        &mut self,
        j: &Jacobian,
        r: &Task,
        pinned: &[i8; 7],
        damping: f64,
    ) {
        let (mut position, mut velocity) = (0.0f64, 0.0f64);
        let mut pushes = [Task::zeros(); 7];
        let mut free = Matrix6::identity() * (damping * damping);
        for (i, &pin) in pinned.iter().enumerate() {
            let column = j.column(i);
            let side = f64::from(pin.signum());
            let along = |direction: Task| {
                let unit = direction / direction.norm();
                (side * unit.dot(r), unit * unit.dot(r))
            };
            // Whole task, and the translation alone: with position first the orientation's share
            // of the residual can cancel the translation's push on a pin, and the flag that
            // drives position first would drop where the translation is still held back.
            let mut translation = Task::zeros();
            translation
                .fixed_rows_mut::<3>(0)
                .copy_from(&column.fixed_rows::<3>(0));
            let (whole, push) = along(column.into());
            let (part, partial) = if translation.norm() > 0.0 {
                along(translation)
            } else {
                (0.0, Task::zeros())
            };
            let (pressure, push) = if part > whole {
                (part, partial)
            } else {
                (whole, push)
            };
            match pin.abs() {
                0 => free += column * column.transpose(),
                1 if pressure > 0.0 => {
                    position = position.max(pressure);
                    pushes[i] = push;
                }
                1 => {}
                _ => velocity = velocity.max(pressure.max(0.0)),
            }
        }
        self.withheld = [None; 2];
        let pushed: f64 = pushes.iter().sum::<Task>().norm();
        if let Some(factor) = (pushed > WALL_FLOOR).then(|| free.cholesky()).flatten() {
            let parts = pushes.map(|push| factor.solve(&push) * (damping * damping));
            let wall: Task = parts.iter().sum();
            for (k, block) in self.withheld.iter_mut().enumerate() {
                let of = |t: &Task| Vector3::from(t.fixed_rows::<3>(3 * k));
                let size = of(&wall).norm();
                if size > WALL_SHARE * pushed {
                    let normal = of(&wall) / size;
                    let joints = std::array::from_fn(|i| {
                        let along = of(&parts[i]).dot(&normal) > WALL_SHARE * size;
                        if along {
                            pinned[i].signum()
                        } else {
                            0
                        }
                    });
                    *block = Some(Wall { normal, joints });
                }
            }
        }
        self.record(position, velocity);
    }

    /// The hysteresis on the pressures of a cycle. A cycle whose position pressure is at or below
    /// [`WALL_FLOOR`] is quiet: any push at all counts, as it counts for a wall, because position
    /// first takes the goal to where the pins allow and so lowers the very pressure that set the
    /// flag (measured: 2.0e-5 at β 0.87 down to 8.6e-6 at 0.98) — a flag cleared on a magnitude
    /// would cycle between the two modes. The flag clears on [`STALL_QUIET_CYCLES`] quiet cycles
    /// within the last [`STALL_QUIET_WINDOW`], so a pin that is no longer being pushed into its
    /// bound clears it even if it is brushed now and then.
    pub(in super::super) fn record(&mut self, position: f64, velocity: f64) {
        if position > STALL_ON {
            // Set: the flag gets a fresh window, so an old run of quiet cycles cannot clear it
            // again on the next one.
            (self.quiet, self.stalled) = (0, true);
        } else {
            self.quiet = (self.quiet << 1) | u64::from(position <= WALL_FLOOR);
            let window = self.quiet & ((1u64 << STALL_QUIET_WINDOW) - 1);
            self.stalled &= window.count_ones() < STALL_QUIET_CYCLES;
        }
        (self.position, self.velocity) = (position, velocity);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pin brushed every few cycles used to restart the run of quiet cycles and latch the flag
    /// on for the rest of the session; a pin still pushed must keep holding it.
    #[test]
    fn the_stall_flag_clears_on_a_pin_no_longer_pushed_and_holds_while_it_is() {
        let mut stall = Stall::default();
        stall.record(2.0 * STALL_ON, 0.0);
        assert!(stall.stalled);
        // Pushed every cycle: held, however lightly.
        for _ in 0..1000 {
            stall.record(2.0 * WALL_FLOOR, 0.0);
        }
        assert!(stall.stalled, "a pushed pin cleared the flag");
        // Resting on its margin, brushed every fourth cycle: 30 of the last 40 are quiet.
        for k in 0..1000 {
            stall.record(if k % 4 == 0 { 2.0 * WALL_FLOOR } else { 0.0 }, 0.0);
        }
        assert!(!stall.stalled, "a pin at rest never cleared the flag");
        // And pushing again sets it at once.
        stall.record(2.0 * STALL_ON, 0.0);
        assert!(stall.stalled);
        for _ in 0..STALL_QUIET_CYCLES - 1 {
            stall.record(0.0, 0.0);
        }
        assert!(stall.stalled, "the flag cleared inside the window");
        stall.record(0.0, 0.0);
        assert!(!stall.stalled);
    }
}
