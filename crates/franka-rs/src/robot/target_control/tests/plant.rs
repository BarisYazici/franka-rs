//! A closed-loop arm for the torque backend, offline: `(M(q) + diag(armature)) q̈ + c(q, q̇) =
//! τ + τ_ext` on the crate's FER or FR3 model, gravity being the robot's. The command goes
//! through what the robot does with it -- clamped, low-passed against the last command, rate
//! limited -- and acts one cycle after the state it was computed from, as in
//! [`velocity`](super::velocity)'s wrist. `τ_ext` is a disturbance of the test's choosing. The
//! tools of the scenarios, and the loops as they ran before the guard in [`as_run`].

pub(super) mod as_run;

use std::collections::VecDeque;

use nalgebra::{SMatrix, SVector};

use super::super::*;
use crate::lowpass_filter::low_pass_filter;
use crate::model::{Frame, Model};
use crate::rate_limiting::fer::{JOINT_POSITION_LIMITS, MAX_TORQUE_RATE};
use crate::rate_limiting::{limit_rate_torques, DELTA_T};
use crate::wire::robot::codec::FciVersion;

pub(super) type Limits = ([f64; 7], [f64; 7]);

/// The smallest distance, rad, of any joint of any of `q` to `limits`, and that joint.
pub(super) fn nearest(q: &[[f64; 7]], limits: &Limits) -> (f64, usize) {
    q.iter()
        .flat_map(|q| (0..7).map(move |i| ((q[i] - limits.0[i]).min(limits.1[i] - q[i]), i)))
        .fold((f64::INFINITY, 0), |a, b| if b.0 < a.0 { b } else { a })
}

/// A tool: its `F_T_EE` (column-major) and its load, flange frame: mass, kg, centre of mass,
/// m, and inertia, kg m².
#[derive(Debug, Clone, Copy)]
pub(super) struct Tool {
    pub name: &'static str,
    pub f_t_ee: [f64; 16],
    pub load: (f64, [f64; 3], [f64; 9]),
}

const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// A diagonal inertia, kg m².
const fn diagonal(i: f64) -> [f64; 9] {
    [i, 0.0, 0.0, 0.0, i, 0.0, 0.0, 0.0, i]
}

const fn along_z(z: f64) -> [f64; 16] {
    let mut t = IDENTITY;
    t[14] = z;
    t
}

pub(super) const FLANGE: Tool = Tool {
    name: "flange",
    f_t_ee: IDENTITY,
    load: (0.0, [0.0; 3], [0.0; 9]),
};

/// The Franka Hand: `F_T_EE` 0.1034 m along z, -45 degrees about it; 0.73 kg.
pub(super) const HAND: Tool = Tool {
    name: "hand",
    f_t_ee: {
        let c = std::f64::consts::FRAC_1_SQRT_2;
        [
            c, -c, 0.0, 0.0, c, c, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1034, 1.0,
        ]
    },
    load: (
        0.73,
        [-0.01, 0.0, 0.03],
        [0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017],
    ),
};

/// A Robotiq 2F-85's TCP, 0.174 m along z, unturned; 0.925 kg.
pub(super) const ROBOTIQ: Tool = Tool {
    name: "robotiq",
    f_t_ee: along_z(0.174),
    load: (0.925, [0.0, 0.0, 0.06], diagonal(0.001)),
};

/// A long tool's tip, 0.25 m along z; 0.5 kg at half its length.
pub(super) const LONG_TOOL: Tool = Tool {
    name: "long tool",
    f_t_ee: along_z(0.25),
    load: (0.5, [0.0, 0.0, 0.125], diagonal(0.003)),
};

/// A fast teleop budget, 1 m/s, 8 m/s², 400 m/s³ and 4 rad/s, 20 rad/s², 500 rad/s³, the
/// deviation guards out of the way.
pub(super) fn teleop() -> TargetControlOptions {
    let limits = |max_velocity, max_acceleration, max_jerk| crate::otg::OtgLimits {
        max_velocity,
        max_acceleration,
        max_jerk,
    };
    TargetControlOptions::default()
        .with_limits(limits(1.0, 8.0, 400.0))
        .with_rotation_limits(limits(4.0, 20.0, 500.0))
        .with_max_deviation(10.0)
        .with_max_angular_deviation(10.0)
}

/// The drives' reflected inertia, kg m²: `motor_inertia × gear_ratio²` of franka_description's
/// `robots/fer/dynamics.yaml` (the URDF itself carries none). The FR3's is not published; its
/// plant takes the FER's, a guess its lighter wrist likely overstates.
const FER_ARMATURE: [f64; 7] = [0.6057, 0.6057, 0.4625, 0.4625, 0.2055, 0.2055, 0.2055];

/// The model of the arm speaking `version`.
fn model_of(version: FciVersion) -> Model {
    match version {
        FciVersion::V5 => Model::native_fer(),
        FciVersion::V10 => {
            Model::from_urdf(include_str!("../../../../tests/data/fr3.urdf")).expect("fr3.urdf")
        }
    }
}

/// How the [`Plant`] is built.
#[derive(Debug, Clone, Copy)]
pub(super) struct PlantOptions {
    /// The arm's model. Default the FER's.
    pub version: FciVersion,
    /// Added to the mass matrix's diagonal, kg m². Default [`FER_ARMATURE`].
    pub armature: [f64; 7],
    /// Cycles between the state the loop sees and the cycle its torque starts acting in.
    /// Default 1, the wrist's.
    pub latency: usize,
    /// Integration steps per cycle. Default 1: the torque is held over the cycle and the free
    /// plant has no stiffness of its own, so more only refine `M(q)` and `c` within a cycle.
    pub substeps: usize,
    /// The command's low-pass, Hz, as `ImpedanceOptions::cutoff_frequency`. Default 100.
    pub cutoff_frequency: f64,
    /// The clamp on the command, Nm. Default the Cartesian preset's.
    pub torque_limits: [f64; 7],
    /// Joint limits the plant reports a violation of. Default the FER's.
    pub limits: Limits,
    /// The tool. Default [`HAND`].
    pub tool: Tool,
}

impl Default for PlantOptions {
    fn default() -> Self {
        PlantOptions {
            version: FciVersion::V5,
            armature: FER_ARMATURE,
            latency: 1,
            substeps: 1,
            cutoff_frequency: 100.0,
            torque_limits: ImpedanceOptions::cartesian().torque_limits,
            limits: JOINT_POSITION_LIMITS,
            tool: HAND,
        }
    }
}

/// See the [module documentation](self).
pub(super) struct Plant {
    model: Model,
    options: PlantOptions,
    /// The states after the last `latency + 1` cycles, the oldest first.
    seen: VecDeque<RobotState>,
    q: [f64; 7],
    dq: [f64; 7],
    /// The last command after the clamp, the low-pass and the rate limit: what acts.
    applied: [f64; 7],
    cycles: usize,
    violation: Option<(usize, usize)>,
}

impl Plant {
    /// At rest in `q`.
    pub(super) fn new(q: [f64; 7], options: PlantOptions) -> Plant {
        let mut plant = Plant {
            model: model_of(options.version),
            options,
            seen: VecDeque::with_capacity(options.latency + 1),
            q,
            dq: [0.0; 7],
            applied: [0.0; 7],
            cycles: 0,
            violation: None,
        };
        let now = plant.now();
        plant
            .seen
            .extend(std::iter::repeat_n(now, options.latency + 1));
        plant
    }

    /// The state the loop is given this cycle: `latency` cycles old.
    pub(super) fn state(&self) -> RobotState {
        self.seen[0]
    }

    /// The plant's configuration and velocity now, not as the loop sees them.
    pub(super) fn joints(&self) -> ([f64; 7], [f64; 7]) {
        (self.q, self.dq)
    }

    /// The torque acting now, after the clamp, the low-pass and the rate limit.
    pub(super) fn applied(&self) -> [f64; 7] {
        self.applied
    }

    /// The first cycle (counted from 0) and joint (from 0) outside the limits, if any.
    pub(super) fn violation(&self) -> Option<(usize, usize)> {
        self.violation
    }

    /// One cycle under the loop's command `tau`.
    pub(super) fn step(&mut self, tau: &[f64; 7]) {
        self.step_with(tau, &[0.0; 7]);
    }

    /// One cycle under the loop's command `tau` and the external torque `external`, Nm.
    pub(super) fn step_with(&mut self, tau: &[f64; 7], external: &[f64; 7]) {
        let o = self.options;
        let limited: [f64; 7] = std::array::from_fn(|i| {
            let clamped = tau[i].clamp(-o.torque_limits[i], o.torque_limits[i]);
            low_pass_filter(DELTA_T, clamped, self.applied[i], o.cutoff_frequency).unwrap()
        });
        self.applied = limit_rate_torques(&MAX_TORQUE_RATE, &limited, &self.applied).unwrap();
        let h = DELTA_T / o.substeps as f64;
        let tau = SVector::<f64, 7>::from(self.applied) + SVector::from(*external);
        for _ in 0..o.substeps {
            let state = self.now();
            let mut m = SMatrix::<f64, 7, 7>::from_column_slice(&self.model.mass(&state));
            for i in 0..7 {
                m[(i, i)] += o.armature[i];
            }
            let c = self.model.coriolis(&state);
            let ddq = m
                .cholesky()
                .expect("the mass matrix is positive definite")
                .solve(&(tau - SVector::from(c)));
            for i in 0..7 {
                self.dq[i] += h * ddq[i];
                self.q[i] += h * self.dq[i];
            }
        }
        let (lower, upper) = o.limits;
        if self.violation.is_none() {
            self.violation = (0..7)
                .find(|&i| self.q[i] < lower[i] || self.q[i] > upper[i])
                .map(|i| (self.cycles, i));
        }
        self.cycles += 1;
        self.seen.pop_front();
        let now = self.now();
        self.seen.push_back(now);
    }

    /// The robot state of the plant as it is.
    fn now(&self) -> RobotState {
        let (mass, com, inertia) = self.options.tool.load;
        let mut state = RobotState {
            q: self.q,
            q_d: self.q,
            dq: self.dq,
            tau_J: self.applied,
            tau_J_d: self.applied,
            F_T_EE: self.options.tool.f_t_ee,
            m_ee: mass,
            F_x_Cee: com,
            I_ee: inertia,
            m_total: mass,
            F_x_Ctotal: com,
            I_total: inertia,
            ..RobotState::default()
        };
        state.F_T_NE = state.F_T_EE;
        state.O_T_EE = self.model.pose(Frame::EndEffector, &state);
        state.O_T_EE_c = state.O_T_EE;
        state.O_T_EE_d = state.O_T_EE;
        state
    }
}
