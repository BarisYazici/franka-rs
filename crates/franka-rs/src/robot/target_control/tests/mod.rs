//! The shared scaffolding: the mock arm, the recording observer. The generator runner in
//! [`runner`], the pose path in [`pose`], the impedance law in [`impedance`], the option
//! validation in [`options`], the rotation arithmetic in [`rotation`], the torque loops in
//! [`torque_cartesian`] and [`torque_joint`] on the helpers of [`torque`].

mod ik;
mod impedance;
mod options;
mod pose;
mod rotation;
mod runner;
mod torque;
mod torque_cartesian;
mod torque_joint;

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};
use std::sync::{Arc, Mutex};

use super::ik::Ik;
use super::*;
use crate::model::{Frame, Model};
use crate::rate_limiting::{self, DELTA_T};

fn is_invalid_argument(result: FrankaResult<()>, needle: &str) -> bool {
    matches!(result, Err(FrankaError::InvalidArgument(m)) if m.contains(needle))
}

const READY: [f64; 7] = [
    0.0,
    -FRAC_PI_4,
    0.0,
    -3.0 * FRAC_PI_4,
    0.0,
    FRAC_PI_2,
    FRAC_PI_4,
];

/// A mock arm on the FER model: at rest in `q`, its `O_T_EE` the model's forward kinematics
/// (the hand and stiffness frames at the identity, as `RobotState::default()` has them).
/// [`follow`](Arm::follow) makes it a perfect tracker one cycle behind the goal.
struct Arm {
    model: Arc<Model>,
    state: RobotState,
}

impl Arm {
    fn at(q: [f64; 7]) -> Arm {
        let model = Arc::new(Model::native_fer());
        let mut state = RobotState::default();
        state.q = q;
        state.q_d = q;
        state.O_T_EE = model.pose(Frame::EndEffector, &state);
        state.O_T_EE_c = state.O_T_EE;
        Arm { model, state }
    }

    /// Moves to `q_goal` in one cycle, `dq` the finite difference.
    fn follow(&mut self, q_goal: &[f64; 7]) {
        let q = self.state.q;
        self.state.dq = std::array::from_fn(|i| (q_goal[i] - q[i]) / DELTA_T);
        self.state.q = *q_goal;
        self.state.q_d = *q_goal;
        self.state.O_T_EE = self.model.pose(Frame::EndEffector, &self.state);
        self.state.O_T_EE_c = self.state.O_T_EE;
    }

    /// Moves the arm, at rest, to the configuration the IK finds `dx` metres along x.
    fn drag_along_x(&mut self, dx: f64) {
        let mut pose = self.pose();
        pose[12] += dx;
        let options = IkOptions {
            max_step: 1.0,
            ..IkOptions::default()
        };
        let mut ik = Ik::new(
            Arc::clone(&self.model),
            options,
            rate_limiting::fer::JOINT_POSITION_LIMITS,
            self.state.q,
            self.state.F_T_EE,
            self.state.EE_T_K,
        );
        let posture = self.state.q;
        let residual = (0..100).fold(f64::INFINITY, |_, _| ik.step(&pose, &posture, DELTA_T).1);
        assert!(residual < 1e-6, "the drag did not converge: {residual}");
        self.follow(&ik.q());
        self.state.dq = [0.0; 7];
    }

    /// The model's pose of the measured configuration.
    fn pose(&self) -> [f64; 16] {
        self.model.pose(Frame::EndEffector, &self.state)
    }
}

type Records<S> = Arc<Mutex<Vec<S>>>;

/// An observer that keeps every record.
fn recording<S: Copy + Send + 'static>() -> (Records<S>, impl FnMut(&RobotState, &S)) {
    let records = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&records);
    (records, move |_: &RobotState, sent: &S| {
        sink.lock().unwrap().push(*sent)
    })
}
