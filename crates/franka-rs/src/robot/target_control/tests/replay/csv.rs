//! A recorded torque-backend session as CSV, replayed commanded (the loop is given the
//! recorded `q` and `dq`) or closed loop (on the [`Plant`]), as it ran or with the guard. The
//! header names the columns; read are `t`, `target_x..z`, `target_qx..qw`, `target_seq`, and
//! `q`, `dq`, `q_goal` with suffixes 1 to 7, optionally `tau_ext1..7` and `target_q1..7` (a
//! joint-space session). One row per cycle; a target is new wherever `target_seq` changes.
//!
//! The recordings are found in the directory `REPLAY_DIR` names; without it the replays skip.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::ops::Range;

use super::super::super::torque::{TorqueLoop, Tracker};
use super::super::super::*;
use super::super::plant::as_run::{self, Restart};
use super::super::plant::{Plant, PlantOptions, Tool, HAND, ROBOTIQ};
use super::super::recording;
use super::super::torque::{cartesian_loop_on, Rig};
use super::{limits, loop_options, Config};
use crate::model::Frame;

/// A loaded session, one entry per cycle.
pub(in super::super) struct Session {
    pub t: Vec<f64>,
    pub q: Vec<[f64; 7]>,
    pub dq: Vec<[f64; 7]>,
    pub q_goal: Vec<[f64; 7]>,
    /// Each new target with the cycle it first appears on: x, y, z, then the unit quaternion
    /// x, y, z, w.
    pub targets: Vec<(usize, [f64; 7])>,
    /// `tau_ext_hat_filtered` per cycle, zero where not recorded.
    pub tau_ext: Vec<[f64; 7]>,
    /// Each new joint-space target with the cycle it first appears on.
    pub joint_targets: Vec<(usize, [f64; 7])>,
}

/// The path of the recording `name` (`<name>.csv`) in `REPLAY_DIR`, `None` without it.
pub(in super::super) fn recording_path(name: &str) -> Option<String> {
    let dir = std::env::var("REPLAY_DIR").ok()?;
    Some(format!("{dir}/{name}.csv"))
}

/// The session in the CSV file at `path`, read a line at a time, only the columns above parsed.
pub(in super::super) fn load(path: &str) -> Session {
    let file = File::open(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut lines = BufReader::new(file)
        .lines()
        .map(|line| line.unwrap_or_else(|e| panic!("{path}: {e}")));
    let header = lines.next().expect("a header");
    let names: Vec<&str> = header.split(',').map(str::trim).collect();
    let find = |name: &str| names.iter().position(|h| *h == name);
    let column = |name: &str| find(name).unwrap_or_else(|| panic!("{path}: no column {name}"));
    let joints = |prefix: &str| -> [usize; 7] {
        std::array::from_fn(|i| column(&format!("{prefix}{}", i + 1)))
    };
    let optional = |prefix: &str| find(&format!("{prefix}1")).map(|_| joints(prefix));
    let pose = ["x", "y", "z", "qx", "qy", "qz", "qw"].map(|c| column(&format!("target_{c}")));
    let (t, seq) = (column("t"), column("target_seq"));
    let (q, dq, q_goal) = (joints("q"), joints("dq"), joints("q_goal"));
    let (tau_ext, target_q) = (optional("tau_ext"), optional("target_q"));
    let mut wanted = vec![false; names.len()];
    let groups = [
        Some(pose),
        Some(q),
        Some(dq),
        Some(q_goal),
        tau_ext,
        target_q,
    ];
    for c in groups.iter().flatten().flatten().chain(&[t, seq]) {
        wanted[*c] = true;
    }
    let mut session = Session {
        t: Vec::new(),
        q: Vec::new(),
        dq: Vec::new(),
        q_goal: Vec::new(),
        targets: Vec::new(),
        tau_ext: Vec::new(),
        joint_targets: Vec::new(),
    };
    let (mut row, mut last_seq) = (vec![f64::NAN; names.len()], f64::NAN);
    for (cycle, line) in lines.enumerate() {
        row.fill(f64::NAN);
        for (c, field) in line.split(',').enumerate().filter(|(c, _)| wanted[*c]) {
            row[c] = field.trim().parse().unwrap_or(f64::NAN);
        }
        let pick = |columns: &[usize; 7]| columns.map(|c| row[c]);
        session.t.push(row[t]);
        session.q.push(pick(&q));
        session.dq.push(pick(&dq));
        session.q_goal.push(pick(&q_goal));
        let external = tau_ext.map_or([0.0; 7], |c| {
            pick(&c).map(|x| if x.is_finite() { x } else { 0.0 })
        });
        session.tau_ext.push(external);
        if row[seq].is_finite() && row[seq] != last_seq {
            last_seq = row[seq];
            let mut target = pick(&pose);
            if target.iter().all(|x| x.is_finite()) {
                let n = target[3..].iter().map(|c| c * c).sum::<f64>().sqrt();
                target[3..].iter_mut().for_each(|c| *c /= n);
                session.targets.push((cycle, target));
            }
            if let Some(joint) = target_q.map(|c| pick(&c)) {
                if joint.iter().all(|x| x.is_finite()) {
                    session.joint_targets.push((cycle, joint));
                }
            }
        }
    }
    session
}

/// The arm a session was recorded on, and its tool.
pub(in super::super) struct Arm {
    pub rig: Rig,
    pub tool: Tool,
}

impl Arm {
    /// The recordings' arm: an FER with the Franka Hand.
    pub(in super::super) fn fer() -> Arm {
        Arm {
            rig: Rig::fer(),
            tool: HAND,
        }
    }

    /// The FR3 recordings' arm, with the Robotiq gripper.
    pub(in super::super) fn fr3() -> Arm {
        Arm {
            rig: Rig::fr3(),
            tool: ROBOTIQ,
        }
    }

    /// The state the loop was given at `cycle` of `session`: the recorded `q` and `dq`, the
    /// model's pose of `q` with the tool.
    pub(in super::super) fn recorded(&self, session: &Session, cycle: usize) -> RobotState {
        let mut state = RobotState {
            q: session.q[cycle],
            dq: session.dq[cycle],
            F_T_EE: self.tool.f_t_ee,
            ..RobotState::default()
        };
        state.O_T_EE = self.rig.model.pose(Frame::EndEffector, &state);
        state
    }

    /// The pose of `q` with the tool, column-major.
    pub(in super::super) fn pose(&self, q: &[f64; 7]) -> [f64; 16] {
        let ee_t_k = RobotState::default().EE_T_K;
        let model = &self.rig.model;
        model.pose_q(Frame::EndEffector, q, &self.tool.f_t_ee, &ee_t_k)
    }
}

/// What drives the loop's measured state.
#[derive(Debug, Clone, Copy)]
pub(in super::super) enum Drive {
    /// The recorded `q` and `dq`.
    Commanded,
    /// The plant, from rest at the first cycle's recorded `q`, under the recorded `tau_ext`
    /// times `disturbance`, its `q` handed to the loop with `noise` rad of white noise on every
    /// joint (the arm's own encoders resolve about 1e-5 rad).
    Closed { disturbance: f64, noise: f64 },
}

impl Drive {
    /// The plant under `disturbance` times the recorded external torques, read exactly.
    pub(in super::super) fn plant(disturbance: f64) -> Drive {
        Drive::Closed {
            disturbance,
            noise: 0.0,
        }
    }
}

/// A seeded xorshift, one draw per call, uniform in `-1..1`.
struct Noise(u64);

impl Noise {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        f64::from((self.0 >> 40) as u32) / f64::from(1u32 << 23) - 1.0
    }
}

/// A replay's measured side, one entry per cycle: the state the loop was given and, for a
/// closed loop, the plant's own (not delayed) configuration, velocity and first limit violation.
pub(in super::super) struct Driven {
    pub q: Vec<[f64; 7]>,
    pub dq: Vec<[f64; 7]>,
    pub violation: Option<(usize, usize)>,
}

/// Runs `torque` over `cycles` of `session` on `arm`, publishing each of `targets` before the
/// cycle it first appears on (the first cycle anchors) and, on a mid-session start, the one in
/// force. Cycles past the session's end hold its last target, on the plant only.
pub(in super::super) fn drive<const N: usize, const S: usize, T: Tracker<N, S>>(
    torque: &mut TorqueLoop<N, S, T>,
    shared: &Shared<S>,
    session: &Session,
    cycles: Range<usize>,
    targets: &[(usize, [f64; S])],
    arm: &Arm,
    how: Drive,
) -> Driven {
    let (mut plant, (scale, noise)) = match how {
        Drive::Closed { disturbance, noise } => {
            let options = PlantOptions {
                limits: arm.rig.limits,
                version: arm.rig.version,
                tool: arm.tool,
                ..PlantOptions::default()
            };
            (
                Some(Plant::new(session.q[cycles.start], options)),
                (disturbance, noise),
            )
        }
        Drive::Commanded => (None, (0.0, 0.0)),
    };
    let mut draw = Noise(0x2545_f491_4f6c_dd1d);
    let mut driven = Driven {
        q: Vec::with_capacity(cycles.len()),
        dq: Vec::with_capacity(cycles.len()),
        violation: None,
    };
    let mut next = targets.partition_point(|(k, _)| *k <= cycles.start);
    for cycle in cycles.clone() {
        let mut state = match plant.as_ref() {
            Some(plant) => plant.state(),
            None => arm.recorded(session, cycle),
        };
        if noise > 0.0 {
            state.q = state.q.map(|q| q + noise * draw.next());
            state.O_T_EE = arm.rig.model.pose(Frame::EndEffector, &state);
        }
        if cycle == cycles.start + 1 && next > 0 {
            shared.slot.publish(targets[next - 1].1);
        }
        while cycle > cycles.start && next < targets.len() && targets[next].0 <= cycle {
            shared.slot.publish(targets[next].1);
            next += 1;
        }
        let tau = torque.cycle(&state).tau_J;
        match plant.as_mut() {
            Some(plant) => {
                let external = session
                    .tau_ext
                    .get(cycle)
                    .map_or([0.0; 7], |x| x.map(|x| scale * x));
                plant.step_with(&tau, &external);
                let (q, dq) = plant.joints();
                driven.q.push(q);
                driven.dq.push(dq);
            }
            None => {
                driven.q.push(state.q);
                driven.dq.push(state.dq);
            }
        }
    }
    driven.violation = plant.and_then(|p| p.violation());
    driven
}

/// The Cartesian loop as it ran, or guarded at a rotation weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(in super::super) enum Mode {
    AsRun(Restart),
    Guarded(f64),
}

/// The Cartesian loop at `config` in `mode` over `cycles` of `session` on `arm`, the posture the
/// session's first configuration, `targets` published before the cycles they name; its records
/// and measured side.
pub(in super::super) fn cartesian(
    session: &Session,
    targets: &[(usize, [f64; 7])],
    arm: &Arm,
    config: Config,
    mode: Mode,
    cycles: Range<usize>,
    how: Drive,
) -> (Vec<CartesianSent>, Driven) {
    let (records, observer) = recording::<CartesianSent>();
    let (options, impedance) = loop_options(config, observer);
    let mut impedance = impedance.with_posture(Some(session.q[0]));
    let rig = &arm.rig;
    let driven = match mode {
        Mode::AsRun(restart) => {
            let (mut torque, shared) = as_run::cartesian_loop(rig, options, impedance, restart);
            drive(&mut torque, &shared, session, cycles, targets, arm, how)
        }
        Mode::Guarded(weight) => {
            impedance.ik.rotation_weight = weight;
            let (mut torque, shared) = cartesian_loop_on(rig, options, impedance);
            drive(&mut torque, &shared, session, cycles, targets, arm, how)
        }
    };
    let sent = std::mem::take(&mut *records.lock().unwrap());
    (sent, driven)
}

/// How closely `goals` reproduce `recorded`, per cycle the largest joint's difference: its
/// median and p99, rad, and the first cycle (index into both) above 20 µrad.
pub(in super::super) fn oracle(
    goals: &[[f64; 7]],
    recorded: &[[f64; 7]],
) -> (f64, f64, Option<usize>) {
    let mut worst: Vec<f64> = goals
        .iter()
        .zip(recorded)
        .map(|(a, b)| (0..7).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max))
        .collect();
    let first = worst.iter().position(|d| *d > 20e-6);
    worst.sort_by(f64::total_cmp);
    let at = |p: f64| worst[((worst.len() - 1) as f64 * p).round() as usize];
    (at(0.5), at(0.99), first)
}

/// The last run of cycles whose goal of `joint` sits at `bound` (within `tolerance`): its
/// first cycle and its length.
pub(in super::super) fn pinned_run(
    q_goal: &[[f64; 7]],
    joint: usize,
    bound: f64,
    tolerance: f64,
) -> Option<(usize, usize)> {
    let pinned = |k: usize| (q_goal[k][joint] - bound).abs() <= tolerance;
    let end = (0..q_goal.len()).rev().find(|&k| pinned(k))?;
    let start = (0..=end).rev().take_while(|&k| pinned(k)).last()?;
    Some((start, end - start + 1))
}

impl Config {
    /// The VR teleop sessions' as run: budget 1 m/s, 20 m/s², 1000 m/s³, rotation 4, 50, 1250,
    /// 1200 N/m, leash 0.03 m and 0.25 rad, cap 0.7, the Franka Hand.
    pub(in super::super) fn teleop() -> Config {
        Config {
            budget: limits([1.0, 20.0, 1000.0]),
            rotation: limits([4.0, 50.0, 1250.0]),
            stiffness: 1200.0,
            leash: Leash {
                translation: 0.03,
                rotation: 0.25,
                ..Leash::default()
            },
            fraction: 0.7,
            hand: true,
        }
    }

    /// The wrist-limit sessions' as run: budget 0.5 m/s, 4 m/s², 200 m/s³, rotation 1.5, 10,
    /// 300, 750 N/m, else [`teleop`](Self::teleop) (their deviation guard, 2 m and 4 rad, never
    /// trips, as the replays' does not).
    pub(in super::super) fn wrist_limit() -> Config {
        Config {
            budget: limits([0.5, 4.0, 200.0]),
            rotation: limits([1.5, 10.0, 300.0]),
            stiffness: 750.0,
            ..Config::teleop()
        }
    }

    /// The FR3 VR sessions' as run: the wrist-limit budgets and stiffness, the default leash.
    pub(in super::super) fn fr3_vr() -> Config {
        Config {
            leash: Leash::default(),
            hand: false,
            ..Config::wrist_limit()
        }
    }

    /// The node's defaults: budget 0.3, 0.5, 20, rotation 0.5, 1, 20, 750 N/m, the default leash.
    pub(in super::super) fn node() -> Config {
        Config {
            budget: limits([0.3, 0.5, 20.0]),
            rotation: limits([0.5, 1.0, 20.0]),
            stiffness: 750.0,
            leash: Leash::default(),
            ..Config::teleop()
        }
    }
}
