//! The streaming core of the flight recorder: a [`FlightLogger`] takes batches of
//! [`franka::Record`]s and writes the time series, the 3D scene and the event log for them,
//! keeping across batches whatever the next batch needs to be compared against (the previous
//! errors and mode, which flags have fired, the record index). [`super::log_records`] feeds it
//! one batch; the [`crate::Recorder`] feeds it every ~100 ms from a background thread.

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::{Errors, Model, Record, RobotMode, RobotState};
use rerun::{
    Arrows3D, Color, Points3D, RecordingStream, Scalars, TextLog, TextLogLevel, TimeColumn,
};

use super::style::{self, Flags, AXES, COLLISION, CONTACT, FORCE, JOINT_NAMES, QUIET};
use super::{FlightOptions, Summary};
use crate::{scene, Result, RobotKind, TIMELINE};

/// Sphere radius at a joint, m, plus this much per Nm of `|tau_ext_hat_filtered|`.
const JOINT_RADIUS: f64 = 0.02;
const JOINT_RADIUS_PER_NM: f64 = 0.0015;

/// Where the timeline comes from; decided on the first batch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Timebase {
    Undecided,
    /// `state.time` in seconds.
    Robot,
    /// The record index as milliseconds, for logs whose `time` never changes.
    Index,
}

/// Writes records to a stream batch by batch; see the module documentation.
pub struct FlightLogger<'m> {
    rec: RecordingStream,
    model: &'m Model,
    kind: RobotKind,
    options: FlightOptions,
    timebase: Timebase,
    /// The previous record's errors and mode, once there is one.
    previous: Option<(Errors, RobotMode)>,
    joint_contact_seen: [bool; 7],
    joint_collision_seen: [bool; 7],
    cartesian_contact_seen: [bool; 6],
    cartesian_collision_seen: [bool; 6],
    /// The previous record's four flag arrays, for the rising edges.
    flags: Option<Flags>,
    last_time: f64,
    summary: Summary,
}

/// The commanded joint positions: the command's `q_c` when the cycle sent a joint-position
/// command (an all-zero `q_c` is not a joint command, joint 4 cannot be at zero), else the
/// robot's own `q_d`.
fn commanded_q(record: &Record) -> [f64; 7] {
    match record.command {
        Some(command) if command.q_c.iter().any(|&v| v != 0.0) => command.q_c,
        _ => record.state.q_d,
    }
}

fn force_norm(state: &RobotState) -> f64 {
    let f = &state.O_F_ext_hat_K;
    (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt()
}

impl<'m> FlightLogger<'m> {
    /// Logs the static parts (series styles, the 3D base, the end effector axes) and returns a
    /// logger ready for [`FlightLogger::log_batch`].
    pub fn new(
        rec: &RecordingStream,
        model: &'m Model,
        kind: RobotKind,
        options: FlightOptions,
    ) -> Result<FlightLogger<'m>> {
        style::log_styles(rec)?;
        scene::log_static(rec, None)?;
        Ok(FlightLogger {
            rec: rec.clone(),
            model,
            kind,
            options: FlightOptions {
                every: options.every.max(1),
                ..options
            },
            timebase: Timebase::Undecided,
            previous: None,
            joint_contact_seen: [false; 7],
            joint_collision_seen: [false; 7],
            cartesian_contact_seen: [false; 6],
            cartesian_collision_seen: [false; 6],
            flags: None,
            last_time: 0.0,
            summary: Summary::default(),
        })
    }

    /// Timeline values for `records`, which start at record index `first`.
    fn times(&mut self, first: usize, records: &[Record]) -> Vec<f64> {
        if self.timebase == Timebase::Undecided {
            let t0 = records[0].state.time;
            let constant = records.len() >= 2 && records.iter().all(|r| r.state.time == t0);
            self.timebase = if constant {
                Timebase::Index
            } else {
                Timebase::Robot
            };
        }
        records
            .iter()
            .enumerate()
            .map(|(i, r)| match self.timebase {
                Timebase::Index => (first + i) as f64 * 1e-3,
                _ => r.state.time.as_secs_f64(),
            })
            .collect()
    }

    /// Writes `records` -- the next ones after everything logged so far -- to the stream.
    pub fn log_batch(&mut self, records: &[Record]) -> Result<()> {
        if records.is_empty() {
            return Ok(());
        }
        let first = self.summary.records;
        let t = self.times(first, records);
        if first == 0 {
            self.summary.first_time = t[0];
        }
        self.last_time = t[t.len() - 1];
        self.summary.last_time = self.last_time;
        self.summary.records += records.len();

        self.send_series("joints/q", &t, records, |r| r.state.q)?;
        self.send_series("joints/q_d", &t, records, commanded_q)?;
        self.send_series("joints/dq", &t, records, |r| r.state.dq)?;
        self.send_series("joints/tau_J", &t, records, |r| r.state.tau_J)?;
        self.send_series("joints/tau_J_d", &t, records, |r| r.state.tau_J_d)?;
        self.send_series("joints/tau_ext", &t, records, |r| {
            r.state.tau_ext_hat_filtered
        })?;
        self.send_series("ee/F_ext", &t, records, |r| r.state.O_F_ext_hat_K)?;
        self.send_series("ee/position", &t, records, |r| {
            let (m, c) = (&r.state.O_T_EE, &r.state.O_T_EE_c);
            [m[12], m[13], m[14], c[12], c[13], c[14]]
        })?;
        self.send_series("flags/joint_contact", &t, records, |r| {
            r.state.joint_contact
        })?;
        self.send_series("flags/joint_collision", &t, records, |r| {
            r.state.joint_collision
        })?;
        self.send_series("flags/cartesian_contact", &t, records, |r| {
            r.state.cartesian_contact
        })?;
        self.send_series("flags/cartesian_collision", &t, records, |r| {
            r.state.cartesian_collision
        })?;

        for (i, (record, &time)) in records.iter().zip(&t).enumerate() {
            let state = &record.state;
            let force = force_norm(state);
            if force > self.summary.peak_force {
                self.summary.peak_force = force;
                self.summary.peak_force_time = time;
            }
            for (joint, &tau) in state.tau_ext_hat_filtered.iter().enumerate() {
                if tau.abs() > self.summary.peak_tau_ext {
                    self.summary.peak_tau_ext = tau.abs();
                    self.summary.peak_tau_ext_joint = joint + 1;
                }
            }
            self.log_events(time, state)?;
            if (first + i).is_multiple_of(self.options.every) {
                self.log_scene(time, state)?;
            }
        }
        Ok(())
    }

    fn send_series<const N: usize>(
        &self,
        entity: &str,
        t: &[f64],
        records: &[Record],
        f: impl Fn(&Record) -> [f64; N],
    ) -> Result<()> {
        let values = records.iter().flat_map(f);
        let columns = Scalars::new(values).columns(std::iter::repeat_n(N, t.len()))?;
        let times = TimeColumn::new_duration_secs(TIMELINE, t.iter().copied());
        self.rec.send_columns(entity, [times], columns)?;
        Ok(())
    }

    /// The arm, the joint spheres and the external force arrow at `time`.
    fn log_scene(&self, time: f64, state: &RobotState) -> Result<()> {
        self.rec.set_duration_secs(TIMELINE, time);
        let points = scene::skeleton(self.model, &state.q, &state.F_T_EE);
        let ee_pose = self.model.pose_q(
            franka::Frame::EndEffector,
            &state.q,
            &state.F_T_EE,
            &IDENTITY_TRANSFORM,
        );
        let ee = scene::log_skeleton(&self.rec, &points, &ee_pose)?;
        let flags = Flags::of(state);
        let joints: [[f32; 3]; 7] = std::array::from_fn(|j| points[j + 1].map(|v| v as f32));
        let colors: [Color; 7] = std::array::from_fn(|j| {
            Color::from_u32(if flags.joint_collision[j] {
                COLLISION
            } else if flags.joint_contact[j] {
                CONTACT
            } else {
                QUIET
            })
        });
        let radii: [f32; 7] = std::array::from_fn(|j| {
            (JOINT_RADIUS + JOINT_RADIUS_PER_NM * state.tau_ext_hat_filtered[j].abs()) as f32
        });
        let spheres = Points3D::new(joints)
            .with_colors(colors)
            .with_radii(radii)
            .with_labels(JOINT_NAMES)
            .with_show_labels(false);
        self.rec.log("world/joints", &spheres)?;

        let f = &state.O_F_ext_hat_K;
        let scale = self.options.force_scale;
        let vector = [f[0] * scale, f[1] * scale, f[2] * scale].map(|v| v as f32);
        let color = if flags.cartesian_collision.iter().any(|&c| c) {
            COLLISION
        } else if flags.cartesian_contact.iter().any(|&c| c) {
            CONTACT
        } else {
            FORCE
        };
        let arrow = Arrows3D::from_vectors([vector])
            .with_origins([ee.map(|v| v as f32)])
            .with_colors([Color::from_u32(color)])
            .with_radii([0.006]);
        self.rec.log("world/force", &arrow)?;
        Ok(())
    }

    fn event(&mut self, time: f64, level: &str, text: String) -> Result<()> {
        self.rec.set_duration_secs(TIMELINE, time);
        self.rec
            .log("events", &TextLog::new(text).with_level(level))?;
        self.summary.events += 1;
        Ok(())
    }

    /// Error and mode changes, and the first rising edge of every contact/collision flag.
    fn log_events(&mut self, time: f64, state: &RobotState) -> Result<()> {
        let (errors, mode) = (state.current_errors, state.robot_mode);
        match self.previous {
            None => {
                let kind = self.kind;
                self.event(time, TextLogLevel::INFO, format!("{kind:?}: mode {mode}"))?;
                if errors.any() {
                    let text = format!("errors at start: {errors}");
                    self.event(time, TextLogLevel::WARN, text)?;
                }
            }
            Some((last_errors, last_mode)) => {
                if errors != last_errors {
                    self.summary.error_changes += 1;
                    let set: Vec<&str> = (0..franka::errors::ERROR_COUNT)
                        .filter(|&i| errors.0[i] && !last_errors.0[i])
                        .map(|i| franka::ERROR_NAMES[i])
                        .collect();
                    let cleared: Vec<&str> = (0..franka::errors::ERROR_COUNT)
                        .filter(|&i| !errors.0[i] && last_errors.0[i])
                        .map(|i| franka::ERROR_NAMES[i])
                        .collect();
                    if !set.is_empty() {
                        let text = format!("errors set: {}", set.join(", "));
                        self.event(time, TextLogLevel::WARN, text)?;
                    }
                    if !cleared.is_empty() {
                        let text = format!("errors cleared: {}", cleared.join(", "));
                        self.event(time, TextLogLevel::INFO, text)?;
                    }
                }
                if mode != last_mode {
                    self.summary.mode_changes += 1;
                    let level = if mode == RobotMode::Reflex {
                        TextLogLevel::ERROR
                    } else {
                        TextLogLevel::INFO
                    };
                    self.event(time, level, format!("mode {last_mode} -> {mode}"))?;
                }
            }
        }
        self.previous = Some((errors, mode));

        let flags = Flags::of(state);
        let last = self.flags.unwrap_or(Flags::NONE);
        self.flags = Some(flags);
        let (n, seen) = self.rising_edges(
            time,
            flags.joint_contact,
            last.joint_contact,
            self.joint_contact_seen,
            &JOINT_NAMES,
            |joint| format!("{joint} contact"),
            TextLogLevel::WARN,
        )?;
        self.summary.joint_contacts += n;
        self.joint_contact_seen = seen;
        let (n, seen) = self.rising_edges(
            time,
            flags.joint_collision,
            last.joint_collision,
            self.joint_collision_seen,
            &JOINT_NAMES,
            |joint| format!("{joint} collision"),
            TextLogLevel::ERROR,
        )?;
        self.summary.joint_collisions += n;
        self.joint_collision_seen = seen;
        let (n, seen) = self.rising_edges(
            time,
            flags.cartesian_contact,
            last.cartesian_contact,
            self.cartesian_contact_seen,
            &AXES,
            |axis| format!("cartesian contact on {axis}"),
            TextLogLevel::WARN,
        )?;
        self.summary.cartesian_contacts += n;
        self.cartesian_contact_seen = seen;
        let (n, seen) = self.rising_edges(
            time,
            flags.cartesian_collision,
            last.cartesian_collision,
            self.cartesian_collision_seen,
            &AXES,
            |axis| format!("cartesian collision on {axis}"),
            TextLogLevel::ERROR,
        )?;
        self.summary.cartesian_collisions += n;
        self.cartesian_collision_seen = seen;
        Ok(())
    }

    /// Rising edges of one family of flags between `last` and `now`; the first one of every
    /// index (per `seen`) becomes an event. Returns the count and the updated `seen`.
    #[allow(clippy::too_many_arguments)]
    fn rising_edges<const N: usize>(
        &mut self,
        time: f64,
        now: [bool; N],
        last: [bool; N],
        mut seen: [bool; N],
        names: &[&str; N],
        label: impl Fn(&str) -> String,
        level: &str,
    ) -> Result<(usize, [bool; N])> {
        let mut count = 0;
        for (i, name) in names.iter().enumerate() {
            if now[i] && !last[i] {
                count += 1;
                if !seen[i] {
                    seen[i] = true;
                    self.event(time, level, label(name))?;
                }
            }
        }
        Ok((count, seen))
    }

    /// The abort line at the last record's time if `last_motion_errors` has any flag set,
    /// then a blocking flush. Returns what was recorded.
    pub fn finish(mut self, last_motion_errors: Option<&Errors>) -> Result<Summary> {
        if let Some(errors) = last_motion_errors.filter(|e| e.any()) {
            self.summary.motion_errors = errors.names();
            let text = format!("motion aborted: {}", errors.names().join(", "));
            let time = self.last_time;
            self.event(time, TextLogLevel::ERROR, text)?;
        }
        self.rec.flush_blocking()?;
        Ok(self.summary)
    }
}
