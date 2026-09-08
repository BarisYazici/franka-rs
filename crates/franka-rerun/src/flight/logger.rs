//! The streaming core of the flight recorder: a [`FlightLogger`] takes batches of
//! [`franka::Record`]s and writes the time series, the 3D scene and the event log for them,
//! keeping across batches whatever the next batch needs to be compared against (the previous
//! errors and mode, which flags have fired, the record index). [`super::log_records`] feeds it
//! one batch; the [`crate::Recorder`] feeds it every ~100 ms from a background thread.

use franka::{Errors, Model, Record, RobotMode, RobotState};
use rerun::{RecordingStream, Scalars, TextLog, TextLogLevel, TimeColumn};

use super::cartesian::{self, Cartesian};
use super::style::{self, Flags, AXES, JOINT_NAMES};
use super::{FlightOptions, Summary};
use crate::{scene, Meshes, Result, RobotKind, TIMELINE};

/// Where the timeline comes from; decided on the first batch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Timebase {
    Undecided,
    /// `state.time` in seconds.
    Robot,
    /// The record index as milliseconds, for logs whose `time` never changes.
    Index,
}

/// Writes records to a stream batch by batch; see the module documentation. The 3D scene and
/// the contact estimate are in `view.rs`.
pub struct FlightLogger<'m> {
    pub(super) rec: RecordingStream,
    pub(super) model: &'m Model,
    pub(super) kind: RobotKind,
    pub(super) options: FlightOptions,
    /// The link meshes, when `options.meshes` named a directory.
    pub(super) meshes: Option<Meshes>,
    /// Whether the static parts have gone out (with the first batch).
    static_sent: bool,
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
    pub(super) summary: Summary,
    /// The baseline of `tau_ext_hat_filtered` the contact estimate is taken against.
    pub(super) tare: super::Tare,
    /// Whether the contact marker is currently in the scene (so it can be cleared).
    pub(super) contact_shown: bool,
    /// Whether the first-estimate and the at-collision event lines were written.
    pub(super) contact_reported: bool,
    pub(super) collision_contact_reported: bool,
    /// The finite differences of the sent position, for records with a Cartesian command.
    pub(super) cartesian: Cartesian,
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
    /// Finds the meshes and returns a logger ready for [`FlightLogger::log_batch`]. The static
    /// parts (series styles, the 3D base and workspace, the end effector axes, the mesh
    /// assets) go out with the first batch, so that a live viewer gets them together with
    /// the first poses.
    pub fn new(
        rec: &RecordingStream,
        model: &'m Model,
        kind: RobotKind,
        options: FlightOptions,
    ) -> Result<FlightLogger<'m>> {
        let meshes = options.meshes.as_deref().map(Meshes::find).transpose()?;
        Ok(FlightLogger {
            rec: rec.clone(),
            model,
            kind,
            options: FlightOptions {
                every: options.every.max(1),
                ..options
            },
            meshes,
            static_sent: false,
            timebase: Timebase::Undecided,
            previous: None,
            joint_contact_seen: [false; 7],
            joint_collision_seen: [false; 7],
            cartesian_contact_seen: [false; 6],
            cartesian_collision_seen: [false; 6],
            flags: None,
            last_time: 0.0,
            summary: Summary::default(),
            tare: super::Tare::default(),
            contact_shown: false,
            contact_reported: false,
            collision_contact_reported: false,
            cartesian: Cartesian::default(),
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
        if !self.static_sent {
            self.static_sent = true;
            style::log_styles(&self.rec)?;
            cartesian::log_cartesian_styles(&self.rec, self.kind)?;
            scene::log_static(&self.rec, None)?;
            if let Some(meshes) = &self.meshes {
                meshes.log_static(&self.rec)?;
            }
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
        self.log_cartesian(first, &t, records)?;
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
            self.log_contact(first + i, time, state)?;
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

    pub(super) fn event(&mut self, time: f64, level: &str, text: String) -> Result<()> {
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
