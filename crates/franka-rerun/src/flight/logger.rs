//! The streaming core of the flight recorder: a [`FlightLogger`] takes batches of
//! [`Stamped`] records and writes the time series, the 3D scene and the event log for them,
//! keeping across batches whatever the next batch needs to be compared against (the previous
//! errors and mode, which flags have fired, the record index). [`super::log_records`] feeds it
//! one batch; the [`crate::Recorder`] feeds it every ~100 ms from a background thread.

use franka::{Errors, Model, Record, RobotMode, RobotState};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion};
use rerun::{RecordingStream, Scalars, TextLog, TextLogLevel, TimeColumn};

use super::cartesian::{self, Cartesian};
use super::style::{self, Flags, AXES, JOINT_NAMES};
use super::{FlightOptions, Stamped, Summary, TorqueLog, COMMANDED_ORIENTATION, ORIENTATION};
use crate::{scene, Meshes, Result, RobotKind, HOST_TIMELINE, TIMELINE};

/// Where the timeline comes from; decided on the first batch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Timebase {
    Undecided,
    /// `state.time` in seconds.
    Robot,
    /// The record index as milliseconds, for logs whose `time` never changes.
    Index,
}

/// When one row happened: the robot's clock, and the host's when the record carried it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(super) struct Stamp {
    pub(super) robot: f64,
    pub(super) host: Option<f64>,
}

impl Stamp {
    /// Puts `rec` at this time on both timelines, for the `log` calls that follow.
    pub(super) fn set(&self, rec: &RecordingStream) {
        rec.set_duration_secs(TIMELINE, self.robot);
        if let Some(host) = self.host {
            rec.set_duration_secs(HOST_TIMELINE, host);
        }
    }
}

/// A batch's times: one value per record on the robot's clock, and on the host's when the
/// records carried it (all of a batch or none, since one producer stamps them all).
pub(super) struct Times {
    pub(super) robot: Vec<f64>,
    pub(super) host: Option<Vec<f64>>,
}

impl Times {
    pub(super) fn len(&self) -> usize {
        self.robot.len()
    }

    pub(super) fn at(&self, i: usize) -> Stamp {
        Stamp {
            robot: self.robot[i],
            host: self.host.as_ref().map(|h| h[i]),
        }
    }

    /// Whether the records carried the host's clock.
    pub(super) fn has_host(&self) -> bool {
        self.host.is_some()
    }
}

/// What a `send_columns` of this crate is indexed by: [`TIMELINE`] always, and
/// [`HOST_TIMELINE`] when the rows have a host time.
pub(super) fn time_columns(robot: &[f64], host: Option<&[f64]>) -> Vec<TimeColumn> {
    let mut columns = vec![TimeColumn::new_duration_secs(
        TIMELINE,
        robot.iter().copied(),
    )];
    if let Some(host) = host {
        columns.push(TimeColumn::new_duration_secs(
            HOST_TIMELINE,
            host.iter().copied(),
        ));
    }
    columns
}

/// Writes records to a stream batch by batch; see the module documentation. The 3D scene and
/// the contact estimate are in `view.rs`.
pub struct FlightLogger<'m> {
    pub(super) rec: RecordingStream,
    pub(super) model: &'m Model,
    pub(super) kind: RobotKind,
    pub(super) options: FlightOptions,
    /// The link meshes, unless `options.meshes` is off.
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
    last: Stamp,
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
    /// The last measured, commanded (from the state) and commanded (from the command)
    /// orientation, for the sign of the next one.
    last_orientation: [Option<[f64; 4]>; 3],
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

/// The rotation of a column-major 4x4 pose as a quaternion `xyzw`, continued from `last`.
///
/// A quaternion and its negation are one rotation and the conversion may return either, which
/// a series must not do from one cycle to the next; the sign that continues `last` is taken. A
/// pose that carries no rotation at all (an all-zero `O_T_EE_c`, what a cycle with no commanded
/// pose has) is no orientation and comes back as zeros.
fn quaternion(pose: &[f64; 16], last: Option<[f64; 4]>) -> [f64; 4] {
    #[rustfmt::skip]
    let rotation = Matrix3::new(
        pose[0], pose[4], pose[8],
        pose[1], pose[5], pose[9],
        pose[2], pose[6], pose[10],
    );
    if rotation.determinant() < 0.5 {
        return [0.0; 4];
    }
    let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
    let q = [q.i, q.j, q.k, q.w];
    let dot: f64 = last.map_or(0.0, |p| q.iter().zip(&p).map(|(a, b)| a * b).sum());
    if dot < 0.0 {
        q.map(|v| -v)
    } else {
        q
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
        let meshes = options.meshes.resolve(kind)?;
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
            last: Stamp::default(),
            summary: Summary::default(),
            tare: super::Tare::default(),
            contact_shown: false,
            contact_reported: false,
            collision_contact_reported: false,
            cartesian: Cartesian::default(),
            last_orientation: [None; 3],
        })
    }

    /// An entity path of this logger: under [`FlightOptions::prefix`], when there is one.
    pub(super) fn path(&self, entity: &str) -> String {
        self.options.prefix.path(entity)
    }

    /// Fixes the timeline records go on instead of leaving it to the first batch: for a caller
    /// that has the whole log and so can see whether `state.time` ever changes over it, which
    /// one batch of it cannot answer for. `constant` puts them on the record index.
    pub(super) fn set_timebase(&mut self, constant: bool) {
        self.timebase = if constant {
            Timebase::Index
        } else {
            Timebase::Robot
        };
    }

    /// Timeline values for `records`, which start at record index `first`.
    fn times(&mut self, first: usize, records: &[Stamped]) -> Times {
        if self.timebase == Timebase::Undecided {
            let t0 = records[0].record.state.time;
            let constant = records.len() >= 2 && records.iter().all(|r| r.record.state.time == t0);
            self.timebase = if constant {
                Timebase::Index
            } else {
                Timebase::Robot
            };
        }
        let robot = records
            .iter()
            .enumerate()
            .map(|(i, r)| match self.timebase {
                Timebase::Index => (first + i) as f64 * 1e-3,
                _ => r.record.state.time.as_secs_f64(),
            })
            .collect();
        // All of a batch or none: one producer stamps every record it pushes, so a batch with
        // one stamp missing would be a bug, not a mixed batch -- and a short column is refused
        // by the SDK rather than silently misaligned.
        let host: Option<Vec<f64>> = records
            .iter()
            .map(Stamped::host_secs)
            .collect::<Option<Vec<f64>>>();
        Times { robot, host }
    }

    /// Writes `records` -- the next ones after everything logged so far -- to the stream.
    pub fn log_batch(&mut self, records: &[Stamped]) -> Result<()> {
        if records.is_empty() {
            return Ok(());
        }
        if !self.static_sent {
            self.static_sent = true;
            let prefix = self.options.prefix.clone();
            style::log_styles(&self.rec, &prefix)?;
            cartesian::log_cartesian_styles(&self.rec, &prefix, self.kind)?;
            scene::log_static(&self.rec, &prefix, None)?;
            if let Some(meshes) = &self.meshes {
                meshes.log_static(&self.rec, &prefix)?;
            }
        }
        let first = self.summary.records;
        let t = self.times(first, records);
        if first == 0 {
            self.summary.first_time = t.robot[0];
        }
        self.last = t.at(t.len() - 1);
        self.summary.last_time = self.last.robot;
        self.summary.records += records.len();

        let columns = time_columns(&t.robot, t.host.as_deref());
        self.send_series("joints/q", &columns, records, |r| r.state.q)?;
        self.send_series("joints/q_d", &columns, records, commanded_q)?;
        self.send_series("joints/dq", &columns, records, |r| r.state.dq)?;
        self.send_series("joints/tau_J", &columns, records, |r| r.state.tau_J)?;
        self.send_series("joints/tau_J_d", &columns, records, |r| r.state.tau_J_d)?;
        self.send_series("joints/tau_ext", &columns, records, |r| {
            r.state.tau_ext_hat_filtered
        })?;
        // All of a batch or none, like the host clock: one producer pushes one kind of record.
        let torque: Option<Vec<TorqueLog>> = records.iter().map(|r| r.torque).collect();
        if let Some(torque) = torque {
            fn rows<const N: usize>(
                torque: &[TorqueLog],
                f: impl Fn(&TorqueLog) -> [f64; N],
            ) -> Vec<[f64; N]> {
                torque.iter().map(f).collect()
            }
            let t = &torque;
            self.send_values("joints/q_goal", &columns, &rows(t, |t| t.q_goal))?;
            self.send_values("joints/dq_goal", &columns, &rows(t, |t| t.dq_goal))?;
            self.send_values("joints/cap_scale", &columns, &rows(t, |t| [t.cap_scale]))?;
            let pinned = rows(t, |t| t.pinned.map(f64::from));
            self.send_values("joints/pinned", &columns, &pinned)?;
            self.send_values(
                "joints/tau_envelope",
                &columns,
                &rows(t, |t| t.tau_envelope),
            )?;
            self.send_values(
                "joints/tau_position",
                &columns,
                &rows(t, |t| t.tau_position),
            )?;
            let stall = rows(t, |t| [t.stall_pressure, f64::from(u8::from(t.stalled))]);
            self.send_values("ik/stall", &columns, &stall)?;
            self.send_values("ik/passes", &columns, &rows(t, |t| [t.ik_passes.into()]))?;
            self.send_values("ee/velocity", &columns, &rows(t, |t| t.ee_velocity))?;
            self.send_values("ik/step", &columns, &rows(t, |t| t.ik_step))?;
            self.send_values("ik/blend", &columns, &rows(t, |t| [t.ik_blend]))?;
            let held = rows(t, |t| {
                let [a, b] = t.wall_age.map(f64::from);
                [f64::from(u8::from(t.held)), a, b]
            });
            self.send_values("ik/held", &columns, &held)?;
            self.send_values("ik/error", &columns, &rows(t, |t| [t.ik_error]))?;
            self.send_values("ee/leash", &columns, &rows(t, |t| t.leash))?;
        }
        self.send_series("ee/F_ext", &columns, records, |r| r.state.O_F_ext_hat_K)?;
        self.send_series("ee/position", &columns, records, |r| {
            let (m, c) = (&r.state.O_T_EE, &r.state.O_T_EE_c);
            [m[12], m[13], m[14], c[12], c[13], c[14]]
        })?;
        // Every cycle, unlike the 3D scene's pose, which `every` decimates: an exporter needs
        // the orientation at the rate the rest of the state is at.
        let orientations = self.orientations(records);
        self.send_values(ORIENTATION, &columns, &orientations)?;
        // The torque backend sends torques, so the state's `O_T_EE_c` is zero and the commanded
        // half of `ee/orientation` with it: the pose it recorded in the command goes out here.
        let commanded = self.commanded_orientations(records);
        self.send_values(COMMANDED_ORIENTATION, &columns, &commanded)?;
        self.send_series("flags/joint_contact", &columns, records, |r| {
            r.state.joint_contact
        })?;
        self.send_series("flags/joint_collision", &columns, records, |r| {
            r.state.joint_collision
        })?;
        self.send_series("flags/cartesian_contact", &columns, records, |r| {
            r.state.cartesian_contact
        })?;
        self.send_series("flags/cartesian_collision", &columns, records, |r| {
            r.state.cartesian_collision
        })?;
        self.log_cartesian(first, &t, records)?;

        for (i, record) in records.iter().enumerate() {
            let state = &record.record.state;
            let at = t.at(i);
            let force = force_norm(state);
            if force > self.summary.peak_force {
                self.summary.peak_force = force;
                self.summary.peak_force_time = at.robot;
            }
            for (joint, &tau) in state.tau_ext_hat_filtered.iter().enumerate() {
                if tau.abs() > self.summary.peak_tau_ext {
                    self.summary.peak_tau_ext = tau.abs();
                    self.summary.peak_tau_ext_joint = joint + 1;
                }
            }
            self.log_events(at, state)?;
            if (first + i).is_multiple_of(self.options.every) {
                self.log_scene(at, state)?;
            }
            self.log_contact(first + i, at, state)?;
        }
        Ok(())
    }

    /// One pose's quaternion `xyzw`, signed to continue slot `slot` of `last_orientation`.
    /// A row of zeros -- a pose that is no rotation -- does not reset the sign that is continued.
    fn continued(&mut self, slot: usize, pose: &[f64; 16]) -> [f64; 4] {
        let q = quaternion(pose, self.last_orientation[slot]);
        if q != [0.0; 4] {
            self.last_orientation[slot] = Some(q);
        }
        q
    }

    /// The measured and the commanded end effector orientation of every record, one row of two
    /// quaternions `xyzw` each, continuous across batches.
    fn orientations(&mut self, records: &[Stamped]) -> Vec<[f64; 8]> {
        records
            .iter()
            .map(|r| {
                let (measured, commanded) = (r.record.state.O_T_EE, r.record.state.O_T_EE_c);
                let m = self.continued(0, &measured);
                let c = self.continued(1, &commanded);
                [m[0], m[1], m[2], m[3], c[0], c[1], c[2], c[3]]
            })
            .collect()
    }

    /// The orientation of the pose every record's *command* carries, which the torque backend
    /// fills where `state.O_T_EE_c` is zero; zeros for a record with no command.
    fn commanded_orientations(&mut self, records: &[Stamped]) -> Vec<[f64; 4]> {
        records
            .iter()
            .map(|r| {
                let pose = r.record.command.map_or([0.0; 16], |c| c.O_T_EE_c);
                self.continued(2, &pose)
            })
            .collect()
    }

    /// One series per column of `rows`, already computed.
    fn send_values<const N: usize>(
        &self,
        entity: &str,
        columns: &[TimeColumn],
        rows: &[[f64; N]],
    ) -> Result<()> {
        let values = rows.iter().flatten().copied();
        let scalars = Scalars::new(values).columns(std::iter::repeat_n(N, rows.len()))?;
        self.rec
            .send_columns(self.path(entity), columns.to_vec(), scalars)?;
        Ok(())
    }

    /// One series of `N` values per record, computed from it.
    fn send_series<const N: usize>(
        &self,
        entity: &str,
        columns: &[TimeColumn],
        records: &[Stamped],
        f: impl Fn(&Record) -> [f64; N],
    ) -> Result<()> {
        let values = records.iter().flat_map(|r| f(&r.record));
        let scalars = Scalars::new(values).columns(std::iter::repeat_n(N, records.len()))?;
        self.rec
            .send_columns(self.path(entity), columns.to_vec(), scalars)?;
        Ok(())
    }

    pub(super) fn event(&mut self, at: Stamp, level: &str, text: String) -> Result<()> {
        at.set(&self.rec);
        self.rec
            .log(self.path("events"), &TextLog::new(text).with_level(level))?;
        self.summary.events += 1;
        Ok(())
    }

    /// Error and mode changes, and the first rising edge of every contact/collision flag.
    fn log_events(&mut self, at: Stamp, state: &RobotState) -> Result<()> {
        let (errors, mode) = (state.current_errors, state.robot_mode);
        match self.previous {
            None => {
                let kind = self.kind;
                self.event(at, TextLogLevel::INFO, format!("{kind:?}: mode {mode}"))?;
                if errors.any() {
                    let text = format!("errors at start: {errors}");
                    self.event(at, TextLogLevel::WARN, text)?;
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
                        self.event(at, TextLogLevel::WARN, text)?;
                    }
                    if !cleared.is_empty() {
                        let text = format!("errors cleared: {}", cleared.join(", "));
                        self.event(at, TextLogLevel::INFO, text)?;
                    }
                }
                if mode != last_mode {
                    self.summary.mode_changes += 1;
                    let level = if mode == RobotMode::Reflex {
                        TextLogLevel::ERROR
                    } else {
                        TextLogLevel::INFO
                    };
                    self.event(at, level, format!("mode {last_mode} -> {mode}"))?;
                }
            }
        }
        self.previous = Some((errors, mode));

        let flags = Flags::of(state);
        let last = self.flags.unwrap_or(Flags::NONE);
        self.flags = Some(flags);
        let (n, seen) = self.rising_edges(
            at,
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
            at,
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
            at,
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
            at,
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
        at: Stamp,
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
                    self.event(at, level, label(name))?;
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
            let at = self.last;
            self.event(at, TextLogLevel::ERROR, text)?;
        }
        self.rec.flush_blocking()?;
        Ok(self.summary)
    }
}

#[cfg(test)]
mod tests {
    use super::quaternion;

    /// A column-major 4x4 pose from the three columns of a rotation.
    fn pose(columns: [[f64; 3]; 3]) -> [f64; 16] {
        let mut pose = [0.0; 16];
        for (k, column) in columns.iter().enumerate() {
            pose[4 * k..4 * k + 3].copy_from_slice(column);
        }
        pose[15] = 1.0;
        pose
    }

    #[test]
    fn a_pose_becomes_a_continuous_quaternion_xyzw() {
        let identity = pose([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(quaternion(&identity, None), [0.0, 0.0, 0.0, 1.0]);
        // 90 degrees about z.
        let z90 = pose([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        let q = quaternion(&z90, None);
        let half = std::f64::consts::FRAC_1_SQRT_2;
        assert!(
            (q[2] - half).abs() < 1e-12 && (q[3] - half).abs() < 1e-12,
            "{q:?}"
        );
        // The sign that continues the previous quaternion, not the one the matrix happens to
        // give: a series must not jump between two spellings of one rotation.
        let continued = quaternion(&z90, Some([0.0, 0.0, -half, -half]));
        assert_eq!(continued, q.map(|v| -v));
        assert_eq!(quaternion(&z90, Some(q)), q);
        // A cycle with no commanded pose carries an all-zero one, which is no orientation.
        assert_eq!(quaternion(&[0.0; 16], None), [0.0; 4]);
    }
}
