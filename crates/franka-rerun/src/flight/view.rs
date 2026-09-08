//! The 3D half of the flight recorder: the arm (skeleton, joint spheres, the meshes when
//! there are any), the external force arrow from the end effector, and the contact estimate
//! -- a marker and a force arrow at the point [`contact::estimate`] finds, the winning link
//! as the `contact/link` series, and one event line at the first estimate and at the
//! collision.

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::RobotState;
use rerun::{Arrows3D, Clear, Color, Points3D, Scalars, TextLogLevel};

use super::contact;
use super::logger::FlightLogger;
use super::style::{Flags, COLLISION, CONTACT, FORCE, JOINT_NAMES, QUIET};
use crate::{scene, Result, TIMELINE};

/// Sphere radius at a joint, m, plus this much per Nm of `|tau_ext_hat_filtered|`.
const JOINT_RADIUS: f64 = 0.02;
const JOINT_RADIUS_PER_NM: f64 = 0.0015;
/// Contact marker radius, m, plus this much per newton of the estimated force.
const CONTACT_RADIUS: f64 = 0.012;
const CONTACT_RADIUS_PER_N: f64 = 0.001;

pub(super) const ESTIMATE_ENTITY: &str = "world/contact/estimate";
pub(super) const FORCE_ENTITY: &str = "world/contact/force";
pub(super) const LINK_ENTITY: &str = "contact/link";

impl FlightLogger<'_> {
    /// The arm, the joint spheres, the meshes and the external force arrow at `time`.
    pub(super) fn log_scene(&self, time: f64, state: &RobotState) -> Result<()> {
        self.rec.set_duration_secs(TIMELINE, time);
        let points = scene::skeleton(self.model, &state.q, &state.F_T_EE);
        let ee_pose = self.model.pose_q(
            franka::Frame::EndEffector,
            &state.q,
            &state.F_T_EE,
            &IDENTITY_TRANSFORM,
        );
        let ee = scene::log_skeleton(&self.rec, &points, &ee_pose)?;
        if let Some(meshes) = &self.meshes {
            meshes.log_poses(&self.rec, self.model, &state.q)?;
        }
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
        let color = if flags.any_cartesian_collision() {
            COLLISION
        } else if flags.any_cartesian_contact() {
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

    /// The contact estimate for record `index` at `time`, from the torques less the [`Tare`]
    /// baseline: computed when a flag is set, or when a torque exceeds the noise floor and the
    /// record is one the 3D scene draws (`every`); cleared from the scene when it goes away.
    ///
    /// [`Tare`]: super::Tare
    pub(super) fn log_contact(
        &mut self,
        index: usize,
        time: f64,
        state: &RobotState,
    ) -> Result<()> {
        let flags = Flags::of(state);
        let floor = self.options.contact.noise_floor;
        let tau = self
            .tare
            .apply(&state.tau_ext_hat_filtered, !flags.any(), floor);
        let loud = tau.iter().any(|t| t.abs() >= floor);
        let due = flags.any() || (loud && index.is_multiple_of(self.options.every));
        let estimate = if due {
            contact::estimate(
                self.model,
                &state.q,
                &tau,
                &state.F_T_EE,
                &self.options.contact,
            )
        } else {
            None
        };
        let Some(estimate) = estimate else {
            if self.contact_shown {
                self.rec.set_duration_secs(TIMELINE, time);
                self.rec.log(ESTIMATE_ENTITY, &Clear::flat())?;
                self.rec.log(FORCE_ENTITY, &Clear::flat())?;
                self.contact_shown = false;
            }
            return Ok(());
        };

        self.rec.set_duration_secs(TIMELINE, time);
        let magnitude = estimate.magnitude();
        let color = Color::from_u32(if flags.any_collision() {
            COLLISION
        } else if flags.any_contact() {
            CONTACT
        } else {
            FORCE
        });
        let point = estimate.point.map(|v| v as f32);
        let marker = Points3D::new([point])
            .with_radii([(CONTACT_RADIUS + CONTACT_RADIUS_PER_N * magnitude) as f32])
            .with_colors([color])
            .with_labels([format!("link {} {:.1} N", estimate.link, magnitude)])
            .with_show_labels(true);
        self.rec.log(ESTIMATE_ENTITY, &marker)?;
        let scale = self.options.force_scale;
        let vector = estimate.force.map(|v| (v * scale) as f32);
        let arrow = Arrows3D::from_vectors([vector])
            .with_origins([point])
            .with_colors([color])
            .with_radii([0.005]);
        self.rec.log(FORCE_ENTITY, &arrow)?;
        self.rec
            .log(LINK_ENTITY, &Scalars::single(estimate.link as f64))?;
        self.contact_shown = true;
        self.summary.contact_estimates += 1;

        if !self.contact_reported {
            self.contact_reported = true;
            let text = format!("estimated {estimate}");
            self.event(time, TextLogLevel::INFO, text)?;
        }
        if flags.any_collision() && !self.collision_contact_reported {
            self.collision_contact_reported = true;
            let text = format!("at the collision: estimated {estimate}");
            self.event(time, TextLogLevel::ERROR, text)?;
        }
        self.summary.last_contact = Some(estimate);
        Ok(())
    }
}
