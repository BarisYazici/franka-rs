//! The arm drawn with Franka's own link meshes instead of (as well as) the skeleton.
//!
//! [`Meshes::find`] looks in a directory for `link0 .. link7`, `hand` and `finger` with any
//! extension the viewer loads (`glb`, `gltf`, `obj`, `stl`, or the original Collada `dae`;
//! `tools/franka-meshes/convert.py` makes the `glb`s from franka_description). Each is logged
//! once, statically, as an [`Asset3D`] at `world/links/<name>/mesh`, and every record gets a
//! [`Transform3D`] on `world/links/<name>`: the pose of that link's frame in the base frame.
//!
//! # Frames
//!
//! franka_description's `link_k` frame is the child frame of `joint_k`, i.e. the frame after
//! joint `k`'s rotation, and its joint origins (`robots/<robot>/kinematics.yaml`) are the ones
//! in this crate's `tests/data/{fer,fr3}.urdf`; `NativeBackend` composes `origin * Rot(axis,
//! q_k)` per joint like Pinocchio and the description's own FK do, so [`Model::pose_q`] of
//! `Frame::JointK` *is* the `link_k` frame, and `link0` is the base. Every arm visual in the
//! description has an identity `<origin>` and no `<scale>`, so the meshes hang directly on
//! those frames; `link7`'s mesh ends at `z = 0.1068` in its frame, the flange being at 0.107.
//! The hand is mounted as the description mounts it: on the flange, yawed by [`HAND_YAW`]
//! (`rpy_ee = "0 0 -pi/4"`), with the fingers [`FINGER_Z`] further along `z` (closed: a control
//! log carries no gripper width) and the right one turned by pi. With the Franka Hand's
//! `F_T_EE`, the end effector then sits 0.1034 m along the hand's `z`.

use std::path::{Path, PathBuf};

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::{Frame, Model};
use rerun::{Asset3D, Mat3x3, RecordingStream, Transform3D};

use crate::Result;

/// Where the meshes live in the entity tree.
pub const ENTITY: &str = "world/links";
/// Extensions tried for each mesh, in order of preference.
pub const EXTENSIONS: [&str; 5] = ["glb", "gltf", "obj", "stl", "dae"];
/// Yaw of the hand frame about the flange's `z`, rad (franka_description's `rpy_ee`).
pub const HAND_YAW: f64 = -std::f64::consts::FRAC_PI_4;
/// The finger frames' offset along the hand's `z`, m (`finger_joint1`'s origin).
pub const FINGER_Z: f64 = 0.0584;

/// The mesh files found for one robot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Meshes {
    /// `link0.glb` .. `link7.glb` (or another extension), where present.
    pub links: [Option<PathBuf>; 8],
    pub hand: Option<PathBuf>,
    pub finger: Option<PathBuf>,
}

fn find_one(dir: &Path, name: &str) -> Option<PathBuf> {
    EXTENSIONS
        .iter()
        .map(|ext| dir.join(format!("{name}.{ext}")))
        .find(|path| path.is_file())
}

impl Meshes {
    /// The meshes in `dir`; an error when it holds no link mesh at all.
    pub fn find(dir: &Path) -> Result<Meshes> {
        let links: [Option<PathBuf>; 8] =
            std::array::from_fn(|k| find_one(dir, &format!("link{k}")));
        if links.iter().all(Option::is_none) {
            return Err(format!(
                "{}: no link0..link7 mesh (.glb/.gltf/.obj/.stl/.dae); see tools/franka-meshes",
                dir.display()
            )
            .into());
        }
        Ok(Meshes {
            links,
            hand: find_one(dir, "hand"),
            finger: find_one(dir, "finger"),
        })
    }

    /// What was found, for printing: `8 link meshes, hand, fingers`.
    pub fn describe(&self) -> String {
        let links = self.links.iter().flatten().count();
        let mut text = format!("{links} link mesh(es)");
        if self.hand.is_some() {
            text.push_str(", hand");
        }
        if self.finger.is_some() {
            text.push_str(", fingers");
        }
        text
    }

    /// The assets, the base link's identity pose and the fingers' fixed offsets, all static.
    pub fn log_static(&self, rec: &RecordingStream) -> Result<()> {
        for (k, path) in self.links.iter().enumerate() {
            if let Some(path) = path {
                let entity = format!("{ENTITY}/link{k}");
                log_asset(rec, &entity, path)?;
                if k == 0 {
                    rec.log_static(entity.as_str(), &transform(&IDENTITY_TRANSFORM))?;
                }
            }
        }
        if let Some(path) = &self.hand {
            log_asset(rec, &format!("{ENTITY}/hand"), path)?;
        }
        if let Some(path) = &self.finger {
            for (name, yaw) in [("finger_left", 0.0), ("finger_right", std::f64::consts::PI)] {
                let entity = format!("{ENTITY}/hand/{name}");
                let mut pose = IDENTITY_TRANSFORM;
                pose[14] = FINGER_Z;
                rec.log_static(entity.as_str(), &transform(&yawed(&pose, yaw)))?;
                log_asset(rec, &entity, path)?;
            }
        }
        Ok(())
    }

    /// At the stream's current time: the pose of every moving link and of the hand for `q`.
    pub fn log_poses(&self, rec: &RecordingStream, model: &Model, q: &[f64; 7]) -> Result<()> {
        for k in 1..8 {
            if self.links[k].is_some() {
                let pose = link_pose(model, q, k);
                rec.log(format!("{ENTITY}/link{k}").as_str(), &transform(&pose))?;
            }
        }
        if self.hand.is_some() {
            let pose = hand_pose(model, q);
            rec.log(format!("{ENTITY}/hand").as_str(), &transform(&pose))?;
        }
        Ok(())
    }
}

fn log_asset(rec: &RecordingStream, entity: &str, path: &Path) -> Result<()> {
    let asset = Asset3D::from_file_path(path).map_err(|e| format!("{}: {e}", path.display()))?;
    rec.log_static(format!("{entity}/mesh").as_str(), &asset)?;
    Ok(())
}

/// The pose of `link_k`'s frame in the base frame: identity for the base, `Frame::JointK`
/// for `1..=7`.
pub fn link_pose(model: &Model, q: &[f64; 7], k: usize) -> [f64; 16] {
    match k {
        0 => IDENTITY_TRANSFORM,
        _ => model.pose_q(
            Frame::ALL[k.clamp(1, 7) - 1],
            q,
            &IDENTITY_TRANSFORM,
            &IDENTITY_TRANSFORM,
        ),
    }
}

/// The hand frame: the flange yawed by [`HAND_YAW`].
pub fn hand_pose(model: &Model, q: &[f64; 7]) -> [f64; 16] {
    let flange = model.pose_q(Frame::Flange, q, &IDENTITY_TRANSFORM, &IDENTITY_TRANSFORM);
    yawed(&flange, HAND_YAW)
}

/// `pose * Rz(yaw)`, column-major.
pub fn yawed(pose: &[f64; 16], yaw: f64) -> [f64; 16] {
    let (s, c) = yaw.sin_cos();
    let mut out = *pose;
    for r in 0..3 {
        out[r] = c * pose[r] + s * pose[4 + r];
        out[4 + r] = -s * pose[r] + c * pose[4 + r];
    }
    out
}

/// A column-major 4x4 pose as a Rerun transform (translation and rotation matrix).
pub fn transform(pose: &[f64; 16]) -> Transform3D {
    let column = |c: usize| [pose[c] as f32, pose[c + 1] as f32, pose[c + 2] as f32];
    Transform3D::from_translation_mat3x3(
        column(12),
        Mat3x3::from([column(0), column(4), column(8)]),
    )
}
