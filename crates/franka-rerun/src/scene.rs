//! The 3D scene under `world`: the arm as a polyline through the frame origins the model
//! computes from joint angles, the end effector frame, marker points, a static trail, and
//! a fading one.

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::{Frame, Model};
use rerun::{
    Boxes3D, Color, FillMode, LineStrip3D, LineStrips3D, Mat3x3, Points3D, RecordingStream,
    Transform3D, TransformAxes3D, ViewCoordinates,
};

use crate::Result;

/// Grey of the base box and the arm, and the faint grey of the workspace box.
const BASE: u32 = 0x7070_70ff;
const ARM: u32 = 0x4a4a_4aff;
const WORKSPACE: u32 = 0x8a8a_8a30;
/// The workspace box, m: centre and half sizes. Static, so that a live viewer frames its
/// camera on the arm's reach from the first frame -- before the first batch of poses, the
/// link meshes all sit at the origin -- and not on that pile.
const WORKSPACE_CENTER: [f32; 3] = [0.0, 0.0, 0.5];
const WORKSPACE_HALF_SIZE: [f32; 3] = [0.7, 0.7, 0.5];

/// The frame origins the skeleton runs through: the base, then [`Frame::ALL`] in order.
pub const SKELETON_POINTS: usize = Frame::ALL.len() + 1;

fn f32s(v: &[f64; 3]) -> [f32; 3] {
    v.map(|x| x as f32)
}

/// Translation of a column-major 4x4 pose.
pub fn translation(pose: &[f64; 16]) -> [f64; 3] {
    [pose[12], pose[13], pose[14]]
}

/// The end effector pose for `q` and the tool offset `f_t_ee` (identity `EE_T_K`).
pub fn end_effector(model: &Model, q: &[f64; 7], f_t_ee: &[f64; 16]) -> [f64; 16] {
    model.pose_q(Frame::EndEffector, q, f_t_ee, &IDENTITY_TRANSFORM)
}

/// The tool offset `F_T_EE` that places the model's end effector at `ee_position` for `q`:
/// identity rotation, and the translation of `ee_position` in the flange frame. Identifies a
/// log's tool from one row of joint angles and measured `O_T_EE`.
pub fn tool_offset(model: &Model, q: &[f64; 7], ee_position: &[f64; 3]) -> [f64; 16] {
    let flange = model.pose_q(Frame::Flange, q, &IDENTITY_TRANSFORM, &IDENTITY_TRANSFORM);
    let d: [f64; 3] = std::array::from_fn(|k| ee_position[k] - flange[12 + k]);
    let mut f_t_ee = IDENTITY_TRANSFORM;
    for k in 0..3 {
        // Column `k` of the flange rotation is `flange[4k..4k + 3]`; `R^T d` dots it with `d`.
        f_t_ee[12 + k] = flange[4 * k] * d[0] + flange[4 * k + 1] * d[1] + flange[4 * k + 2] * d[2];
    }
    f_t_ee
}

/// The arm as points: the base origin, then the origin of every [`Frame::ALL`] frame at `q`
/// with the tool offset `f_t_ee` (identity `EE_T_K`, so the last two coincide).
pub fn skeleton(model: &Model, q: &[f64; 7], f_t_ee: &[f64; 16]) -> [[f64; 3]; SKELETON_POINTS] {
    let mut points = [[0.0; 3]; SKELETON_POINTS];
    for (point, &frame) in points[1..].iter_mut().zip(Frame::ALL.iter()) {
        *point = translation(&model.pose_q(frame, q, f_t_ee, &IDENTITY_TRANSFORM));
    }
    points
}

/// The parts that do not change: Z-up coordinates, a box for the base, a faint wireframe
/// box around the workspace, the end effector's axes, and `trail` (if any) as a faint
/// polyline at `world/{trail_name}`.
pub fn log_static(rec: &RecordingStream, trail: Option<(&str, &[[f64; 3]], u32)>) -> Result<()> {
    rec.log_static("world", &ViewCoordinates::RIGHT_HAND_Z_UP())?;
    let base = Boxes3D::from_centers_and_half_sizes([[0.0, 0.0, 0.03]], [[0.1, 0.1, 0.03]])
        .with_colors([Color::from_u32(BASE)])
        .with_fill_mode(FillMode::Solid);
    rec.log_static("world/base", &base)?;
    let workspace = Boxes3D::from_centers_and_half_sizes([WORKSPACE_CENTER], [WORKSPACE_HALF_SIZE])
        .with_colors([Color::from_u32(WORKSPACE)])
        .with_fill_mode(FillMode::MajorWireframe);
    rec.log_static("world/workspace", &workspace)?;
    rec.log_static("world/ee", &TransformAxes3D::new(0.1))?;
    if let Some((name, points, color)) = trail {
        let strip = LineStrips3D::new([LineStrip3D::from_iter(points.iter().map(f32s))])
            .with_colors([Color::from_u32(color)])
            .with_radii([0.0015]);
        rec.log_static(format!("world/{name}").as_str(), &strip)?;
    }
    Ok(())
}

/// At the stream's current time: the arm at `q` as `world/arm` and the end effector frame as
/// `world/ee`. Returns the end effector position.
pub fn log_arm(
    rec: &RecordingStream,
    model: &Model,
    q: &[f64; 7],
    f_t_ee: &[f64; 16],
) -> Result<[f64; 3]> {
    let points = skeleton(model, q, f_t_ee);
    let ee = end_effector(model, q, f_t_ee);
    log_skeleton(rec, &points, &ee)
}

/// [`log_arm`] from an already computed [`skeleton`] and end effector pose, for callers that
/// need the frame origins themselves. Returns the end effector position.
pub fn log_skeleton(
    rec: &RecordingStream,
    points: &[[f64; 3]; SKELETON_POINTS],
    ee: &[f64; 16],
) -> Result<[f64; 3]> {
    let arm = LineStrips3D::new([LineStrip3D::from_iter(points.iter().map(f32s))])
        .with_colors([Color::from_u32(ARM)])
        .with_radii([0.012]);
    rec.log("world/arm", &arm)?;
    let column = |c: usize| [ee[c] as f32, ee[c + 1] as f32, ee[c + 2] as f32];
    let position = translation(ee);
    let frame = Transform3D::from_translation(f32s(&position)).with_mat3x3(Mat3x3::from([
        column(0),
        column(4),
        column(8),
    ]));
    rec.log("world/ee", &frame)?;
    Ok(position)
}

/// At the stream's current time: `world/{name}` as several strips, each in its own colour
/// -- a trail whose segments fade with age when the caller re-logs it as time goes on.
pub fn log_strips(
    rec: &RecordingStream,
    name: &str,
    strips: &[(Vec<[f64; 3]>, u32)],
) -> Result<()> {
    let lines = LineStrips3D::new(
        strips
            .iter()
            .map(|(points, _)| LineStrip3D::from_iter(points.iter().map(f32s))),
    )
    .with_colors(strips.iter().map(|&(_, color)| Color::from_u32(color)))
    .with_radii(std::iter::repeat_n(0.003, strips.len()));
    rec.log(format!("world/{name}").as_str(), &lines)?;
    Ok(())
}

/// At the stream's current time: one marker point at `world/{name}`.
pub fn log_point(
    rec: &RecordingStream,
    name: &str,
    position: &[f64; 3],
    radius: f32,
    color: u32,
) -> Result<()> {
    let point = Points3D::new([f32s(position)])
        .with_radii([radius])
        .with_colors([Color::from_u32(color)]);
    rec.log(format!("world/{name}").as_str(), &point)?;
    Ok(())
}
