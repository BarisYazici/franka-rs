//! The viewer layouts: the flight recorder's default, and the one for a live Cartesian
//! commander (the raw target on top of the sent and measured position per axis, the
//! derivatives against the limits).

use rerun::blueprint::components::PanelState;
use rerun::blueprint::{
    Blueprint, BlueprintActivation, BlueprintPanel, Grid, Horizontal, SelectionPanel,
    Spatial3DView, Tabs, TextLogView, TimePanel, TimeSeriesView, Vertical,
};
use rerun::RecordingStream;

use super::cartesian::{AXES, DERIVATIVES_PREFIX, POSITION_PREFIX, TARGET_PREFIX, TARGET_SPEED};
use crate::{Result, TIMELINE};

fn plot(name: &str, origin: &str) -> TimeSeriesView {
    TimeSeriesView::new(name).with_origin(origin)
}

/// One plot per axis with the raw target (when a commander logged one) over the sent and the
/// measured position -- two entities composed into one view.
fn axis_plots() -> [TimeSeriesView; 3] {
    AXES.map(|axis| {
        TimeSeriesView::new(axis).with_origin("/").with_contents([
            format!("+ /{POSITION_PREFIX}/{axis}"),
            format!("+ /{TARGET_PREFIX}/{axis}"),
        ])
    })
}

fn derivative_plots() -> [TimeSeriesView; 3] {
    ["speed", "acceleration", "jerk"]
        .map(|name| plot(name, &format!("{DERIVATIVES_PREFIX}/{name}")))
}

fn events() -> TextLogView {
    TextLogView::new("events").with_origin("events")
}

/// Sends `top` over the event log as the active, default blueprint, with the time panel on
/// `robot_time`.
fn send(rec: &RecordingStream, top: Horizontal) -> Result<()> {
    let root = Vertical::new([top.into(), events().into()]).with_row_shares([4.0, 1.0]);
    Blueprint::new(root)
        .with_time_panel(TimePanel::new().with_timeline(TIMELINE))
        .send(rec, BlueprintActivation::default())?;
    Ok(())
}

/// The default layout: the 3D scene on the left; the joint, end effector and flag plots on
/// the right (commanded and measured `q` share one plot, the torques and the flags are tabs,
/// the position has the per-axis plots and the derivatives of a Cartesian command as tabs
/// behind it); the event log along the bottom; the time panel on `robot_time`.
pub fn send_blueprint(rec: &RecordingStream) -> Result<()> {
    let q = TimeSeriesView::new("q vs q_d")
        .with_origin("joints")
        .with_contents(["+ $origin/q", "+ $origin/q_d"]);
    let torques = Tabs::new([
        plot("tau_ext", "joints/tau_ext").into(),
        plot("contact link", "contact/link").into(),
        plot("tau_J", "joints/tau_J").into(),
        plot("tau_J_d", "joints/tau_J_d").into(),
    ]);
    let flags = Tabs::new([
        plot("joint contact", "flags/joint_contact").into(),
        plot("joint collision", "flags/joint_collision").into(),
        plot("cartesian contact", "flags/cartesian_contact").into(),
        plot("cartesian collision", "flags/cartesian_collision").into(),
    ]);
    // `+ $origin` alone: the six-series entity, not its per-axis children.
    let all_axes = plot("position", POSITION_PREFIX).with_contents(["+ $origin"]);
    let position = Tabs::new(
        std::iter::once(all_axes.into())
            .chain(axis_plots().map(Into::into))
            .chain(derivative_plots().map(Into::into)),
    );
    let plots = Grid::new([
        q.into(),
        torques.into(),
        plot("F_ext", "ee/F_ext").into(),
        position.into(),
        plot("dq", "joints/dq").into(),
        flags.into(),
    ])
    .with_grid_columns(2);
    let top = Horizontal::new([
        Spatial3DView::new("arm").with_origin("world").into(),
        plots.into(),
    ])
    .with_column_shares([2.0, 3.0]);
    send(rec, top)
}

/// The layout for a live Cartesian commander, made for watching: the 3D scene over the
/// event log on the left; on the right `x`, `y`, `z` (raw target, sent, measured) and the
/// raw target's implied speed down the first column, the speed, acceleration and jerk of
/// the sent position against the limits and `F_ext` down the second; the blueprint and
/// selection panels collapsed. Send it after the recorder's own blueprint; the last one
/// sent is the one the viewer opens.
pub fn send_commander_blueprint(rec: &RecordingStream) -> Result<()> {
    let [x, y, z] = axis_plots();
    let [speed, acceleration, jerk] = derivative_plots();
    let plots = Grid::new([
        x.into(),
        speed.into(),
        y.into(),
        acceleration.into(),
        z.into(),
        jerk.into(),
        plot("raw target speed", TARGET_SPEED).into(),
        plot("F_ext", "ee/F_ext").into(),
    ])
    .with_grid_columns(2);
    let left = Vertical::new([
        Spatial3DView::new("arm").with_origin("world").into(),
        events().into(),
    ])
    .with_row_shares([3.0, 1.0]);
    let root = Horizontal::new([left.into(), plots.into()]).with_column_shares([2.0, 3.0]);
    Blueprint::new(root)
        .with_time_panel(TimePanel::new().with_timeline(TIMELINE))
        .with_blueprint_panel(BlueprintPanel::from_state(PanelState::Collapsed))
        .with_selection_panel(SelectionPanel::from_state(PanelState::Collapsed))
        .send(rec, BlueprintActivation::default())?;
    Ok(())
}
