//! The viewer layouts: the flight recorder's default, and the one for a live Cartesian
//! commander (the raw target on top of the sent and measured position per axis, the
//! derivatives against the limits).

use rerun::blueprint::components::PanelState;
use rerun::blueprint::{
    Blueprint, BlueprintActivation, BlueprintPanel, Grid, Horizontal, SelectionPanel,
    Spatial2DView, Spatial3DView, Tabs, TextLogView, TimePanel, TimeSeriesView, Vertical,
};
use rerun::RecordingStream;

use super::cartesian::{AXES, DERIVATIVES_PREFIX, POSITION_PREFIX, TARGET_PREFIX, TARGET_SPEED};
use super::{COMMANDED_ORIENTATION, ORIENTATION};
use crate::{Prefix, Result, TIMELINE};

/// What a recording's layout has to name: one set of views per robot writing into it, and the
/// timeline its time panel opens on.
///
/// A blueprint sent at all turns the viewer's automatic layout off, so an entity no view names
/// is in the file and not on screen: every arm of a shared recording has to be in here, and
/// every arm sends the same [`Layout`] so that it does not matter which one the viewer sees
/// first. Views of an arm that never ran are empty, which is better than missing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layout {
    /// The entity prefix of every robot the recording holds, in the order they are laid out.
    pub arms: Vec<Prefix>,
    /// The timeline the time panel opens on. [`crate::TIMELINE`] for one robot;
    /// [`crate::HOST_TIMELINE`] for several, the only axis on which two controllers' clocks
    /// agree.
    pub timeline: String,
}

impl Default for Layout {
    /// One unprefixed robot on `robot_time`: what a replay of one log wants.
    fn default() -> Layout {
        Layout::single(Prefix::none())
    }
}

impl Layout {
    /// One robot under `prefix`, on `robot_time`.
    pub fn single(prefix: Prefix) -> Layout {
        Layout {
            arms: vec![prefix],
            timeline: TIMELINE.to_string(),
        }
    }

    /// The same layout with the time panel on `timeline`.
    pub fn on_timeline(self, timeline: &str) -> Layout {
        Layout {
            timeline: timeline.to_string(),
            ..self
        }
    }
}

fn plot(name: &str, origin: &str) -> TimeSeriesView {
    TimeSeriesView::new(name).with_origin(origin)
}

/// One plot per axis with the raw target (when a commander logged one) over the sent and the
/// measured position -- two entities composed into one view.
fn axis_plots(prefix: &Prefix) -> [TimeSeriesView; 3] {
    AXES.map(|axis| {
        TimeSeriesView::new(prefix.label(axis))
            .with_origin("/")
            .with_contents([
                format!("+ {}", prefix.rooted(&format!("{POSITION_PREFIX}/{axis}"))),
                format!("+ {}", prefix.rooted(&format!("{TARGET_PREFIX}/{axis}"))),
            ])
    })
}

fn derivative_plots(prefix: &Prefix) -> [TimeSeriesView; 3] {
    ["speed", "acceleration", "jerk"].map(|name| {
        plot(
            &prefix.label(name),
            &prefix.path(&format!("{DERIVATIVES_PREFIX}/{name}")),
        )
    })
}

fn events(prefix: &Prefix) -> TextLogView {
    TextLogView::new(prefix.label("events")).with_origin(prefix.path("events"))
}

/// Sends `root` as the active, default blueprint, with the time panel on `timeline`.
fn send(rec: &RecordingStream, root: Vertical, timeline: &str) -> Result<()> {
    Blueprint::new(root)
        .with_time_panel(TimePanel::new().with_timeline(timeline))
        .send(rec, BlueprintActivation::default())?;
    Ok(())
}

/// One arm's views: the 3D scene on the left, with the arm's cameras behind it as a tab;
/// the joint, end effector and flag plots on the right (commanded and measured `q` share
/// one plot, the torques and the flags are tabs, the gripper is a tab of the flags, the position
/// has the measured and the sent orientation, the end effector twist, the per-axis plots and the
/// derivatives of a Cartesian command as tabs behind it); the arm's event log along the bottom.
/// Under a prefix every view's name carries it, so two arms' plots are told apart by their titles
/// and not only by their row.
///
/// The views name entities this crate does not write: `gripper/*` comes from the node that owns
/// the hand and `cam/*` from a camera node recording into the same episode. Both are under the
/// arm's prefix, so an arm's camera tab shows the cameras recording with that arm alone.
fn arm_views(prefix: &Prefix) -> Vertical {
    let q = TimeSeriesView::new(prefix.label("q vs q_d"))
        .with_origin(prefix.path("joints"))
        .with_contents(["+ $origin/q", "+ $origin/q_d", "+ $origin/q_goal"]);
    let torques = Tabs::new([
        plot(&prefix.label("tau_ext"), &prefix.path("joints/tau_ext")).into(),
        plot(&prefix.label("contact link"), &prefix.path("contact/link")).into(),
        plot(&prefix.label("tau_J"), &prefix.path("joints/tau_J")).into(),
        plot(&prefix.label("tau_J_d"), &prefix.path("joints/tau_J_d")).into(),
        plot(
            &prefix.label("tau_envelope"),
            &prefix.path("joints/tau_envelope"),
        )
        .into(),
        plot(
            &prefix.label("tau_position"),
            &prefix.path("joints/tau_position"),
        )
        .into(),
    ]);
    // The gripper shares a tab strip with the flags: a session without a hand simply leaves it
    // empty, and one with a hand has the width and the grasp beside the arm's own contacts.
    let flag = |name: &str, entity: &str| {
        plot(
            &prefix.label(name),
            &prefix.path(&format!("flags/{entity}")),
        )
        .into()
    };
    let flags = Tabs::new([
        flag("joint contact", "joint_contact"),
        flag("joint collision", "joint_collision"),
        flag("cartesian contact", "cartesian_contact"),
        flag("cartesian collision", "cartesian_collision"),
        plot(&prefix.label("gripper"), &prefix.path("gripper")).into(),
    ]);
    // `+ $origin` alone: the six-series entity, not its per-axis children.
    let all_axes =
        plot(&prefix.label("position"), &prefix.path(POSITION_PREFIX)).with_contents(["+ $origin"]);
    // `+ $origin` here too: `ee/orientation/commanded` is a child of `ee/orientation` and would
    // otherwise be drawn into the measured plot as well as into its own tab.
    let orientation =
        plot(&prefix.label("orientation"), &prefix.path(ORIENTATION)).with_contents(["+ $origin"]);
    let commanded_orientation = plot(
        &prefix.label("orientation sent"),
        &prefix.path(COMMANDED_ORIENTATION),
    );
    let ee_velocity = plot(&prefix.label("ee velocity"), &prefix.path("ee/velocity"));
    let position = Tabs::new(
        [
            all_axes.into(),
            orientation.into(),
            commanded_orientation.into(),
            ee_velocity.into(),
        ]
        .into_iter()
        .chain(axis_plots(prefix).map(Into::into))
        .chain(derivative_plots(prefix).map(Into::into)),
    );
    let velocities = Tabs::new([
        plot(&prefix.label("dq"), &prefix.path("joints/dq")).into(),
        plot(&prefix.label("dq_goal"), &prefix.path("joints/dq_goal")).into(),
        plot(&prefix.label("cap scale"), &prefix.path("joints/cap_scale")).into(),
        plot(&prefix.label("pinned"), &prefix.path("joints/pinned")).into(),
        plot(&prefix.label("ik stall"), &prefix.path("ik/stall")).into(),
        plot(&prefix.label("ik passes"), &prefix.path("ik/passes")).into(),
        plot(&prefix.label("ik step"), &prefix.path("ik/step")).into(),
        plot(&prefix.label("ik blend"), &prefix.path("ik/blend")).into(),
        plot(&prefix.label("ik held"), &prefix.path("ik/held")).into(),
        plot(&prefix.label("ik error"), &prefix.path("ik/error")).into(),
        plot(&prefix.label("leash"), &prefix.path("ee/leash")).into(),
    ]);
    let plots = Grid::new([
        q.into(),
        torques.into(),
        plot(&prefix.label("F_ext"), &prefix.path("ee/F_ext")).into(),
        position.into(),
        velocities.into(),
        flags.into(),
    ])
    .with_grid_columns(2);
    // The arm's cameras, when a camera node recorded frames under `<arm>/cam/<name>`.
    // A recording without them shows an empty tab rather than nothing at all, which is the only
    // way the arm's blueprint can make room for a file it does not write itself.
    let scene = Tabs::new([
        Spatial3DView::new(prefix.label("arm"))
            .with_origin(prefix.path("world"))
            .into(),
        Spatial2DView::new(prefix.label("cameras"))
            .with_origin(prefix.path("cam"))
            .into(),
    ]);
    let top = Horizontal::new([scene.into(), plots.into()]).with_column_shares([2.0, 3.0]);
    Vertical::new([top.into(), events(prefix).into()]).with_row_shares([4.0, 1.0])
}

/// The default layout: one arm's views (the 3D scene and the cameras, the plots, the event log)
/// for every arm of `layout`, stacked and sharing the height equally; the time panel on
/// [`Layout::timeline`].
pub fn send_blueprint(rec: &RecordingStream, layout: &Layout) -> Result<()> {
    let arms: Vec<Prefix> = if layout.arms.is_empty() {
        vec![Prefix::none()]
    } else {
        layout.arms.clone()
    };
    let root = match arms.as_slice() {
        [one] => arm_views(one),
        several => Vertical::new(
            several
                .iter()
                .map(|prefix| arm_views(prefix).with_name(prefix.name()).into()),
        ),
    };
    send(rec, root, &layout.timeline)
}

/// The layout for a live Cartesian commander, made for watching: the 3D scene over the
/// event log on the left; on the right `x`, `y`, `z` (raw target, sent, measured) and the
/// raw target's implied speed down the first column, the speed, acceleration and jerk of
/// the sent position against the limits and `F_ext` down the second; the blueprint and
/// selection panels collapsed. Send it after the recorder's own blueprint; the last one
/// sent is the one the viewer opens.
pub fn send_commander_blueprint(rec: &RecordingStream, prefix: &Prefix) -> Result<()> {
    let [x, y, z] = axis_plots(prefix);
    let [speed, acceleration, jerk] = derivative_plots(prefix);
    let plots = Grid::new([
        x.into(),
        speed.into(),
        y.into(),
        acceleration.into(),
        z.into(),
        jerk.into(),
        plot(
            &prefix.label("raw target speed"),
            &prefix.path(TARGET_SPEED),
        )
        .into(),
        plot(&prefix.label("F_ext"), &prefix.path("ee/F_ext")).into(),
    ])
    .with_grid_columns(2);
    let left = Vertical::new([
        Spatial3DView::new(prefix.label("arm"))
            .with_origin(prefix.path("world"))
            .into(),
        events(prefix).into(),
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
