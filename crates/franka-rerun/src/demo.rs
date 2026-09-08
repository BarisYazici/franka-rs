//! The demo layout for a commander replay, made for a screen capture: the arm with its
//! meshes at full height on the left, with a fading trail of the measured path; on the
//! right, per axis, the raw signal -- the user's target staircase -- against the processed
//! one that went to the robot, then the speed of both (the raw one shoots off the top at
//! every step) and the acceleration and jerk of the processed one, under the commander's
//! red budget lines; no event log, the blueprint and selection panels hidden, the time
//! panel playing on a loop.
//!
//! The SDK's blueprint builders (`rerun::blueprint::TimeSeriesView` and friends) carry no
//! setter for a view's own properties -- the y-axis range of a plot, the 3D eye -- so this
//! layout is assembled from the blueprint archetypes directly, on a blueprint stream of its
//! own, the way the builders do it inside: every view at `view/<uuid>`, its properties at
//! `view/<uuid>/<Archetype>`, every container at `container/<uuid>`, the root in
//! `viewport`, and the messages handed to the recording with an activation command.

use rerun::blueprint::components::{
    BackgroundKind, ColumnShare, ContainerKind, Corner2D, Eye3DKind, IncludedContent,
    LockRangeDuringZoom, LoopMode, PanelState, PlayState, QueryExpression, RootContainer, RowShare,
    TimelineName, ViewClass,
};
use rerun::components::{Color, Name, Position3D, Range1D, Vector3D};
use rerun::encodings::{Bool, Float32};
use rerun::external::re_log_types::BlueprintActivationCommand;
use rerun::external::re_sdk_types::blueprint::archetypes::{
    Background, ContainerBlueprint, EyeControls3D, PanelBlueprint, PlotLegend, ScalarAxis,
    TimePanelBlueprint, ViewBlueprint, ViewContents, ViewportBlueprint,
};
use rerun::external::uuid::Uuid;
use rerun::log::LogMsg;
use rerun::{AsComponents, RecordingStream, RecordingStreamBuilder};

use crate::series::Limits;
use crate::{Result, TIMELINE};

/// Width of the 3D view as a share of the window; the plots take the rest.
const SCENE_SHARE: f32 = 55.0;
/// The derivative plots' y range as a multiple of the budget line, so the sent command
/// visibly sits under it.
const HEADROOM: f64 = 1.25;
/// The eye of the 3D view: where it stands and what it looks at, m in the base frame. A
/// three-quarter front view from the robot's right (`-y`), a little ahead of it: the base
/// at the lower left, the arm's side profile with the wrist and hand towards the right,
/// the target and the measured trail in front of the hand, the whole arm inside the view
/// with some margin at the top.
const EYE_POSITION: [f32; 3] = [0.62, -0.95, 0.68];
const EYE_LOOK_AT: [f32; 3] = [0.20, -0.06, 0.40];
/// The 3D view's background: a solid near-black, the demo animation's ground.
const BACKGROUND: u32 = 0x0b0f_14ff;

/// Sends the demo layout as the active, default blueprint. `budget` is what the
/// `processed/*` limit lines were logged with; it fixes the derivative plots' y ranges.
pub fn send_blueprint(rec: &RecordingStream, budget: &Limits) -> Result<()> {
    let app_id = rec
        .store_info()
        .map(|info| info.application_id().clone())
        .ok_or("the recording has no store info")?;
    let (bp, storage) = RecordingStreamBuilder::new(app_id).blueprint().memory()?;
    bp.set_time_sequence("blueprint", 0);

    let scene = view(
        &bp,
        "3D",
        "arm",
        "world",
        &[
            "+ $origin/**",
            "- $origin/arm",
            "- $origin/workspace",
            "- $origin/measured",
            "- $origin/measured_path",
            "- $origin/ee",
        ],
        &[
            (
                "EyeControls3D",
                &EyeControls3D::new()
                    .with_kind(Eye3DKind::Orbital)
                    .with_position(Position3D::from(EYE_POSITION))
                    .with_look_target(Position3D::from(EYE_LOOK_AT))
                    .with_eye_up(Vector3D::from([0.0, 0.0, 1.0])),
            ),
            (
                "Background",
                &Background::new(BackgroundKind::SolidColor)
                    .with_color(Color::from_u32(BACKGROUND)),
            ),
        ],
    )?;
    // The legends in the top right corner, off the first steps.
    let legend = PlotLegend::new().with_corner(Corner2D::RightTop);
    let mut plots = Vec::new();
    for axis in ["x", "y", "z"] {
        let (raw, processed) = (format!("+ /raw/{axis}"), format!("+ /processed/{axis}"));
        plots.push(view(
            &bp,
            "TimeSeries",
            &format!("{axis}: raw vs processed"),
            "/",
            &[raw.as_str(), processed.as_str()],
            &[("PlotLegend", &legend)],
        )?);
    }
    let lines = [
        ("speed", budget.speed),
        ("acceleration", budget.acceleration),
        ("jerk", budget.jerk),
    ];
    // The raw spikes only in the speed panel: they clutter the other two.
    for (name, limit) in lines {
        let range = axis_range(0.0, limit * HEADROOM);
        let (raw, processed) = (format!("+ /raw/{name}"), format!("+ /processed/{name}/**"));
        let (title, contents): (String, &[&str]) = if name == "speed" {
            (
                format!("{name}: raw vs processed vs limit"),
                &[raw.as_str(), processed.as_str()],
            )
        } else {
            (format!("{name}: processed vs limit"), &[processed.as_str()])
        };
        plots.push(view(
            &bp,
            "TimeSeries",
            &title,
            "/",
            contents,
            &[("ScalarAxis", &range), ("PlotLegend", &legend)],
        )?);
    }
    let column = container(&bp, ContainerKind::Vertical, &plots, None)?;
    let root = container(
        &bp,
        ContainerKind::Horizontal,
        &[scene, format!("container/{column}")],
        Some([SCENE_SHARE, 100.0 - SCENE_SHARE]),
    )?;

    let viewport = ViewportBlueprint::new()
        .with_root_container(RootContainer(root.into()))
        .with_auto_layout(Bool(false))
        .with_auto_views(Bool(false));
    bp.log("viewport", &viewport)?;
    let hidden = PanelBlueprint::new().with_state(PanelState::Hidden);
    bp.log("blueprint_panel", &hidden)?;
    bp.log("selection_panel", &hidden)?;
    // Collapsed: the play controls and the timeline, not the streams tree. Playing, and
    // looping the whole run, so the viewer never sits at the end.
    let time_panel = TimePanelBlueprint::new()
        .with_state(PanelState::Collapsed)
        .with_timeline(TimelineName(TIMELINE.into()))
        .with_play_state(PlayState::Playing)
        .with_loop_mode(LoopMode::All);
    bp.log("time_panel", &time_panel)?;

    let msgs = storage.take();
    let blueprint_id = msgs
        .iter()
        .find_map(|msg| match msg {
            LogMsg::SetStoreInfo(info) => Some(info.info.store_id.clone()),
            _ => None,
        })
        .ok_or("the blueprint stream produced no store info")?;
    let activation = BlueprintActivationCommand {
        blueprint_id,
        make_active: true,
        make_default: true,
    };
    rec.send_blueprint(msgs, activation);
    Ok(())
}

/// A fixed y range, locked against zooming.
fn axis_range(start: f64, end: f64) -> ScalarAxis {
    ScalarAxis::new()
        .with_range(Range1D::new(start, end))
        .with_zoom_lock(LockRangeDuringZoom(Bool(true)))
}

/// One view of `class` (`"3D"`, `"TimeSeries"`) named `name`, rooted at `origin`, with
/// the entity queries `contents` and the view `properties` (archetype short name, value).
/// Returns its blueprint path.
fn view(
    bp: &RecordingStream,
    class: &str,
    name: &str,
    origin: &str,
    contents: &[&str],
    properties: &[(&str, &dyn AsComponents)],
) -> Result<String> {
    let path = format!("view/{}", Uuid::new_v4());
    let queries = contents.iter().map(|q| QueryExpression((*q).into()));
    bp.log(format!("{path}/ViewContents"), &ViewContents::new(queries))?;
    let blueprint = ViewBlueprint::new(ViewClass(class.into()))
        .with_display_name(Name(name.into()))
        .with_space_origin(origin);
    bp.log(path.as_str(), &blueprint)?;
    for (archetype, value) in properties {
        bp.log_serialized_batches(
            format!("{path}/{archetype}"),
            false,
            value.as_serialized_batches(),
        )?;
    }
    Ok(path)
}

/// A container of `kind` over `children` (blueprint paths); `shares` splits a horizontal
/// one's width or a vertical one's height. Returns its id.
fn container(
    bp: &RecordingStream,
    kind: ContainerKind,
    children: &[String],
    shares: Option<[f32; 2]>,
) -> Result<Uuid> {
    let id = Uuid::new_v4();
    let mut blueprint = ContainerBlueprint::new(kind)
        .with_contents(children.iter().map(|c| IncludedContent(c.as_str().into())));
    if let Some(shares) = shares {
        let shares = shares.map(Float32);
        blueprint = match kind {
            ContainerKind::Horizontal => blueprint.with_col_shares(shares.map(ColumnShare)),
            _ => blueprint.with_row_shares(shares.map(RowShare)),
        };
    }
    bp.log(format!("container/{id}"), &blueprint)?;
    Ok(id)
}
