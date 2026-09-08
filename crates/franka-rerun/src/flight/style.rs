//! Colours, legend names and the per-record flag decoding shared by the flight recorder's
//! series and its 3D scene.

use franka::RobotState;
use rerun::{Color, RecordingStream, SeriesLines};

use crate::Result;

/// `0xRRGGBBAA`: amber for a contact, red for a collision, grey for neither.
pub const CONTACT: u32 = 0xf5a6_23ff;
pub const COLLISION: u32 = 0xd62_728ff;
pub const QUIET: u32 = 0x8a8a_8aff;
/// The external force arrow while no Cartesian flag is set.
pub(super) const FORCE: u32 = 0x2a78_d6ff;

/// One distinct colour per joint (Tableau 10, first seven).
const JOINTS: [u32; 7] = [
    0x1f77_b4ff,
    0xff7f_0eff,
    0x2ca0_2cff,
    0xd627_28ff,
    0x9467_bdff,
    0x8c56_4bff,
    0xe377_c2ff,
];
/// Measured `x`, `y`, `z` and, paler, the commanded ones.
const POSITION: [u32; 6] = [
    0xd627_28ff,
    0x2ca0_2cff,
    0x1f77_b4ff,
    0xff98_96ff,
    0x98df_8aff,
    0xaec7_e8ff,
];

pub(super) const JOINT_NAMES: [&str; 7] = [
    "joint 1", "joint 2", "joint 3", "joint 4", "joint 5", "joint 6", "joint 7",
];
const WRENCH_NAMES: [&str; 6] = [
    "Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]",
];
pub(super) const AXES: [&str; 6] = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"];
const POSITION_NAMES: [&str; 6] = ["x", "y", "z", "x_c", "y_c", "z_c"];

/// The four flag arrays of a state as booleans (the robot sends 0.0 / 1.0).
#[derive(Debug, Clone, Copy)]
pub(super) struct Flags {
    pub(super) joint_contact: [bool; 7],
    pub(super) joint_collision: [bool; 7],
    pub(super) cartesian_contact: [bool; 6],
    pub(super) cartesian_collision: [bool; 6],
}

impl Flags {
    /// Nothing set: what the record before the first one counts as.
    pub(super) const NONE: Flags = Flags {
        joint_contact: [false; 7],
        joint_collision: [false; 7],
        cartesian_contact: [false; 6],
        cartesian_collision: [false; 6],
    };

    pub(super) fn of(state: &RobotState) -> Flags {
        Flags {
            joint_contact: state.joint_contact.map(|v| v > 0.5),
            joint_collision: state.joint_collision.map(|v| v > 0.5),
            cartesian_contact: state.cartesian_contact.map(|v| v > 0.5),
            cartesian_collision: state.cartesian_collision.map(|v| v > 0.5),
        }
    }

    pub(super) fn any_cartesian_contact(&self) -> bool {
        self.cartesian_contact.iter().any(|&c| c)
    }

    pub(super) fn any_cartesian_collision(&self) -> bool {
        self.cartesian_collision.iter().any(|&c| c)
    }

    /// Any contact flag, joint or Cartesian.
    pub(super) fn any_contact(&self) -> bool {
        self.joint_contact.iter().any(|&c| c) || self.any_cartesian_contact()
    }

    /// Any collision flag, joint or Cartesian.
    pub(super) fn any_collision(&self) -> bool {
        self.joint_collision.iter().any(|&c| c) || self.any_cartesian_collision()
    }

    pub(super) fn any(&self) -> bool {
        self.any_contact() || self.any_collision()
    }
}

/// `n` shades of `base`, the first the base itself, the rest blended towards white.
fn shades(base: u32, n: usize) -> Vec<Color> {
    (0..n)
        .map(|i| {
            let f = i as f32 / n as f32 * 0.6;
            let channel = |shift: u32| {
                let c = ((base >> shift) & 0xff) as f32;
                (c + (255.0 - c) * f).round() as u8
            };
            Color::from_rgb(channel(24), channel(16), channel(8))
        })
        .collect()
}

fn palette(colors: &[u32]) -> Vec<Color> {
    colors.iter().map(|&c| Color::from_u32(c)).collect()
}

/// The `SeriesLines` style -- legend names, colours, widths -- of every series entity.
pub(super) fn log_styles(rec: &RecordingStream) -> Result<()> {
    let joints = palette(&JOINTS);
    let styles: [(&str, &[&str], Vec<Color>); 13] = [
        ("joints/q", &JOINT_NAMES, joints.clone()),
        ("joints/q_d", &JOINT_NAMES, joints.clone()),
        ("joints/dq", &JOINT_NAMES, joints.clone()),
        ("joints/tau_J", &JOINT_NAMES, joints.clone()),
        ("joints/tau_J_d", &JOINT_NAMES, joints.clone()),
        ("joints/tau_ext", &JOINT_NAMES, joints),
        ("ee/F_ext", &WRENCH_NAMES, palette(&POSITION)),
        ("ee/position", &POSITION_NAMES, palette(&POSITION)),
        ("flags/joint_contact", &JOINT_NAMES, shades(CONTACT, 7)),
        ("flags/joint_collision", &JOINT_NAMES, shades(COLLISION, 7)),
        ("flags/cartesian_contact", &AXES, shades(CONTACT, 6)),
        ("flags/cartesian_collision", &AXES, shades(COLLISION, 6)),
        ("contact/link", &["link"], palette(&[FORCE])),
    ];
    for (entity, names, colors) in styles {
        let style = SeriesLines::new()
            .with_names(names.iter().copied())
            .with_colors(colors)
            .with_widths(std::iter::repeat_n(1.5, names.len()));
        rec.log_static(entity, &style)?;
    }
    Ok(())
}
