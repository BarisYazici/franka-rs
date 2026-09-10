//! The FER fixture: where the captured `libfcimodels_x64.so` comes from, the
//! eight joint configurations and four load configurations the suite drives it
//! with, and the column-major linear algebra the invariants are written in.

use std::path::{Path, PathBuf};

use franka::model::Model;

/// Where the suite looks for the FER model library when no environment variable
/// names one.
///
/// The library is the `libfcimodels_x64.so` an FER serves over `LoadModelLibrary`;
/// it is not redistributable and therefore not committed. Capture it from a robot
/// (or from the FER simulator image) and point `FRANKA_FER_MODEL_SO` at it, or
/// drop it at this path; every test skips loudly when it is absent. See
/// `docs/book/src/reference/model.md`.
pub const DEFAULT_MODEL_LIBRARY: &str = "target/fer-fixture/libfcimodels_x64.so";

/// Where the suite looks for a mesh-free FER URDF when `FRANKA_FER_URDF` is
/// unset; used only for the measured `NativeBackend` comparison.
pub const DEFAULT_FER_URDF: &str = "target/fer-fixture/fer.urdf";

/// `franka_description`'s FER, as shipped by `example-robot-data`. Optional:
/// unlike the simulator's URDF it carries `<inertial>` elements, so it is the
/// only one of the two that can be compared on dynamics at all.
///
/// The path itself is external and still uses the upstream package's older
/// "panda" naming (`panda_description`, `panda.urdf`), so it is left exactly
/// as installed rather than rewritten to match this crate's own naming.
pub const FRANKA_DESCRIPTION_FER_URDF: &str =
    "/opt/openrobots/share/example-robot-data/robots/panda_description/urdf/panda.urdf";

/// Distance from the joint-7 frame to the flange frame along the joint-7 z
/// axis (`panda_joint8` in the external `franka_description` package above:
/// `xyz="0 0 0.107"`).
pub const FLANGE_OFFSET_Z: f64 = 0.107;

/// The captured `libfcimodels_x64.so` to drive: `$FRANKA_FER_MODEL_SO`, else
/// the older `$FRANKA_FER_MODEL_LIBRARY`, else [`DEFAULT_MODEL_LIBRARY`].
pub fn model_library_path() -> PathBuf {
    for name in ["FRANKA_FER_MODEL_SO", "FRANKA_FER_MODEL_LIBRARY"] {
        if let Some(value) = std::env::var_os(name) {
            return PathBuf::from(value);
        }
    }
    PathBuf::from(DEFAULT_MODEL_LIBRARY)
}

/// Writes one line to the process's *real* stderr, bypassing libtest's output
/// capture.
///
/// Every test in this file is a no-op when the fixture is absent, and a plain
/// `cargo test` (CI included) prints nothing a passing test wrote — which is
/// how nine green-but-vacuous results could look like coverage. This makes the
/// `SKIP:` lines show up without `--nocapture`.
pub fn skip(message: &str) {
    println!("{message}");
    #[cfg(unix)]
    {
        use std::io::Write as _;
        use std::os::fd::FromRawFd as _;
        // SAFETY: fd 2 is the process's stderr, open for the whole run. The `File` is
        // wrapped in `ManuallyDrop` so it is never closed, and writing to it is what any
        // other stderr user does.
        let mut real_stderr = std::mem::ManuallyDrop::new(unsafe { std::fs::File::from_raw_fd(2) });
        let _ = writeln!(real_stderr, "{message}");
        let _ = real_stderr.flush();
    }
    #[cfg(not(unix))]
    eprintln!("{message}");
}

/// The message every skip prints, so the reason and the override are in one place.
pub fn skip_message(path: &Path) -> String {
    format!(
        "SKIP: the captured FER model library is not present at {}. \
         Set FRANKA_FER_MODEL_SO to a libfcimodels_x64.so to run the FCI v5 model \
         conformance suite (see docs/book/src/reference/model.md).",
        path.display()
    )
}

pub fn fer_urdf_path() -> PathBuf {
    match std::env::var_os("FRANKA_FER_URDF") {
        Some(value) => PathBuf::from(value),
        None => PathBuf::from(DEFAULT_FER_URDF),
    }
}

/// Loads the captured model library, or `None` (with a printed reason) when it
/// is not on this machine.
pub fn so_model() -> Option<Model> {
    let path = model_library_path();
    if !path.exists() {
        skip(&skip_message(&path));
        return None;
    }
    // SAFETY: the file named by `$FRANKA_FER_MODEL_SO` (or the default capture) is a
    // `libfcimodels` build the operator of this test deliberately pointed it at; running
    // its code is the whole point of the suite.
    let model = unsafe { Model::from_model_library_path(&path) }
        .unwrap_or_else(|e| panic!("cannot load {}: {e}", path.display()));
    Some(model)
}

/// Runs `body` with the captured library, or skips.
macro_rules! with_so_model {
    (|$model:ident| $body:block) => {
        match crate::fixture::so_model() {
            None => return,
            Some($model) => $body,
        }
    };
}

pub(crate) use with_so_model;

// --------------------------------------------------------------------------
// Fixture: eight joint configurations and four load configurations
// --------------------------------------------------------------------------

/// FER joint limits (`franka_description`'s `fer_joint*` `<limit>` bounds,
/// which is also what `include/franka/robot.h` 0.9.2 documents).
pub const Q_MIN: [f64; 7] = [
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
];
pub const Q_MAX: [f64; 7] = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];

/// The eight configurations, mirroring the fixed part of the FR3 fixture
/// (`tools/model-reference/main.cpp`: the zero pose, the "ready" pose and six
/// joint-limit corners) with the corners made deterministic — one fixed bit
/// pattern per corner instead of the generator's RNG — and clamped to the
/// FER's own limits, which differ from the FR3's on joints 4 and 6.
pub fn poses() -> Vec<(String, [f64; 7], [f64; 7])> {
    use std::f64::consts::PI;

    let mut out: Vec<(String, [f64; 7], [f64; 7])> = Vec::with_capacity(8);
    out.push(("zero".to_string(), [0.0; 7], [0.0; 7]));
    out.push((
        "ready".to_string(),
        [
            0.0,
            -PI / 4.0,
            0.0,
            -3.0 * PI / 4.0,
            0.0,
            PI / 2.0,
            PI / 4.0,
        ],
        [1.0; 7],
    ));

    // Six corners of the joint box; bit j of the mask picks the upper limit of joint j.
    const MASKS: [u8; 6] = [
        0b1010101, 0b0101010, 0b1100110, 0b0011001, 0b1111000, 0b0000111,
    ];
    for (corner, mask) in MASKS.iter().enumerate() {
        let mut q = [0.0; 7];
        let mut dq = [0.0; 7];
        for j in 0..7 {
            let upper = mask & (1 << j) != 0;
            q[j] = if upper { Q_MAX[j] } else { Q_MIN[j] };
            // A deterministic, modest velocity, alternating in sign.
            dq[j] = if upper { 0.5 } else { -0.5 };
        }
        out.push((format!("limit_corner_{corner}"), q, dq));
    }
    out
}

pub const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// A load configuration: the flange offsets and the *combined* inertial
/// properties `franka::Model` is called with (`I_total`, `m_total`,
/// `F_x_Ctotal`), which is what the library's `_load` arguments actually
/// receive.
#[allow(non_snake_case)]
pub struct LoadConfig {
    pub name: &'static str,
    pub F_T_EE: [f64; 16],
    pub EE_T_K: [f64; 16],
    pub m_total: f64,
    pub F_x_Ctotal: [f64; 3],
    pub I_total: [f64; 9],
}

pub fn load_configs() -> Vec<LoadConfig> {
    // Franka Hand flange -> EE transform, column-major: -45 degrees about z and
    // 0.1034 m along z (`franka_description`'s `fer_hand_tcp`).
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let hand: [f64; 16] = [
        s, -s, 0.0, 0.0, s, s, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1034, 1.0,
    ];
    // Stiffness frame: 30 degrees about z and 0.05 m along z, as in the FR3 fixture.
    let (s30, c30) = (30.0_f64).to_radians().sin_cos();
    let stiffness: [f64; 16] = [
        c30, s30, 0.0, 0.0, -s30, c30, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.05, 1.0,
    ];
    // A stiffness frame that does **not** commute with `hand`: 30 degrees about x
    // (`hand` is a screw about z, so the two rotation axes differ) plus an off-axis
    // translation. Without this, `F_T_EE * EE_T_K` and `EE_T_K * F_T_EE` are equal to
    // 5.6e-17 and a swapped product order in the backend passes the whole suite.
    let tilted_stiffness: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 0.0, c30, s30, 0.0, 0.0, -s30, c30, 0.0, 0.02, 0.0, 0.05, 1.0,
    ];

    vec![
        LoadConfig {
            name: "no_load",
            F_T_EE: IDENTITY,
            EE_T_K: IDENTITY,
            m_total: 0.0,
            F_x_Ctotal: [0.0; 3],
            I_total: [0.0; 9],
        },
        LoadConfig {
            name: "hand_and_load",
            F_T_EE: hand,
            EE_T_K: IDENTITY,
            // Franka Hand (0.73 kg) plus a 0.5 kg payload, combined the way
            // `Robot::Impl::convertRobotState` does.
            m_total: 1.23,
            F_x_Ctotal: [-0.0018699187, 0.0081300813, 0.03],
            I_total: [0.0031, 0.0, 0.0, 0.0, 0.0046, 0.0, 0.0, 0.0, 0.0049],
        },
        LoadConfig {
            name: "hand_load_and_stiffness",
            F_T_EE: hand,
            EE_T_K: stiffness,
            m_total: 1.23,
            F_x_Ctotal: [-0.0018699187, 0.0081300813, 0.03],
            I_total: [0.0031, 0.0, 0.0, 0.0, 0.0046, 0.0, 0.0, 0.0, 0.0049],
        },
        LoadConfig {
            name: "hand_load_and_tilted_stiffness",
            F_T_EE: hand,
            EE_T_K: tilted_stiffness,
            m_total: 1.23,
            F_x_Ctotal: [-0.0018699187, 0.0081300813, 0.03],
            I_total: [0.0031, 0.0, 0.0, 0.0, 0.0046, 0.0, 0.0, 0.0, 0.0049],
        },
    ]
}

pub const GRAVITY_EARTH: [f64; 3] = [0.0, 0.0, -9.81];

// --------------------------------------------------------------------------
// Small linear-algebra helpers (column-major, like the library's own output)
// --------------------------------------------------------------------------

pub fn mat4_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0; 16];
    for column in 0..4 {
        for row in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[row + 4 * k] * b[k + 4 * column];
            }
            out[row + 4 * column] = sum;
        }
    }
    out
}

pub fn translation_z(distance: f64) -> [f64; 16] {
    let mut out = IDENTITY;
    out[14] = distance;
    out
}

pub fn max_abs_difference(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0_f64, f64::max)
}

/// `true` when the symmetric 7x7 column-major `mass` has a Cholesky
/// factorisation, i.e. is positive definite.
pub fn is_positive_definite(mass: &[f64; 49]) -> bool {
    let mut l = [0.0_f64; 49];
    for j in 0..7 {
        for i in j..7 {
            let mut sum = mass[i + 7 * j];
            for k in 0..j {
                sum -= l[i + 7 * k] * l[j + 7 * k];
            }
            if i == j {
                // NaN and non-positive pivots both mean "not positive definite".
                if !sum.is_finite() || sum <= 0.0 {
                    return false;
                }
                l[i + 7 * j] = sum.sqrt();
            } else {
                l[i + 7 * j] = sum / l[j + 7 * j];
            }
        }
    }
    true
}
