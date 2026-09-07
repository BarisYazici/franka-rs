//! Measured comparison of the URDF-based [`franka::model::NativeBackend`] against
//! the robot's own `libfcimodels_x64.so`, on the same poses. It reports; it does
//! not assert.

use std::path::{Path, PathBuf};

use franka::model::{Frame, Model};

use crate::fixture::*;

/// Max absolute difference per quantity, over every pose, load configuration
/// and frame.
#[derive(Default)]
struct Differences {
    pose: f64,
    body_jacobian: f64,
    zero_jacobian: f64,
    mass: f64,
    coriolis: f64,
    gravity: f64,
}

fn compare_backends(native: &Model, so: &Model) -> Differences {
    let mut d = Differences::default();
    for (_name, q, dq) in poses() {
        for config in load_configs() {
            for frame in Frame::ALL {
                d.pose = d.pose.max(max_abs_difference(
                    &native.pose_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    &so.pose_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                ));
                d.body_jacobian = d.body_jacobian.max(max_abs_difference(
                    &native.body_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    &so.body_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                ));
                d.zero_jacobian = d.zero_jacobian.max(max_abs_difference(
                    &native.zero_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    &so.zero_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                ));
            }
            d.mass = d.mass.max(max_abs_difference(
                &native.mass_q(&q, &config.I_total, config.m_total, &config.F_x_Ctotal),
                &so.mass_q(&q, &config.I_total, config.m_total, &config.F_x_Ctotal),
            ));
            d.coriolis = d.coriolis.max(max_abs_difference(
                &native.coriolis_q(
                    &q,
                    &dq,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH,
                ),
                &so.coriolis_q(
                    &q,
                    &dq,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH,
                ),
            ));
            d.gravity = d.gravity.max(max_abs_difference(
                &native.gravity_q(&q, config.m_total, &config.F_x_Ctotal, &GRAVITY_EARTH),
                &so.gravity_q(&q, config.m_total, &config.F_x_Ctotal, &GRAVITY_EARTH),
            ));
        }
    }
    d
}

/// `NativeBackend` looks for the flange link by libfranka's own name, `link8`
/// (`RobotModel::kLastLinkName`). Every FER (Franka Emika Robot) URDF in the
/// wild prefixes its links with the arm id — still `panda_`, the historical
/// prefix used upstream — so strip that prefix off the link names.
fn normalise_fer_urdf(urdf: &str) -> String {
    urdf.replace("\"panda_link", "\"link")
}

/// Removes every `<joint type="prismatic">` block.
///
/// `NativeBackend` rejects a URDF that has a movable joint off the arm chain,
/// and `franka_description`'s FER carries the two gripper fingers. libfranka
/// never sees them either: on FCI v10 the robot serves an arm-only URDF, and on
/// FCI v5 it serves this very shared object.
fn strip_prismatic_joints(urdf: &str) -> String {
    const CLOSE: &str = "</joint>";
    let mut out = String::with_capacity(urdf.len());
    let mut rest = urdf;
    while let Some(start) = rest.find("<joint") {
        let Some(tag_end) = rest[start..].find('>').map(|i| start + i) else {
            break;
        };
        let (end, block_is_element) = if rest[start..tag_end].ends_with('/') {
            (tag_end + 1, true)
        } else {
            match rest[start..].find(CLOSE) {
                Some(i) => (start + i + CLOSE.len(), true),
                None => (rest.len(), false),
            }
        };
        out.push_str(&rest[..start]);
        let block = &rest[start..end];
        if !block_is_element || !block.contains("type=\"prismatic\"") {
            out.push_str(block);
        }
        rest = &rest[end..];
    }
    out.push_str(rest);
    out
}

#[test]
fn native_urdf_backend_vs_libfcimodels() {
    let library = model_library_path();
    if !library.exists() {
        skip(&skip_message(&library));
        return;
    }
    // SAFETY: as in `so_model`.
    let so = unsafe { Model::from_model_library_path(&library) }.expect("loads");

    let mut candidates: Vec<(&str, PathBuf)> = vec![
        (
            "franka-rs tests/data/fer.urdf (built in)",
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/fer.urdf"),
        ),
        ("franka-sim panda.urdf", fer_urdf_path()),
    ];
    let franka_description = PathBuf::from(FRANKA_DESCRIPTION_FER_URDF);
    if franka_description.exists() {
        candidates.push(("franka_description panda.urdf", franka_description));
    }

    println!(
        "FER: native URDF backend vs libfcimodels -- 8 fixed joint configurations x 4 load \
         configurations, every one of the ten `Frame` values, measured against the robot's \
         own `libfcimodels_x64.so` ({} bytes) as served by `LoadModelLibrary` on FCI v5.",
        std::fs::metadata(&library).map(|m| m.len()).unwrap_or(0)
    );

    let mut measured = false;
    for (label, path) in &candidates {
        let Ok(urdf) = std::fs::read_to_string(path) else {
            println!("SKIP {label}: cannot read {}", path.display());
            continue;
        };
        let normalised = strip_prismatic_joints(&normalise_fer_urdf(&urdf));
        let native = match Model::from_urdf(&normalised) {
            Ok(model) => model,
            Err(e) => {
                println!("SKIP {label}: not usable: {e}");
                continue;
            }
        };
        let d = compare_backends(&native, &so);
        measured = true;

        println!("--- {label} ({})", path.display());
        println!("  pose           {:.3e}", d.pose);
        println!("  body_jacobian  {:.3e}", d.body_jacobian);
        println!("  zero_jacobian  {:.3e}", d.zero_jacobian);
        println!("  mass           {:.3e}", d.mass);
        println!("  coriolis       {:.3e}", d.coriolis);
        println!("  gravity        {:.3e}", d.gravity);
    }

    if !measured {
        println!(
            "SKIP: the model library is present but no FER URDF could be loaded for the \
             comparison. Set FRANKA_FER_URDF to an arm URDF whose flange link is reachable \
             to run it."
        );
    }
}
