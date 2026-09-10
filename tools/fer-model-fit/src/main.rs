//! Identifies the Franka Emika Robot (FER)'s link inertial parameters from the
//! robot's own `libfcimodels_x64.so` and writes the two artefacts the crate
//! then ships: `crates/franka-rs/tests/data/fer.urdf` and
//! `crates/franka-rs/tests/data/model_reference_fer.json`.
//!
//! ```sh
//! export FRANKA_FER_MODEL_SO=/path/to/libfcimodels_x64.so
//! cargo run --manifest-path tools/fer-model-fit/Cargo.toml -- compare-fr3
//! cargo run --manifest-path tools/fer-model-fit/Cargo.toml -- fit
//! cargo run --manifest-path tools/fer-model-fit/Cargo.toml -- verify
//! cargo run --manifest-path tools/fer-model-fit/Cargo.toml -- fixture
//! ```
//!
//! Nothing here runs in CI and nothing here touches a robot; it is the
//! generator behind two tracked files, kept so the numbers can be reproduced.

mod fer;
mod fit;
mod fixture;
mod hash;
mod json;
mod report;

use std::path::{Path, PathBuf};

use franka::model::{Frame, Model};

use fixture::{load_configs, samples, GRAVITY_EARTH, GRAVITY_EARTH_ALT};
use hash::sha256_hex;
use json::{json_array, number};
use report::{compare, Row};

/// Samples drawn for the identification and for the reported comparisons.
const RANDOM_SAMPLES: usize = 200;
/// Samples written into the committed JSON fixture (which is ~2 MB at 32).
const FIXTURE_RANDOM_SAMPLES: usize = 32;
/// Seed of the sample RNG; recorded in the fixture.
const SEED: u64 = 20260905;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("the repository root exists")
}

fn data_dir() -> PathBuf {
    repo_root().join("crates/franka-rs/tests/data")
}

fn model_library_path() -> PathBuf {
    match std::env::var_os("FRANKA_FER_MODEL_SO") {
        Some(value) => PathBuf::from(value),
        None => panic!(
            "set FRANKA_FER_MODEL_SO to the path of the FER's libfcimodels_x64.so \
             (extracted from the franka-sim panda image; see tools/fer-model-fit/README.md)"
        ),
    }
}

fn so_model() -> Model {
    let path = model_library_path();
    // SAFETY: the operator of this tool pointed it at a `libfcimodels` build with
    // `FRANKA_FER_MODEL_SO`; loading and calling it is the whole point.
    unsafe { Model::from_model_library_path(&path) }
        .unwrap_or_else(|e| panic!("cannot load {}: {e}", path.display()))
}

// --------------------------------------------------------------------------
// Commands
// --------------------------------------------------------------------------

/// How far the FR3's parameters are from the FER's.
fn command_compare_fr3() {
    let so = so_model();
    let fr3_urdf = std::fs::read_to_string(data_dir().join("fr3.urdf")).expect("fr3.urdf");
    let fr3 = Model::from_urdf(&fr3_urdf).expect("fr3.urdf loads");
    let samples = samples(RANDOM_SAMPLES, SEED);

    let report = compare(&fr3, &so, &samples);
    report.print(&format!(
        "native backend with tests/data/fr3.urdf vs the FER's libfcimodels \
         ({} joint configurations x 4 load configurations)",
        samples.len()
    ));
    print!("\n{}", report.markdown());
}

/// Identifies the FER's parameters and writes `tests/data/fer.urdf`.
fn command_fit() {
    let so = so_model();
    let samples = samples(RANDOM_SAMPLES, SEED);
    let qs: Vec<[f64; 7]> = samples.iter().map(|s| s.q).collect();

    let prior = fer::franka_description_params();
    let identified = fit::identify(&so, &prior, &qs);

    println!(
        "regressor: {} rows x 70 columns, rank {}, singular values {:.3e} .. {:.3e}",
        qs.len() * 49,
        identified.rank,
        identified.singular_values.0,
        identified.singular_values.1
    );
    println!(
        "observation residual: {:.3e} (franka_description prior: {:.3e})",
        identified.residual, identified.prior_residual
    );
    println!(
        "smallest pseudo-inertia eigenvalue: {:.3e}; best reachable anywhere in the exact-fit \
         family: {:.3e} (positive would mean every link is a real rigid body)",
        identified.min_pseudo_inertia_eigenvalue, identified.best_pseudo_inertia_eigenvalue
    );

    println!("\nidentified inertial parameters (mass, com, I about the com):");
    for (i, p) in identified.params.iter().enumerate() {
        let (mass, com, i_com) = fer::com_from_params(p);
        println!(
            "  link{}: m = {:.9}  com = [{:.9}, {:.9}, {:.9}]",
            i + 1,
            mass,
            com[0],
            com[1],
            com[2]
        );
        println!(
            "          ixx = {:.9} ixy = {:.9} ixz = {:.9} iyy = {:.9} iyz = {:.9} izz = {:.9}",
            i_com[0], i_com[1], i_com[2], i_com[3], i_com[4], i_com[5]
        );
    }

    let header = format!(
        "<!--\n  \
         Franka Emika Robot (FER, FCI v5), arm only, mesh-free. Generated file: see\n  \
         `tools/fer-model-fit` and `docs/book/src/reference/model.md`; do not edit by hand.\n\n  \
         Joint origins and axes: `franka_description`'s `fer_joint1..8`, which agree with\n  \
         the robot's own `libfcimodels_x64.so` to 4e-16.\n\n  \
         `<inertial>` elements: identified from that shared object by the `fit` command of\n  \
         `tools/fer-model-fit`, as the minimum-norm correction to `franka_description`'s\n  \
         published values that reproduces the library's mass matrix and gravity torques.\n  \
         Regressor rank {} of 70 (the classical base parameters), worst residual {:.3e} over\n  \
         {} joint configurations.\n\n  \
         These ten numbers per link are a base-parameter-equivalent set, NOT a physically\n  \
         meaningful description of each casting: some moments of inertia are negative. That\n  \
         is a property of the robot's own model, not of the fit — no exact fit is physically\n  \
         consistent (the smallest reachable pseudo-inertia eigenvalue over the whole family\n  \
         of exact fits is {:.3e}). Use this file to reproduce libfranka's numbers, not to\n  \
         seed a physics simulation.\n\n  \
         Link names are libfranka's own `link0..link8`, because `franka::model::NativeBackend`\n  \
         looks for the flange under `RobotModel::kLastLinkName`.\n\
         -->\n",
        identified.rank,
        identified.residual,
        qs.len(),
        identified.best_pseudo_inertia_eigenvalue
    );
    let urdf = fer::urdf(&identified.params, &header);
    let path = data_dir().join("fer.urdf");
    std::fs::write(&path, &urdf).unwrap_or_else(|e| panic!("cannot write {}: {e}", path.display()));
    println!("\nwrote {}", path.display());

    let native = Model::from_urdf(&urdf).expect("the generated URDF loads");
    let report = compare(&native, &so, &samples);
    report.print("native FER parameters vs libfcimodels");
}

/// Reports the committed parameter set against the shared object.
fn command_verify() {
    let so = so_model();
    let urdf = std::fs::read_to_string(data_dir().join("fer.urdf")).expect("fer.urdf");
    let native = Model::from_urdf(&urdf).expect("fer.urdf loads");
    let samples = samples(RANDOM_SAMPLES, SEED);
    let report = compare(&native, &so, &samples);
    report.print(&format!(
        "tests/data/fer.urdf vs libfcimodels ({} joint configurations x 4 load configurations)",
        samples.len()
    ));
    print!("\n{}", report.markdown());
}

/// Characterises how the shared object handles the payload.
///
/// Three findings, in one place, because they are what sets the tolerances of
/// `tests/fer_native_conformance.rs`:
///
/// 1. with no payload the native parameters reproduce the library exactly;
/// 2. the payload's *rotational* inertia `I_load` enters exactly as a rigid
///    body's does, and so do `m_load` and `m_load * F_x_Cload` in `g_NE`;
/// 3. but `M_NE` is **not affine in `m_load`**, which no rigid-body model can
///    reproduce: `dM(2) - 2 dM(1)` is 7e-4 where it must be zero.
fn command_load_model() {
    let so = so_model();
    let urdf = std::fs::read_to_string(data_dir().join("fer.urdf")).expect("fer.urdf");
    let native = Model::from_urdf(&urdf).expect("fer.urdf loads");
    let samples = samples(RANDOM_SAMPLES, SEED);

    println!(
        "per load configuration, over {} joint configurations:",
        samples.len()
    );
    for config in load_configs() {
        let mut mass = Row::default();
        let mut coriolis = Row::default();
        let mut gravity = Row::default();
        for sample in &samples {
            let (i, m, c) = (&config.I_total, config.m_total, &config.F_x_Ctotal);
            mass.observe(
                &native.mass_q(&sample.q, i, m, c),
                &so.mass_q(&sample.q, i, m, c),
            );
            coriolis.observe(
                &native.coriolis_q(&sample.q, &sample.dq, i, m, c, &GRAVITY_EARTH),
                &so.coriolis_q(&sample.q, &sample.dq, i, m, c, &GRAVITY_EARTH),
            );
            gravity.observe(
                &native.gravity_q(&sample.q, m, c, &GRAVITY_EARTH),
                &so.gravity_q(&sample.q, m, c, &GRAVITY_EARTH),
            );
        }
        println!(
            "  {:32} mass {:.3e}  coriolis {:.3e}  gravity {:.3e}",
            config.name, mass.abs, coriolis.abs, gravity.abs
        );
    }

    println!("\none ingredient of the load at a time, in the mass matrix:");
    let basis: [(&str, [f64; 9]); 6] = [
        ("Ixx", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("Iyy", [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
        ("Izz", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        ("Ixy", [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("Ixz", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]),
        ("Iyz", [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0]),
    ];
    for (name, i_total) in basis {
        let mut row = Row::default();
        for sample in &samples {
            row.observe(
                &native.mass_q(&sample.q, &i_total, 0.0, &[0.0; 3]),
                &so.mass_q(&sample.q, &i_total, 0.0, &[0.0; 3]),
            );
        }
        println!("  I_load = {name} = 1, m_load = 0   {:.3e}", row.abs);
    }
    for m in [1.0_f64, 2.0, 4.0] {
        let mut row = Row::default();
        for sample in &samples {
            row.observe(
                &native.mass_q(&sample.q, &[0.0; 9], m, &[0.0; 3]),
                &so.mass_q(&sample.q, &[0.0; 9], m, &[0.0; 3]),
            );
        }
        println!("  m_load = {m}, I = 0, c = 0        {:.3e}", row.abs);
    }

    println!("\nis M_NE affine in m_load?  |dM(2) - 2 dM(1)| over the same configurations:");
    for (label, model) in [("libfcimodels", &so), ("native      ", &native)] {
        let mut worst = 0.0_f64;
        let mut worst_relative = 0.0_f64;
        for sample in &samples {
            let m0 = model.mass_q(&sample.q, &[0.0; 9], 0.0, &[0.0; 3]);
            let m1 = model.mass_q(&sample.q, &[0.0; 9], 1.0, &[0.0; 3]);
            let m2 = model.mass_q(&sample.q, &[0.0; 9], 2.0, &[0.0; 3]);
            for i in 0..49 {
                let curvature = ((m2[i] - m0[i]) - 2.0 * (m1[i] - m0[i])).abs();
                worst = worst.max(curvature);
                let scale = (m1[i] - m0[i]).abs();
                if scale > 1e-6 {
                    worst_relative = worst_relative.max(curvature / scale);
                }
            }
        }
        println!("  {label}  {worst:.3e} absolute, {worst_relative:.3e} relative");
    }

    println!("\neffective load mass the library behaves as if it had (q = ready, c = 0):");
    let q = samples[1].q;
    let n0 = native.mass_q(&q, &[0.0; 9], 0.0, &[0.0; 3]);
    let n1 = native.mass_q(&q, &[0.0; 9], 1.0, &[0.0; 3]);
    let s0 = so.mass_q(&q, &[0.0; 9], 0.0, &[0.0; 3]);
    let unit = n1[8] - n0[8];
    for m in [0.125_f64, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0] {
        let s = so.mass_q(&q, &[0.0; 9], m, &[0.0; 3]);
        println!(
            "  m_load = {m:6}  ->  {:.6} kg  (deficit {:.3e} kg)",
            (s[8] - s0[8]) / unit,
            m - (s[8] - s0[8]) / unit
        );
    }
}

/// Dumps the shared object's answers into the committed JSON fixture.
fn command_fixture() {
    let so = so_model();
    let samples = samples(FIXTURE_RANDOM_SAMPLES, SEED);
    let configs = load_configs();
    let library = model_library_path();
    let size = std::fs::metadata(&library).map(|m| m.len()).unwrap_or(0);
    let urdf = std::fs::read_to_string(data_dir().join("fer.urdf")).expect("fer.urdf");

    let mut out = String::with_capacity(4 << 20);
    out.push_str("{\n\"meta\":{");
    out.push_str("\"generator\":\"tools/fer-model-fit\",");
    out.push_str("\"source\":\"libfcimodels_x64.so served by LoadModelLibrary on FCI v5\",");
    out.push_str(&format!("\"model_library_bytes\":{size},"));
    out.push_str(&format!(
        "\"urdf_sha256\":\"{}\",",
        sha256_hex(urdf.as_bytes())
    ));
    out.push_str("\"rng\":\"splitmix64\",");
    out.push_str(&format!("\"rng_seed\":{SEED},"));
    out.push_str(&format!(
        "\"random_sample_count\":{FIXTURE_RANDOM_SAMPLES},"
    ));
    out.push_str("\"jacobian_layout\":\"6x7 column-major, rows [vx,vy,vz,wx,wy,wz]\",");
    out.push_str("\"pose_layout\":\"4x4 column-major\",");
    out.push_str(&format!(
        "\"gravity_earth\":{},",
        json_array(&GRAVITY_EARTH)
    ));
    out.push_str(&format!(
        "\"gravity_earth_alt\":{},",
        json_array(&GRAVITY_EARTH_ALT)
    ));
    out.push_str(&format!("\"q_min\":{},", json_array(&fer::Q_MIN)));
    out.push_str(&format!("\"q_max\":{},", json_array(&fer::Q_MAX)));
    out.push_str(
        "\"frames\":[\"Joint1\",\"Joint2\",\"Joint3\",\"Joint4\",\"Joint5\",\"Joint6\",\
         \"Joint7\",\"Flange\",\"EndEffector\",\"Stiffness\"]},\n",
    );

    out.push_str("\"load_configs\":[\n");
    for (i, config) in configs.iter().enumerate() {
        out.push_str(&format!(
            "{{\"name\":\"{}\",\"F_T_EE\":{},\"EE_T_K\":{},\"m_total\":{},\"F_x_Ctotal\":{},\"I_total\":{}}}",
            config.name,
            json_array(&config.F_T_EE),
            json_array(&config.EE_T_K),
            number(config.m_total),
            json_array(&config.F_x_Ctotal),
            json_array(&config.I_total)
        ));
        out.push_str(if i + 1 == configs.len() { "\n" } else { ",\n" });
    }
    out.push_str("],\n\"samples\":[\n");

    for (s, sample) in samples.iter().enumerate() {
        out.push_str(&format!(
            "{{\"index\":{},\"kind\":\"{}\",\"q\":{},\"dq\":{},\"cases\":[\n",
            sample.index,
            sample.kind,
            json_array(&sample.q),
            json_array(&sample.dq)
        ));
        for (c, config) in configs.iter().enumerate() {
            let mut pose = Vec::with_capacity(160);
            let mut body = Vec::with_capacity(420);
            let mut zero = Vec::with_capacity(420);
            for frame in Frame::ALL {
                pose.extend_from_slice(&so.pose_q(
                    frame,
                    &sample.q,
                    &config.F_T_EE,
                    &config.EE_T_K,
                ));
                body.extend_from_slice(&so.body_jacobian_q(
                    frame,
                    &sample.q,
                    &config.F_T_EE,
                    &config.EE_T_K,
                ));
                zero.extend_from_slice(&so.zero_jacobian_q(
                    frame,
                    &sample.q,
                    &config.F_T_EE,
                    &config.EE_T_K,
                ));
            }
            let mass = so.mass_q(
                &sample.q,
                &config.I_total,
                config.m_total,
                &config.F_x_Ctotal,
            );
            let coriolis = so.coriolis_q(
                &sample.q,
                &sample.dq,
                &config.I_total,
                config.m_total,
                &config.F_x_Ctotal,
                &GRAVITY_EARTH,
            );
            let gravity = so.gravity_q(
                &sample.q,
                config.m_total,
                &config.F_x_Ctotal,
                &GRAVITY_EARTH,
            );
            let gravity_alt = so.gravity_q(
                &sample.q,
                config.m_total,
                &config.F_x_Ctotal,
                &GRAVITY_EARTH_ALT,
            );
            out.push_str(&format!(
                "{{\"config\":{c},\"pose\":{},\"body_jacobian\":{},\"zero_jacobian\":{},\
                 \"mass\":{},\"coriolis\":{},\"gravity\":{},\"gravity_alt\":{}}}",
                json_array(&pose),
                json_array(&body),
                json_array(&zero),
                json_array(&mass),
                json_array(&coriolis),
                json_array(&gravity),
                json_array(&gravity_alt)
            ));
            out.push_str(if c + 1 == configs.len() { "\n" } else { ",\n" });
        }
        out.push_str("]}");
        out.push_str(if s + 1 == samples.len() { "\n" } else { ",\n" });
    }
    out.push_str("]\n}\n");

    let path = data_dir().join("model_reference_fer.json");
    std::fs::write(&path, &out).unwrap_or_else(|e| panic!("cannot write {}: {e}", path.display()));
    println!(
        "wrote {} ({} samples x {} load configurations, {:.1} MB)",
        path.display(),
        samples.len(),
        configs.len(),
        out.len() as f64 / (1 << 20) as f64
    );
}

fn main() {
    let command = std::env::args().nth(1).unwrap_or_default();
    match command.as_str() {
        "compare-fr3" => command_compare_fr3(),
        "fit" => command_fit(),
        "verify" => command_verify(),
        "load-model" => command_load_model(),
        "fixture" => command_fixture(),
        other => {
            eprintln!("usage: fer-model-fit <compare-fr3|fit|verify|diagnose|fixture>");
            if !other.is_empty() {
                eprintln!("unknown command: {other}");
            }
            std::process::exit(2);
        }
    }
}
