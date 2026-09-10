//! Conformance suite for the built-in Franka Emika Robot (FER) model.
//!
//! An FER has no `GetRobotModel`: libfranka 0.9.2 downloads a compiled
//! `libfcimodels_x64.so` and `dlopen`s it. franka-rs ships the same model as
//! parameters instead — [`franka::model::FER_URDF`], evaluated by the native
//! backend — so [`franka::Robot::load_model`] needs no download at all. This
//! suite is what says the two agree, and by how much.
//!
//! The oracle is `tests/data/model_reference_fer.json`, dumped from a real
//! FER's shared object by `tools/fer-model-fit` (40 joint configurations x
//! 4 load configurations x every one of the ten `Frame` values). The fixture is
//! committed, so **the whole suite runs in CI with no `.so` present**. When
//! `$FRANKA_FER_MODEL_SO` *is* set, two further tests drive the library live:
//! one checks the fixture is still what the library answers, the other pins the
//! one property of it that a rigid-body model cannot reproduce.
//!
//! | quantity | tolerance | measured |
//! |---|---|---|
//! | pose, body Jacobian, zero Jacobian, all ten frames | 1e-12 | 9e-16 |
//! | gravity, any payload | 1e-9 | 5e-14 |
//! | mass, Coriolis, no payload | 1e-9 | 4e-14 |
//! | mass, Coriolis, with a payload | characterised, see below | 3e-3 / 5e-2 |
//!
//! The last row is not a defect of the parameter set. The library's `M_NE` is
//! **not affine in `m_load`** — `dM(2 kg) - 2 dM(1 kg)` is 7e-4 where a rigid
//! payload would give exactly zero — so no rigid-body model can match it with a
//! payload attached. It behaves as though the payload were up to 18 g lighter
//! than asked for. `gravity` is unaffected, and so is every kinematic quantity.
//! See `docs/book/src/reference/model.md`.

use std::path::{Path, PathBuf};

use franka::model::{Frame, Model};

/// SHA-256, so the fixture and the URDF it was generated from cannot drift
/// apart; shared with `model_conformance.rs`.
mod sha256;

/// Poses and both Jacobians. Measured worst case: 8.882e-16.
const KINEMATIC_TOLERANCE: f64 = 1e-12;
/// Gravity at any payload, and the mass matrix and Coriolis vector with none.
/// Measured worst case: 4.974e-14.
const DYNAMIC_TOLERANCE: f64 = 1e-9;
/// The mass matrix with a payload attached. Measured worst case: 2.980e-3.
const LOADED_MASS_TOLERANCE: f64 = 1e-2;
/// The Coriolis vector with a payload attached. Measured worst case: 5.008e-2.
const LOADED_CORIOLIS_TOLERANCE: f64 = 1e-1;

// --------------------------------------------------------------------------
// Fixture
// --------------------------------------------------------------------------

#[derive(serde::Deserialize)]
struct Fixture {
    meta: Meta,
    load_configs: Vec<LoadConfig>,
    samples: Vec<Sample>,
}

#[derive(serde::Deserialize)]
struct Meta {
    urdf_sha256: String,
    rng_seed: u64,
    random_sample_count: usize,
    model_library_bytes: u64,
    gravity_earth: Vec<f64>,
    gravity_earth_alt: Vec<f64>,
    frames: Vec<String>,
}

#[allow(non_snake_case)]
#[derive(serde::Deserialize)]
struct LoadConfig {
    name: String,
    F_T_EE: Vec<f64>,
    EE_T_K: Vec<f64>,
    m_total: f64,
    F_x_Ctotal: Vec<f64>,
    I_total: Vec<f64>,
}

#[derive(serde::Deserialize)]
struct Sample {
    index: usize,
    kind: String,
    q: Vec<f64>,
    dq: Vec<f64>,
    cases: Vec<Case>,
}

#[derive(serde::Deserialize)]
struct Case {
    config: usize,
    pose: Vec<f64>,
    body_jacobian: Vec<f64>,
    zero_jacobian: Vec<f64>,
    mass: Vec<f64>,
    coriolis: Vec<f64>,
    gravity: Vec<f64>,
    gravity_alt: Vec<f64>,
}

fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn load_fixture() -> Fixture {
    let path = data_dir().join("model_reference_fer.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    serde_json::from_str(&text).expect("fixture parses")
}

fn array3(v: &[f64]) -> [f64; 3] {
    v.try_into().expect("3 elements")
}
fn array7(v: &[f64]) -> [f64; 7] {
    v.try_into().expect("7 elements")
}
fn array9(v: &[f64]) -> [f64; 9] {
    v.try_into().expect("9 elements")
}
fn array16(v: &[f64]) -> [f64; 16] {
    v.try_into().expect("16 elements")
}

// --------------------------------------------------------------------------
// Error accounting
// --------------------------------------------------------------------------

#[derive(Default, Clone)]
struct MaxError {
    value: f64,
    detail: String,
}

impl MaxError {
    fn observe(
        &mut self,
        actual: &[f64],
        expected: &[f64],
        context: &str,
        sample: usize,
        config: &str,
    ) {
        assert_eq!(actual.len(), expected.len(), "{context}: length mismatch");
        for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
            let error = (a - e).abs();
            if error > self.value {
                self.value = error;
                self.detail =
                    format!("sample {sample}, {config}, {context}, entry {i} (got {a}, want {e})");
            }
        }
    }

    fn check(&self, name: &str, tolerance: f64) {
        assert!(
            self.value <= tolerance,
            "{name}: max |error| {:.3e} > {tolerance:.0e} at {}",
            self.value,
            self.detail
        );
        println!(
            "{name}: max |error| {:.3e} (tolerance {tolerance:.0e})",
            self.value
        );
    }
}

// --------------------------------------------------------------------------
// The suite
// --------------------------------------------------------------------------

#[test]
fn fixture_matches_the_committed_urdf() {
    let fixture = load_fixture();
    let digest = sha256::hex(franka::model::FER_URDF.as_bytes());
    assert_eq!(
        digest, fixture.meta.urdf_sha256,
        "tests/data/fer.urdf no longer matches the URDF the fixture was generated from; \
         regenerate both with tools/fer-model-fit (`fit` then `fixture`)"
    );
    assert_eq!(fixture.meta.rng_seed, 20260905);
    assert_eq!(fixture.meta.random_sample_count, 32);
    assert_eq!(fixture.meta.model_library_bytes, 329_592);
    assert_eq!(fixture.samples.len(), fixture.meta.random_sample_count + 8);
    // The suite slices `pose` / `body_jacobian` / `zero_jacobian` frame-major in
    // `Frame::ALL` order, so the fixture's frame list must be exactly that order.
    assert_eq!(
        fixture.meta.frames,
        [
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
            "Joint7",
            "Flange",
            "EndEffector",
            "Stiffness"
        ]
    );
    // One unloaded configuration and three carrying the Franka Hand plus a payload.
    assert_eq!(fixture.load_configs.len(), 4);
    assert_eq!(fixture.load_configs[0].m_total, 0.0);
    assert!(fixture.load_configs[1..].iter().all(|c| c.m_total > 1.0));
    assert!(fixture.samples.iter().any(|s| s.kind == "zero"));
    assert!(fixture.samples.iter().any(|s| s.kind == "random"));
}

/// Poses and both Jacobians, at every frame, sample and load configuration.
#[test]
fn native_fer_kinematics_match_libfcimodels() {
    let fixture = load_fixture();
    let model = Model::native_fer();

    let mut pose = MaxError::default();
    let mut body = MaxError::default();
    let mut zero = MaxError::default();

    for sample in &fixture.samples {
        let q = array7(&sample.q);
        for case in &sample.cases {
            let config = &fixture.load_configs[case.config];
            let f_t_ee = array16(&config.F_T_EE);
            let ee_t_k = array16(&config.EE_T_K);
            for (f, frame) in Frame::ALL.iter().enumerate() {
                pose.observe(
                    &model.pose_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.pose[f * 16..(f + 1) * 16],
                    &format!("pose[{}]", fixture.meta.frames[f]),
                    sample.index,
                    &config.name,
                );
                body.observe(
                    &model.body_jacobian_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.body_jacobian[f * 42..(f + 1) * 42],
                    &format!("body_jacobian[{}]", fixture.meta.frames[f]),
                    sample.index,
                    &config.name,
                );
                zero.observe(
                    &model.zero_jacobian_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.zero_jacobian[f * 42..(f + 1) * 42],
                    &format!("zero_jacobian[{}]", fixture.meta.frames[f]),
                    sample.index,
                    &config.name,
                );
            }
        }
    }

    pose.check("pose", KINEMATIC_TOLERANCE);
    body.check("body_jacobian", KINEMATIC_TOLERANCE);
    zero.check("zero_jacobian", KINEMATIC_TOLERANCE);
}

/// Gravity for every payload, and the mass matrix and Coriolis vector for the
/// unloaded one: the parameters reproduce the robot's own model exactly.
#[test]
fn native_fer_dynamics_match_libfcimodels() {
    let fixture = load_fixture();
    let model = Model::native_fer();
    let gravity_earth = array3(&fixture.meta.gravity_earth);
    let gravity_earth_alt = array3(&fixture.meta.gravity_earth_alt);

    let mut mass = MaxError::default();
    let mut coriolis = MaxError::default();
    let mut gravity = MaxError::default();
    let mut gravity_alt = MaxError::default();

    for sample in &fixture.samples {
        let q = array7(&sample.q);
        let dq = array7(&sample.dq);
        for case in &sample.cases {
            let config = &fixture.load_configs[case.config];
            let i_total = array9(&config.I_total);
            let f_x_ctotal = array3(&config.F_x_Ctotal);

            // Gravity does not depend on the payload's rotational inertia, and the
            // library's `g_NE` is affine in `m_load` the way a rigid body is, so it
            // is checked at every load configuration.
            gravity.observe(
                &model.gravity_q(&q, config.m_total, &f_x_ctotal, &gravity_earth),
                &case.gravity,
                "gravity",
                sample.index,
                &config.name,
            );
            gravity_alt.observe(
                &model.gravity_q(&q, config.m_total, &f_x_ctotal, &gravity_earth_alt),
                &case.gravity_alt,
                "gravity (alternative g)",
                sample.index,
                &config.name,
            );

            if config.m_total != 0.0 {
                continue;
            }
            mass.observe(
                &model.mass_q(&q, &i_total, config.m_total, &f_x_ctotal),
                &case.mass,
                "mass",
                sample.index,
                &config.name,
            );
            coriolis.observe(
                &model.coriolis_q(
                    &q,
                    &dq,
                    &i_total,
                    config.m_total,
                    &f_x_ctotal,
                    &gravity_earth,
                ),
                &case.coriolis,
                "coriolis",
                sample.index,
                &config.name,
            );
        }
    }

    gravity.check("gravity", DYNAMIC_TOLERANCE);
    gravity_alt.check("gravity (alternative g)", DYNAMIC_TOLERANCE);
    mass.check("mass (no payload)", DYNAMIC_TOLERANCE);
    coriolis.check("coriolis (no payload)", DYNAMIC_TOLERANCE);
}

/// With a payload attached the two models part company, by a bounded amount.
///
/// This is a *characterisation*: the shared object's `M_NE` is not affine in
/// `m_load` (see [`libfcimodels_mass_matrix_is_not_affine_in_the_load_mass`]),
/// so no rigid-body model can reproduce it here, and the point of the test is
/// to hold the size of that gap still. It is about 7e-4 of the mass matrix and
/// 2e-3 of the Coriolis vector.
#[test]
fn native_fer_dynamics_with_a_payload_differ_by_a_bounded_amount() {
    let fixture = load_fixture();
    let model = Model::native_fer();
    let gravity_earth = array3(&fixture.meta.gravity_earth);

    let mut mass = MaxError::default();
    let mut coriolis = MaxError::default();
    let mut loaded_cases = 0;

    for sample in &fixture.samples {
        let q = array7(&sample.q);
        let dq = array7(&sample.dq);
        for case in &sample.cases {
            let config = &fixture.load_configs[case.config];
            if config.m_total == 0.0 {
                continue;
            }
            loaded_cases += 1;
            let i_total = array9(&config.I_total);
            let f_x_ctotal = array3(&config.F_x_Ctotal);
            mass.observe(
                &model.mass_q(&q, &i_total, config.m_total, &f_x_ctotal),
                &case.mass,
                "mass",
                sample.index,
                &config.name,
            );
            coriolis.observe(
                &model.coriolis_q(
                    &q,
                    &dq,
                    &i_total,
                    config.m_total,
                    &f_x_ctotal,
                    &gravity_earth,
                ),
                &case.coriolis,
                "coriolis",
                sample.index,
                &config.name,
            );
        }
    }

    assert!(loaded_cases > 0, "no loaded configuration in the fixture");
    mass.check("mass (with a payload)", LOADED_MASS_TOLERANCE);
    coriolis.check("coriolis (with a payload)", LOADED_CORIOLIS_TOLERANCE);
    // The gap is real, not noise: if it ever collapses to the unloaded tolerance the
    // library has changed and the loose tolerances above should be tightened.
    assert!(
        mass.value > DYNAMIC_TOLERANCE,
        "the payload gap has disappeared (mass {:.3e}); tighten the tolerances",
        mass.value
    );
}

// --------------------------------------------------------------------------
// Live against the shared object, when it is available
// --------------------------------------------------------------------------

#[cfg(feature = "model-library")]
/// The captured `libfcimodels_x64.so`, if this machine has one.
fn model_library_path() -> Option<PathBuf> {
    for name in ["FRANKA_FER_MODEL_SO", "FRANKA_FER_MODEL_LIBRARY"] {
        if let Some(value) = std::env::var_os(name) {
            let path = PathBuf::from(value);
            if path.exists() {
                return Some(path);
            }
        }
    }
    None
}

#[cfg(feature = "model-library")]
/// Writes one line to the process's *real* stderr, bypassing libtest's capture,
/// so a skipped test cannot be mistaken for coverage.
fn skip(message: &str) {
    println!("{message}");
    #[cfg(unix)]
    {
        use std::io::Write as _;
        use std::os::fd::FromRawFd as _;
        // SAFETY: fd 2 is the process's stderr, open for the whole run. The `File` is
        // wrapped in `ManuallyDrop` so it is never closed.
        let mut real_stderr = std::mem::ManuallyDrop::new(unsafe { std::fs::File::from_raw_fd(2) });
        let _ = writeln!(real_stderr, "{message}");
        let _ = real_stderr.flush();
    }
    #[cfg(not(unix))]
    eprintln!("{message}");
}

#[cfg(feature = "model-library")]
const SKIP_MESSAGE: &str = "SKIP: $FRANKA_FER_MODEL_SO is unset or missing, so this test \
     compared nothing against a live libfcimodels. The committed fixture covers the rest of \
     this suite; see docs/book/src/reference/model.md.";

/// The committed fixture is still what the library answers.
///
/// Everything else in this file trusts `model_reference_fer.json`. This is
/// the test that says the JSON was not hand-edited and did not go stale.
#[test]
#[cfg(feature = "model-library")]
fn the_committed_fixture_still_matches_the_shared_object() {
    let Some(path) = model_library_path() else {
        skip(SKIP_MESSAGE);
        return;
    };
    // SAFETY: the operator of this test pointed it at a `libfcimodels` build with
    // `FRANKA_FER_MODEL_SO`; loading and calling it is the point of the test.
    let so = unsafe { Model::from_model_library_path(&path) }
        .unwrap_or_else(|e| panic!("cannot load {}: {e}", path.display()));

    let fixture = load_fixture();
    let gravity_earth = array3(&fixture.meta.gravity_earth);
    let gravity_earth_alt = array3(&fixture.meta.gravity_earth_alt);
    let mut worst = MaxError::default();

    for sample in &fixture.samples {
        let q = array7(&sample.q);
        let dq = array7(&sample.dq);
        for case in &sample.cases {
            let config = &fixture.load_configs[case.config];
            let f_t_ee = array16(&config.F_T_EE);
            let ee_t_k = array16(&config.EE_T_K);
            let i_total = array9(&config.I_total);
            let f_x_ctotal = array3(&config.F_x_Ctotal);
            for (f, frame) in Frame::ALL.iter().enumerate() {
                worst.observe(
                    &so.pose_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.pose[f * 16..(f + 1) * 16],
                    "pose",
                    sample.index,
                    &config.name,
                );
                worst.observe(
                    &so.body_jacobian_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.body_jacobian[f * 42..(f + 1) * 42],
                    "body_jacobian",
                    sample.index,
                    &config.name,
                );
                worst.observe(
                    &so.zero_jacobian_q(*frame, &q, &f_t_ee, &ee_t_k),
                    &case.zero_jacobian[f * 42..(f + 1) * 42],
                    "zero_jacobian",
                    sample.index,
                    &config.name,
                );
            }
            worst.observe(
                &so.mass_q(&q, &i_total, config.m_total, &f_x_ctotal),
                &case.mass,
                "mass",
                sample.index,
                &config.name,
            );
            worst.observe(
                &so.coriolis_q(
                    &q,
                    &dq,
                    &i_total,
                    config.m_total,
                    &f_x_ctotal,
                    &gravity_earth,
                ),
                &case.coriolis,
                "coriolis",
                sample.index,
                &config.name,
            );
            worst.observe(
                &so.gravity_q(&q, config.m_total, &f_x_ctotal, &gravity_earth),
                &case.gravity,
                "gravity",
                sample.index,
                &config.name,
            );
            worst.observe(
                &so.gravity_q(&q, config.m_total, &f_x_ctotal, &gravity_earth_alt),
                &case.gravity_alt,
                "gravity_alt",
                sample.index,
                &config.name,
            );
        }
    }

    // The fixture stores shortest round-tripping decimals, so nothing is lost writing
    // it out and reading it back. It is still not bit-exact: the shared object's own
    // answers move by a few units in the last place between runs — up to 1.8e-14 on a
    // 30 Nm gravity torque, i.e. 3 ulps — which is what vectorised code that branches on
    // buffer alignment does. A stale or edited fixture is wrong by far more than that.
    worst.check("fixture vs the live shared object", 1e-12);
}

/// The library's mass matrix is not affine in `m_load`, and that is the whole
/// reason [`native_fer_dynamics_with_a_payload_differ_by_a_bounded_amount`]
/// has the tolerances it has.
///
/// For a rigid payload, `M(2m) - M(0)` is exactly `2 (M(m) - M(0))`. The
/// library misses that by 7e-4, behaving as though the payload were up to 18 g
/// lighter than it was told; the native backend satisfies it to 2e-15.
#[test]
#[cfg(feature = "model-library")]
fn libfcimodels_mass_matrix_is_not_affine_in_the_load_mass() {
    let Some(path) = model_library_path() else {
        skip(SKIP_MESSAGE);
        return;
    };
    // SAFETY: as above.
    let so = unsafe { Model::from_model_library_path(&path) }
        .unwrap_or_else(|e| panic!("cannot load {}: {e}", path.display()));
    let native = Model::native_fer();
    let fixture = load_fixture();

    let curvature = |model: &Model| {
        let mut worst = 0.0_f64;
        for sample in &fixture.samples {
            let q = array7(&sample.q);
            let m0 = model.mass_q(&q, &[0.0; 9], 0.0, &[0.0; 3]);
            let m1 = model.mass_q(&q, &[0.0; 9], 1.0, &[0.0; 3]);
            let m2 = model.mass_q(&q, &[0.0; 9], 2.0, &[0.0; 3]);
            for i in 0..49 {
                worst = worst.max(((m2[i] - m0[i]) - 2.0 * (m1[i] - m0[i])).abs());
            }
        }
        worst
    };

    let library = curvature(&so);
    let ours = curvature(&native);
    println!("|dM(2) - 2 dM(1)|: libfcimodels {library:.3e}, native {ours:.3e}");
    assert!(
        ours < 1e-12,
        "the native backend must be affine in m_load, got {ours:.3e}"
    );
    assert!(
        library > 1e-5,
        "libfcimodels' mass matrix has become affine in m_load ({library:.3e}); the loaded \
         tolerances of this suite can now be tightened to the unloaded ones"
    );
}
