//! Offline conformance suite for the FCI v5 (Franka Emika Robot, FER) model library.
//!
//! On FCI v5 the robot serves a compiled `libfcimodels_x64.so` through the
//! `LoadModelLibrary` command instead of a URDF (libfranka 0.9.2
//! `src/library_downloader.cpp`, `src/model_library.cpp`, `src/model.cpp`).
//! This suite drives that shared object through
//! [`franka::model::Model::from_model_library_path`] /
//! [`franka::model::Model::from_model_library_bytes`] and checks the invariants
//! `franka::Model` guarantees, at eight fixed joint configurations: the zero
//! pose, the libfranka "ready" pose, and six corners of the FER joint box.
//!
//! No simulator and no robot are needed. The library is *not* committed: it is
//! read from the capture the FER simulator serves,
//! `$FRANKA_FER_MODEL_SO` (or the older `$FRANKA_FER_MODEL_LIBRARY`, or the
//! path below), and every test skips with a `SKIP:` message when it is absent --
//! printed past libtest's output capture, and summarised by
//! `model_library_fixture_present_or_skipped_loudly`, so a run that asserted
//! nothing cannot be mistaken for coverage.
//!
//! The last test additionally *measures* the URDF-based
//! [`franka::model::NativeBackend`] against the same library on the same poses
//! and prints the per-quantity max absolute differences. It reports; it does
//! not assert.

//! The whole suite needs the default-on `model-library` feature (the
//! `libloading` backend); with `--no-default-features` it compiles to nothing.
#![cfg(feature = "model-library")]

use franka::model::{Frame, Model};

mod conformance;
#[path = "fer_model_conformance/fixture.rs"]
mod fixture;
#[path = "fer_model_conformance/native.rs"]
mod native;

use fixture::{
    is_positive_definite, load_configs, mat4_mul, max_abs_difference, model_library_path, poses,
    skip, skip_message, translation_z, with_so_model, FLANGE_OFFSET_Z, GRAVITY_EARTH, IDENTITY,
};

/// Reports, loudly, whether this run actually exercised the model library.
///
/// Every other test in this file returns early when the captured
/// `libfcimodels_x64.so` is absent, and libtest does not show what a passing
/// test printed — so without this the whole suite is green either way and there
/// is no way to tell coverage from a no-op. This test never fails: it either
/// asserts the fixture loads (and that the suite therefore ran for real), or it
/// prints one `SKIP:` line straight to the process's stderr, past libtest's
/// capture, naming `FRANKA_FER_MODEL_SO`.
#[test]
fn model_library_fixture_present_or_skipped_loudly() {
    let path = model_library_path();
    if !path.exists() {
        skip(&skip_message(&path));
        return;
    }
    // SAFETY: as in `so_model` — this is the fixture the operator pointed the suite at.
    let model = unsafe { Model::from_model_library_path(&path) }
        .unwrap_or_else(|e| panic!("cannot load {}: {e}", path.display()));
    // Touch the library so "present" also means "loadable and callable".
    let q_ready = poses()[1].1;
    let mass = model.mass_q(&q_ready, &[0.0; 9], 0.0, &[0.0; 3]);
    assert!(
        mass[0].is_finite() && mass[0] > 0.0,
        "{}: M_NE(q_ready)[0,0] = {}, expected a finite positive inertia",
        path.display(),
        mass[0]
    );
    println!("model library fixture: {}", path.display());
}

// --------------------------------------------------------------------------
// Invariants
// --------------------------------------------------------------------------

#[test]
fn mass_matrix_is_symmetric_and_positive_definite() {
    with_so_model!(|model| {
        for (name, q, _dq) in poses() {
            for config in load_configs() {
                let mass = model.mass_q(&q, &config.I_total, config.m_total, &config.F_x_Ctotal);
                for i in 0..7 {
                    for j in 0..7 {
                        let asymmetry = (mass[i + 7 * j] - mass[j + 7 * i]).abs();
                        assert!(
                            asymmetry < 1e-12,
                            "{name}/{}: mass[{i}][{j}] asymmetric by {asymmetry:e}",
                            config.name
                        );
                    }
                }
                assert!(
                    is_positive_definite(&mass),
                    "{name}/{}: mass matrix is not positive definite: {mass:?}",
                    config.name
                );
            }
        }
    })
}

#[test]
fn gravity_is_finite_and_loads_the_expected_joints() {
    with_so_model!(|model| {
        for (name, q, _dq) in poses() {
            for config in load_configs() {
                let g = model.gravity_q(&q, config.m_total, &config.F_x_Ctotal, &GRAVITY_EARTH);
                for (joint, value) in g.iter().enumerate() {
                    assert!(
                        value.is_finite(),
                        "{name}/{}: gravity[{joint}] = {value}",
                        config.name
                    );
                }
            }
        }

        // At the "ready" pose the arm hangs out of the vertical, so the three pitch joints
        // 2, 4 and 6 (1-based) must carry a non-zero gravity torque.
        let ready = poses()[1].1;
        let g = model.gravity_q(&ready, 0.0, &[0.0; 3], &GRAVITY_EARTH);
        for joint in [2usize, 4, 6] {
            assert!(
                g[joint - 1].abs() > 1e-3,
                "ready pose: gravity[{joint}] = {} is ~zero",
                g[joint - 1]
            );
        }
    })
}

#[test]
fn coriolis_vanishes_at_zero_joint_velocity() {
    with_so_model!(|model| {
        for (name, q, _dq) in poses() {
            for config in load_configs() {
                let c = model.coriolis_q(
                    &q,
                    &[0.0; 7],
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH,
                );
                for (joint, value) in c.iter().enumerate() {
                    assert!(
                        value.abs() < 1e-12,
                        "{name}/{}: coriolis[{joint}] = {value:e} at dq = 0",
                        config.name
                    );
                }
            }
        }
    })
}

#[test]
fn flange_is_joint7_translated_along_its_own_z_axis() {
    with_so_model!(|model| {
        for (name, q, _dq) in poses() {
            let joint7 = model.pose_q(Frame::Joint7, &q, &IDENTITY, &IDENTITY);
            let flange = model.pose_q(Frame::Flange, &q, &IDENTITY, &IDENTITY);
            let expected = mat4_mul(&joint7, &translation_z(FLANGE_OFFSET_Z));
            let error = max_abs_difference(&flange, &expected);
            assert!(
                error < 1e-9,
                "{name}: flange != joint7 * trans_z({FLANGE_OFFSET_Z}), max error {error:e}\n\
                 got  {flange:?}\nwant {expected:?}"
            );
        }
    })
}

#[test]
fn end_effector_and_stiffness_frames_are_the_flange_post_multiplied() {
    with_so_model!(|model| {
        for (name, q, _dq) in poses() {
            let flange = model.pose_q(Frame::Flange, &q, &IDENTITY, &IDENTITY);
            for config in load_configs() {
                let ee = model.pose_q(Frame::EndEffector, &q, &config.F_T_EE, &config.EE_T_K);
                let expected_ee = mat4_mul(&flange, &config.F_T_EE);
                let error = max_abs_difference(&ee, &expected_ee);
                assert!(
                    error < 1e-12,
                    "{name}/{}: EE != flange * F_T_EE, max error {error:e}",
                    config.name
                );

                let k = model.pose_q(Frame::Stiffness, &q, &config.F_T_EE, &config.EE_T_K);
                let expected_k = mat4_mul(&expected_ee, &config.EE_T_K);
                let error = max_abs_difference(&k, &expected_k);
                assert!(
                    error < 1e-12,
                    "{name}/{}: K != flange * F_T_EE * EE_T_K, max error {error:e}",
                    config.name
                );
            }
        }

        // The non-identity offsets are what make the two checks above meaningful.
        assert_ne!(load_configs()[1].F_T_EE, IDENTITY);
        assert_ne!(load_configs()[2].EE_T_K, IDENTITY);
    })
}

#[test]
fn at_least_one_load_configuration_has_a_non_commuting_stiffness_offset() {
    // `Frame::Stiffness` is evaluated at the column-major product `F_T_EE * EE_T_K`
    // (`so_backend.rs`, `pose_stiffness` / `*_jacobian_stiffness`). If every fixture's
    // two transforms commuted, swapping that product order would leave the whole suite
    // green while putting stiffness-frame poses and Jacobians wrong on hardware, so pin
    // that the fixture actually distinguishes the two orders.
    let mut worst = 0.0_f64;
    for config in load_configs() {
        let forward = mat4_mul(&config.F_T_EE, &config.EE_T_K);
        let reversed = mat4_mul(&config.EE_T_K, &config.F_T_EE);
        worst = worst.max(max_abs_difference(&forward, &reversed));
    }
    assert!(
        worst > 1e-3,
        "no load configuration distinguishes F_T_EE * EE_T_K from EE_T_K * F_T_EE \
         (worst difference {worst:e}); the stiffness-frame product order in \
         src/model/so_backend.rs is then untested"
    );
}

#[test]
fn zero_jacobian_is_the_body_jacobian_rotated_into_the_base_frame() {
    with_so_model!(|model| {
        for (name, q, _dq) in poses() {
            for config in load_configs() {
                for frame in Frame::ALL {
                    let pose = model.pose_q(frame, &q, &config.F_T_EE, &config.EE_T_K);
                    let body = model.body_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K);
                    let zero = model.zero_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K);

                    // zero = blkdiag(R, R) * body, with R the frame's rotation in the base.
                    let mut error = 0.0_f64;
                    for column in 0..7 {
                        for block in 0..2 {
                            for row in 0..3 {
                                let mut expected = 0.0;
                                for k in 0..3 {
                                    expected +=
                                        pose[row + 4 * k] * body[6 * column + 3 * block + k];
                                }
                                let got = zero[6 * column + 3 * block + row];
                                error = error.max((got - expected).abs());
                            }
                        }
                    }
                    assert!(
                        error < 1e-9,
                        "{name}/{}/{frame:?}: zero != R * body, max error {error:e}",
                        config.name
                    );
                }
            }
        }
    })
}

#[test]
fn joint1_jacobians_are_constant() {
    // `Ji_J_J1` and `O_J_J1` take no `q` at all (`src/libfcimodels.h`, 0.9.2), so the
    // backend must not pass one; the observable consequence is that joint 1's Jacobians
    // are the same at every configuration.
    with_so_model!(|model| {
        let reference_body = model.body_jacobian_q(Frame::Joint1, &[0.0; 7], &IDENTITY, &IDENTITY);
        let reference_zero = model.zero_jacobian_q(Frame::Joint1, &[0.0; 7], &IDENTITY, &IDENTITY);
        for (name, q, _dq) in poses() {
            let body = model.body_jacobian_q(Frame::Joint1, &q, &IDENTITY, &IDENTITY);
            let zero = model.zero_jacobian_q(Frame::Joint1, &q, &IDENTITY, &IDENTITY);
            assert_eq!(
                body, reference_body,
                "{name}: body Jacobian of joint 1 moved"
            );
            assert_eq!(
                zero, reference_zero,
                "{name}: zero Jacobian of joint 1 moved"
            );
        }
    })
}

#[test]
fn loading_from_bytes_matches_loading_from_the_file() {
    let path = model_library_path();
    if !path.exists() {
        skip(&skip_message(&path));
        return;
    }
    let bytes = std::fs::read(&path).expect("readable");
    // SAFETY: as in `so_model` — the fixture is a `libfcimodels` build this test was
    // deliberately pointed at, and the bytes are that same file read back.
    let (from_path, from_bytes) = unsafe {
        (
            Model::from_model_library_path(&path).expect("loads from path"),
            Model::from_model_library_bytes(&bytes).expect("loads from bytes"),
        )
    };

    for (name, q, dq) in poses() {
        for config in load_configs() {
            for frame in Frame::ALL {
                assert_eq!(
                    from_path.pose_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    from_bytes.pose_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    "{name}/{}/{frame:?}: pose",
                    config.name
                );
                assert_eq!(
                    from_path.body_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    from_bytes.body_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    "{name}/{}/{frame:?}: body Jacobian",
                    config.name
                );
                assert_eq!(
                    from_path.zero_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    from_bytes.zero_jacobian_q(frame, &q, &config.F_T_EE, &config.EE_T_K),
                    "{name}/{}/{frame:?}: zero Jacobian",
                    config.name
                );
            }
            assert_eq!(
                from_path.mass_q(&q, &config.I_total, config.m_total, &config.F_x_Ctotal),
                from_bytes.mass_q(&q, &config.I_total, config.m_total, &config.F_x_Ctotal),
                "{name}/{}: mass",
                config.name
            );
            assert_eq!(
                from_path.coriolis_q(
                    &q,
                    &dq,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH
                ),
                from_bytes.coriolis_q(
                    &q,
                    &dq,
                    &config.I_total,
                    config.m_total,
                    &config.F_x_Ctotal,
                    &GRAVITY_EARTH
                ),
                "{name}/{}: coriolis",
                config.name
            );
            assert_eq!(
                from_path.gravity_q(&q, config.m_total, &config.F_x_Ctotal, &GRAVITY_EARTH),
                from_bytes.gravity_q(&q, config.m_total, &config.F_x_Ctotal, &GRAVITY_EARTH),
                "{name}/{}: gravity",
                config.name
            );
        }
    }
}
