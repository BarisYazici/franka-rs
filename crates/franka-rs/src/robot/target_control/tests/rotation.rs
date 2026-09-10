//! The rotation arithmetic: rotation vectors, quaternions, matrices and the checked inputs.

use nalgebra::Matrix3;

use super::super::rotation::{
    angle_between, checked_pose, exp, from_quaternion, log, norm, orthonormality_error, pose_from,
    to_quaternion, unit_quaternion,
};
use super::is_invalid_argument;

#[test]
fn rotation_vectors_quaternions_and_matrices_round_trip() {
    let mut rng = crate::otg::tests::Rng(0x2545_F491_4F6C_DD1D);
    for _ in 0..500 {
        let v = [
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
        ];
        let angle = rng.uniform(1e-9, 3.1);
        let v = v.map(|x| x * angle / norm(&v));
        let r = exp(&v);
        assert!(orthonormality_error(&r) < 1e-12);
        let back = log(&r);
        assert!(
            norm(&[back[0] - v[0], back[1] - v[1], back[2] - v[2]]) < 1e-9,
            "{v:?} -> {back:?}"
        );
        let q = to_quaternion(&r);
        assert!(q[3] >= 0.0 && (norm(&[q[0], q[1], q[2]]).hypot(q[3]) - 1.0).abs() < 1e-12);
        assert!(angle_between(&r, &from_quaternion(&q)) < 1e-12);
        assert!((angle_between(&Matrix3::identity(), &r) - angle).abs() < 1e-9);
    }
    assert_eq!(log(&Matrix3::identity()), [0.0; 3]);
    assert_eq!(exp(&[0.0; 3]), Matrix3::identity());
    assert_eq!(to_quaternion(&Matrix3::identity()), [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn poses_and_quaternions_are_repaired_within_tolerance_and_refused_beyond() {
    let r = exp(&[0.3, -0.2, 0.5]);
    let pose = pose_from(&r, &[0.4, 0.0, 0.5]);
    let (position, checked) = checked_pose(&pose).unwrap();
    assert_eq!(position, [0.4, 0.0, 0.5]);
    assert!(angle_between(&r, &checked) < 1e-12);

    let scaled = |factor: f64| pose_from(&(r * factor), &[0.4, 0.0, 0.5]);
    let (_, repaired) = checked_pose(&scaled(1.0003)).unwrap();
    assert!(orthonormality_error(&repaired) < 1e-12 && angle_between(&r, &repaired) < 1e-9);
    assert!(is_invalid_argument(
        checked_pose(&scaled(1.01)).map(drop),
        "not orthonormal"
    ));
    let mut reflected = pose;
    reflected[0..3].iter_mut().for_each(|x| *x = -*x);
    assert!(is_invalid_argument(
        checked_pose(&reflected).map(drop),
        "not orthonormal"
    ));
    let mut bad_row = pose;
    bad_row[3] = 0.1;
    assert!(is_invalid_argument(
        checked_pose(&bad_row).map(drop),
        "last row"
    ));
    let mut nan = pose;
    nan[5] = f64::NAN;
    assert!(is_invalid_argument(checked_pose(&nan).map(drop), "finite"));

    assert_eq!(
        unit_quaternion([0.0, 0.0, 0.0, 1.0005]).unwrap(),
        [0.0, 0.0, 0.0, 1.0]
    );
    assert!(is_invalid_argument(
        unit_quaternion([0.0, 0.0, 0.0, 2.0]).map(drop),
        "[x, y, z, w]"
    ));
    assert!(is_invalid_argument(
        unit_quaternion([f64::INFINITY, 0.0, 0.0, 1.0]).map(drop),
        "finite"
    ));
}
