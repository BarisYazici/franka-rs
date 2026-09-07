//! First-order low-pass filters for control signals.
//!
//! Port of libfranka 0.21.2 `include/franka/lowpass_filter.h` and `src/lowpass_filter.cpp`.

use nalgebra::{Matrix3, Matrix4, Quaternion, Rotation3, UnitQuaternion};

use crate::error::{FrankaError, FrankaResult};
use crate::math_utils::{linear_of, orthonormalized_rotation, translation_of};

/// Maximum cutoff frequency. Port of `franka::kMaxCutoffFrequency`.
pub const MAX_CUTOFF_FREQUENCY: f64 = 1000.0;
/// Default cutoff frequency. Port of `franka::kDefaultCutoffFrequency`.
pub const DEFAULT_CUTOFF_FREQUENCY: f64 = 100.0;

/// Applies a first-order low-pass filter.
///
/// Port of `franka::lowpassFilter`.
///
/// * `sample_time` – sample time constant
/// * `y` – current value of the signal to be filtered
/// * `y_last` – value of the signal in the previous time step
/// * `cutoff_frequency` – cutoff frequency of the low-pass filter
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `sample_time` is negative, infinite or NaN, if
/// `cutoff_frequency` is zero, negative, infinite or NaN, or if either signal value is infinite
/// or NaN.
pub fn low_pass_filter(
    sample_time: f64,
    y: f64,
    y_last: f64,
    cutoff_frequency: f64,
) -> FrankaResult<f64> {
    if sample_time < 0.0 || !sample_time.is_finite() {
        return Err(FrankaError::InvalidArgument(
            "lowpass-filter: sample_time is negative, infinite or NaN.".to_string(),
        ));
    }
    if cutoff_frequency <= 0.0 || !cutoff_frequency.is_finite() {
        return Err(FrankaError::InvalidArgument(
            "lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN.".to_string(),
        ));
    }
    if !y.is_finite() || !y_last.is_finite() {
        return Err(FrankaError::InvalidArgument(
            "lowpass-filter: current or past input value of the signal to be filtered is infinite \
             or NaN."
                .to_string(),
        ));
    }
    let gain = gain(sample_time, cutoff_frequency);
    Ok(gain * y + (1.0 - gain) * y_last)
}

/// Applies a first-order low-pass filter to the translation and spherical linear interpolation
/// to the rotation of a transformation matrix which represents a Cartesian motion.
///
/// Port of `franka::cartesianLowpassFilter`. `y`, `y_last` and the return value are column-major
/// 4x4 homogeneous transformation matrices.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `sample_time` is negative, infinite or NaN, if
/// `cutoff_frequency` is zero, negative, infinite or NaN, or if an element of either matrix is
/// infinite or NaN.
pub fn cartesian_low_pass_filter(
    sample_time: f64,
    y: &[f64; 16],
    y_last: &[f64; 16],
    cutoff_frequency: f64,
) -> FrankaResult<[f64; 16]> {
    if sample_time < 0.0 || !sample_time.is_finite() {
        return Err(FrankaError::InvalidArgument(
            "Cartesian lowpass-filter: sample_time is negative, infinite or NaN.".to_string(),
        ));
    }
    if cutoff_frequency <= 0.0 || !cutoff_frequency.is_finite() {
        return Err(FrankaError::InvalidArgument(
            "Cartesian lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN."
                .to_string(),
        ));
    }
    for i in 0..16 {
        if !y[i].is_finite() || !y_last[i].is_finite() {
            return Err(FrankaError::InvalidArgument(
                "Cartesian lowpass-filter: current or past input value of the signal to be \
                 filtered is infinite or NaN."
                    .to_string(),
            ));
        }
    }

    let transform = Matrix4::from_column_slice(y);
    let transform_last = Matrix4::from_column_slice(y_last);
    // Eigen's Affine3d::rotation() orthonormalizes the linear part (polar decomposition); this is
    // what lets the filter smooth barely-orthonormal input poses.
    let orientation = quaternion_from_matrix(&orthonormalized_rotation(&linear_of(&transform)));
    let orientation_last =
        quaternion_from_matrix(&orthonormalized_rotation(&linear_of(&transform_last)));

    let gain = gain(sample_time, cutoff_frequency);
    let translation =
        gain * translation_of(&transform) + (1.0 - gain) * translation_of(&transform_last);
    let orientation = slerp(&orientation_last, gain, &orientation);

    let rotation = UnitQuaternion::new_normalize(orientation).to_rotation_matrix();

    let mut filtered_values = *y;
    let rotation = rotation.matrix();
    for col in 0..3 {
        for row in 0..3 {
            filtered_values[col * 4 + row] = rotation[(row, col)];
        }
    }
    filtered_values[12] = translation[0];
    filtered_values[13] = translation[1];
    filtered_values[14] = translation[2];
    Ok(filtered_values)
}

/// Filter gain, shared by both filters: `dt / (dt + 1 / (2 pi f_c))`.
fn gain(sample_time: f64, cutoff_frequency: f64) -> f64 {
    sample_time / (sample_time + (1.0 / (2.0 * std::f64::consts::PI * cutoff_frequency)))
}

fn quaternion_from_matrix(m: &Matrix3<f64>) -> Quaternion<f64> {
    *UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*m)).quaternion()
}

/// Port of `Eigen::QuaternionBase::slerp`, which interpolates along the shortest arc.
fn slerp(from: &Quaternion<f64>, t: f64, to: &Quaternion<f64>) -> Quaternion<f64> {
    const ONE: f64 = 1.0 - f64::EPSILON;
    let d = from.dot(to);
    let abs_d = d.abs();
    let (scale0, mut scale1) = if abs_d >= ONE {
        (1.0 - t, t)
    } else {
        let theta = abs_d.acos();
        let sin_theta = theta.sin();
        (
            ((1.0 - t) * theta).sin() / sin_theta,
            (t * theta).sin() / sin_theta,
        )
    };
    if d < 0.0 {
        scale1 = -scale1;
    }
    from * scale0 + to * scale1
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_utils::differentiate_one_sample_pose as differentiate_one_sample;

    /// Builds a column-major pose array from a row-major 3x3 linear block and a translation.
    fn pose(linear_row_major: [f64; 9], translation: [f64; 3]) -> [f64; 16] {
        let mut out = [0.0; 16];
        for row in 0..3 {
            for col in 0..3 {
                out[col * 4 + row] = linear_row_major[row * 3 + col];
            }
        }
        out[12] = translation[0];
        out[13] = translation[1];
        out[14] = translation[2];
        out[15] = 1.0;
        out
    }

    // ---- TEST(LowpassFilter, KeepsValueIfNoChange) ----
    #[test]
    fn keeps_value_if_no_change() {
        for cutoff in [100.0, 500.0, 1000.0] {
            let filtered = low_pass_filter(0.001, 1.0, 1.0, cutoff).unwrap();
            assert!((filtered - 1.0).abs() < 1e-6, "cutoff {cutoff}: {filtered}");
        }
    }

    // ---- TEST(LowpassFilter, DoesFilter) ----
    #[test]
    fn does_filter() {
        for (cutoff, expected) in [(100.0, 0.3859), (500.0, 0.7585), (900.0, 0.8497)] {
            let filtered = low_pass_filter(0.001, 1.0, 0.0, cutoff).unwrap();
            assert!(
                (filtered - expected).abs() < 1e-4,
                "cutoff {cutoff}: {filtered} vs {expected}"
            );
        }
    }

    // ---- TEST(CartesianLowpassFilter, CanFixNonOrthonormalRotation) ----
    #[test]
    fn can_fix_non_orthonormal_rotation() {
        // These three poses are all only barely orthonormal, such that the filter will generate a
        // jerky movement when it does not orthonormalize them before filtering.
        let pose1 = pose(
            [
                0.00462567,
                0.999974,
                0.00335239, //
                0.0145489,
                -0.00341934,
                0.999888, //
                0.999874,
                -0.00457638,
                -0.0145646,
            ],
            [1.0, 1.0, 1.0],
        );
        let pose2 = pose(
            [
                0.00463526,
                0.999984,
                0.00335239, //
                0.014549,
                -0.00341951,
                0.999888, //
                0.999883,
                -0.00458597,
                -0.0145646,
            ],
            [1.0, 1.0, 1.0],
        );
        let pose3 = pose(
            [
                0.00465436,
                0.999984,
                0.00335239, //
                0.0145489,
                -0.00341979,
                0.999888, //
                0.999883,
                -0.00460507,
                -0.0145646,
            ],
            [1.0, 1.0, 1.0],
        );

        let output1 = cartesian_low_pass_filter(0.001, &pose2, &pose1, 100.0).unwrap();
        let output2 = cartesian_low_pass_filter(0.001, &pose3, &pose2, 100.0).unwrap();
        let velocity1 = differentiate_one_sample(&output1, &pose1, 0.001);
        let velocity2 = differentiate_one_sample(&output2, &pose2, 0.001);

        let mut jerk = [0.0; 6];
        for i in 0..6 {
            let acceleration1 = velocity1[i] / 0.001;
            let acceleration2 = (velocity2[i] - velocity1[i]) / 0.001;
            jerk[i] = (acceleration2 - acceleration1) / 0.001;
        }
        let total_jerk = (jerk[3].powi(2) + jerk[4].powi(2) + jerk[5].powi(2)).sqrt();
        assert!(total_jerk < 1000.0, "total jerk {total_jerk}");
    }

    // ---- additional tests: the invalid-argument paths ----
    #[test]
    fn rejects_invalid_arguments() {
        let message = |e: FrankaError| match e {
            FrankaError::InvalidArgument(msg) => msg,
            other => panic!("expected InvalidArgument, got {other:?}"),
        };
        let identity = pose([1., 0., 0., 0., 1., 0., 0., 0., 1.], [0., 0., 0.]);

        for bad_sample_time in [-1e-3, f64::NAN, f64::INFINITY] {
            assert_eq!(
                message(low_pass_filter(bad_sample_time, 1.0, 0.0, 100.0).unwrap_err()),
                "lowpass-filter: sample_time is negative, infinite or NaN."
            );
            assert_eq!(
                message(
                    cartesian_low_pass_filter(bad_sample_time, &identity, &identity, 100.0)
                        .unwrap_err()
                ),
                "Cartesian lowpass-filter: sample_time is negative, infinite or NaN."
            );
        }
        for bad_cutoff in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                message(low_pass_filter(0.001, 1.0, 0.0, bad_cutoff).unwrap_err()),
                "lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN."
            );
            assert_eq!(
                message(
                    cartesian_low_pass_filter(0.001, &identity, &identity, bad_cutoff).unwrap_err()
                ),
                "Cartesian lowpass-filter: cutoff_frequency is zero, negative, infinite or NaN."
            );
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                message(low_pass_filter(0.001, bad, 0.0, 100.0).unwrap_err()),
                "lowpass-filter: current or past input value of the signal to be filtered is \
                 infinite or NaN."
            );
            assert_eq!(
                message(low_pass_filter(0.001, 1.0, bad, 100.0).unwrap_err()),
                "lowpass-filter: current or past input value of the signal to be filtered is \
                 infinite or NaN."
            );
            let mut bad_pose = identity;
            bad_pose[5] = bad;
            assert_eq!(
                message(cartesian_low_pass_filter(0.001, &bad_pose, &identity, 100.0).unwrap_err()),
                "Cartesian lowpass-filter: current or past input value of the signal to be \
                 filtered is infinite or NaN."
            );
            assert_eq!(
                message(cartesian_low_pass_filter(0.001, &identity, &bad_pose, 100.0).unwrap_err()),
                "Cartesian lowpass-filter: current or past input value of the signal to be \
                 filtered is infinite or NaN."
            );
        }
    }

    #[test]
    fn cartesian_filter_keeps_pose_if_no_change() {
        let p = pose(
            [
                0.0, -1.0, 0.0, //
                1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0,
            ],
            [0.3, -0.2, 0.5],
        );
        let filtered = cartesian_low_pass_filter(0.001, &p, &p, 100.0).unwrap();
        for i in 0..16 {
            assert!(
                (filtered[i] - p[i]).abs() < 1e-12,
                "element {i}: {} vs {}",
                filtered[i],
                p[i]
            );
        }
    }

    #[test]
    fn constants_match_libfranka() {
        assert_eq!(MAX_CUTOFF_FREQUENCY, 1000.0);
        assert_eq!(DEFAULT_CUTOFF_FREQUENCY, 100.0);
    }
}
