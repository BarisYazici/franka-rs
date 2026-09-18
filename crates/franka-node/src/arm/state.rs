//! Pure helpers of the arm thread.

/// The position and the unit quaternion xyzw of a column-major pose (Shepperd's method).
pub(super) fn pose_target(pose: &[f64; 16]) -> [f64; 7] {
    let m = |row: usize, col: usize| pose[col * 4 + row];
    let trace = m(0, 0) + m(1, 1) + m(2, 2);
    let [x, y, z, w] = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (m(2, 1) - m(1, 2)) / s,
            (m(0, 2) - m(2, 0)) / s,
            (m(1, 0) - m(0, 1)) / s,
            0.25 * s,
        ]
    } else if m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2) {
        let s = (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt() * 2.0;
        [
            0.25 * s,
            (m(0, 1) + m(1, 0)) / s,
            (m(0, 2) + m(2, 0)) / s,
            (m(2, 1) - m(1, 2)) / s,
        ]
    } else if m(1, 1) > m(2, 2) {
        let s = (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt() * 2.0;
        [
            (m(0, 1) + m(1, 0)) / s,
            0.25 * s,
            (m(1, 2) + m(2, 1)) / s,
            (m(0, 2) - m(2, 0)) / s,
        ]
    } else {
        let s = (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt() * 2.0;
        [
            (m(0, 2) + m(2, 0)) / s,
            (m(1, 2) + m(2, 1)) / s,
            0.25 * s,
            (m(1, 0) - m(0, 1)) / s,
        ]
    };
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    [
        pose[12],
        pose[13],
        pose[14],
        x / norm,
        y / norm,
        z / norm,
        w / norm,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(rotation: [[f64; 3]; 3], position: [f64; 3]) -> [f64; 16] {
        let mut pose = [0.0; 16];
        for (col, column) in rotation.iter().enumerate() {
            for (row, value) in column.iter().enumerate() {
                pose[col * 4 + row] = *value;
            }
        }
        pose[12..15].copy_from_slice(&position);
        pose[15] = 1.0;
        pose
    }

    fn close(a: [f64; 7], b: [f64; 7]) -> bool {
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-12)
    }

    #[test]
    fn pose_target_covers_every_branch() {
        let identity = pose(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.3, 0.1, 0.5],
        );
        assert!(close(
            pose_target(&identity),
            [0.3, 0.1, 0.5, 0.0, 0.0, 0.0, 1.0]
        ));
        let about_x = pose(
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            [0.0; 3],
        );
        assert!(close(
            pose_target(&about_x),
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        ));
        let about_y = pose(
            [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
            [0.0; 3],
        );
        assert!(close(
            pose_target(&about_y),
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        ));
        let about_z = pose(
            [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0; 3],
        );
        assert!(close(
            pose_target(&about_z),
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        ));
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let quarter_z = pose(
            [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            [0.0; 3],
        );
        assert!(close(
            pose_target(&quarter_z),
            [0.0, 0.0, 0.0, 0.0, 0.0, h, h]
        ));
    }
}
