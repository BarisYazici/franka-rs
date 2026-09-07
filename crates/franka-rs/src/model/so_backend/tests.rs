//! Unit tests for the column-major product and the temporary-file loader.

use super::*;

#[test]
fn column_major_product_matches_a_hand_worked_example() {
    // a = translation (1, 2, 3); b = 90 degree rotation about z.
    let a = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
    ];
    let b = [
        0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    let product = mat4_mul(&a, &b);
    // Rotation block is b's, translation block is a's.
    let expected = [
        0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
    ];
    assert_eq!(product, expected);
}

#[test]
fn column_major_product_is_not_commutative() {
    let a = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 3.0, 1.0,
    ];
    let b = [
        0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    assert_ne!(mat4_mul(&a, &b), mat4_mul(&b, &a));
}

#[test]
fn temp_library_file_is_written_private_and_removed_on_drop() {
    let guard = write_temp_library(b"not a shared object").expect("temp file");
    let path = guard.path.clone();
    assert_eq!(
        std::fs::read(&path).expect("readable"),
        b"not a shared object"
    );
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mode = std::fs::metadata(&path).unwrap().permissions().mode();
        assert_eq!(mode & 0o777, 0o600, "mode {mode:o}");
    }
    drop(guard);
    assert!(!path.exists(), "{} still exists", path.display());
}

#[test]
fn two_temp_libraries_do_not_collide() {
    let a = write_temp_library(b"a").expect("temp file");
    let b = write_temp_library(b"b").expect("temp file");
    assert_ne!(a.path, b.path);
}

#[test]
fn loading_a_non_library_is_a_model_error() {
    let guard = write_temp_library(b"not a shared object").expect("temp file");
    // SAFETY: the "library" is a file this test just wrote; `dlopen` rejects it
    // before any of its (non-existent) code could run.
    let error = unsafe { SoModelBackend::open(&guard.path) }.expect_err("must fail");
    assert!(
        format!("{error}").contains("Cannot load model library"),
        "{error}"
    );
}
