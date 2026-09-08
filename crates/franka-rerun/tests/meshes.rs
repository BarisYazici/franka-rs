//! The mesh placement: the URDF the FER model is built from against franka_description's joint
//! origins, the link and hand frames, and -- with `FRANKA_MESHES=<dir>` pointing at the output
//! of `tools/franka-meshes/convert.py` for one robot -- the converted meshes' extents against
//! the flange and against each other.

use std::path::{Path, PathBuf};

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::{Frame, Model};
use franka_rerun::meshes::{hand_pose, link_pose, yawed, FINGER_Z, HAND_YAW};
use franka_rerun::Meshes;

const PI: f64 = std::f64::consts::PI;

/// franka_description `robots/fer/kinematics.yaml` (also `robots/fr3/kinematics.yaml`):
/// `(xyz, roll)` of `joint1 .. joint8`; pitch and yaw are zero throughout.
const FRANKA_DESCRIPTION_JOINTS: [([f64; 3], f64); 8] = [
    ([0.0, 0.0, 0.333], 0.0),
    ([0.0, 0.0, 0.0], -PI / 2.0),
    ([0.0, -0.316, 0.0], PI / 2.0),
    ([0.0825, 0.0, 0.0], PI / 2.0),
    ([-0.0825, 0.384, 0.0], -PI / 2.0),
    ([0.0, 0.0, 0.0], PI / 2.0),
    ([0.088, 0.0, 0.0], PI / 2.0),
    ([0.0, 0.0, 0.107], 0.0),
];

fn assert_franka_description_chain(urdf: &str, prefix: &str) {
    let robot = urdf_rs::read_from_string(urdf).unwrap();
    for (k, (xyz, roll)) in FRANKA_DESCRIPTION_JOINTS.iter().enumerate() {
        let name = format!("{prefix}joint{}", k + 1);
        let joint = robot
            .joints
            .iter()
            .find(|j| j.name == name)
            .unwrap_or_else(|| panic!("{name}"));
        assert_eq!(joint.parent.link, format!("{prefix}link{k}"), "{name}");
        assert_eq!(joint.child.link, format!("{prefix}link{}", k + 1), "{name}");
        for (actual, expected) in joint.origin.xyz.0.iter().zip(xyz) {
            assert!((actual - expected).abs() < 1e-9, "{name} xyz");
        }
        let rpy = joint.origin.rpy.0;
        assert!(
            (rpy[0] - roll).abs() < 1e-9 && rpy[1].abs() < 1e-9 && rpy[2].abs() < 1e-9,
            "{name} rpy"
        );
        if k < 7 {
            assert_eq!(joint.axis.xyz.0, [0.0, 0.0, 1.0], "{name} axis");
        }
    }
}

#[test]
fn the_urdfs_have_franka_descriptions_joint_origins() {
    assert_franka_description_chain(franka::model::FER_URDF, "");
    let fr3 = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../franka-rs/tests/data/fr3.urdf"
    );
    assert_franka_description_chain(&std::fs::read_to_string(fr3).unwrap(), "");
}

const Q: [f64; 7] = [0.3, -0.6, 0.2, -2.0, 0.1, 1.8, 0.9];

#[test]
fn link_frames_are_the_joint_frames_and_the_base() {
    let model = Model::native_fer();
    assert_eq!(link_pose(&model, &Q, 0), IDENTITY_TRANSFORM);
    for (k, frame) in Frame::ALL[..7].iter().enumerate() {
        let expected = model.pose_q(*frame, &Q, &IDENTITY_TRANSFORM, &IDENTITY_TRANSFORM);
        assert_eq!(link_pose(&model, &Q, k + 1), expected, "link {}", k + 1);
    }
}

#[test]
fn the_hand_frame_puts_the_franka_hands_end_effector_0_1034_m_along_its_z() {
    let model = Model::native_fer();
    let mut f_t_ee = yawed(&IDENTITY_TRANSFORM, HAND_YAW);
    f_t_ee[14] = 0.1034;
    let ee = model.pose_q(Frame::EndEffector, &Q, &f_t_ee, &IDENTITY_TRANSFORM);
    let hand = hand_pose(&model, &Q);
    for r in 0..3 {
        let tip = hand[12 + r] + 0.1034 * hand[8 + r];
        assert!((tip - ee[12 + r]).abs() < 1e-9, "translation {r}");
        for c in 0..3 {
            assert!(
                (hand[4 * c + r] - ee[4 * c + r]).abs() < 1e-9,
                "rotation {r} {c}"
            );
        }
    }
    // The fingers sit FINGER_Z along the same axis, short of the end effector.
    let tip = FINGER_Z;
    assert!(tip < 0.1034);
}

/// `[min, max]` of every `POSITION` accessor of a `.glb`, united; asserts every node is the
/// identity so that the accessor bounds are the mesh's extent in the link frame.
fn glb_bounds(path: &Path) -> ([f64; 3], [f64; 3]) {
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(&bytes[0..4], b"glTF", "{}", path.display());
    let json_len = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    assert_eq!(&bytes[16..20], b"JSON");
    let json: serde_json::Value = serde_json::from_slice(&bytes[20..20 + json_len]).unwrap();
    for node in json["nodes"].as_array().unwrap() {
        for key in ["matrix", "rotation", "translation", "scale"] {
            assert!(node.get(key).is_none(), "{}: node {key}", path.display());
        }
    }
    let accessors = json["accessors"].as_array().unwrap();
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for mesh in json["meshes"].as_array().unwrap() {
        for primitive in mesh["primitives"].as_array().unwrap() {
            let accessor =
                &accessors[primitive["attributes"]["POSITION"].as_u64().unwrap() as usize];
            for k in 0..3 {
                lo[k] = lo[k].min(accessor["min"][k].as_f64().unwrap());
                hi[k] = hi[k].max(accessor["max"][k].as_f64().unwrap());
            }
        }
    }
    (lo, hi)
}

fn meshes_dir() -> Option<PathBuf> {
    match std::env::var_os("FRANKA_MESHES") {
        Some(dir) => Some(PathBuf::from(dir)),
        None => {
            eprintln!("FRANKA_MESHES not set: skipping the converted-mesh checks");
            None
        }
    }
}

#[test]
fn converted_meshes_end_at_the_flange_and_meet_each_other() {
    let Some(dir) = meshes_dir() else { return };
    let meshes = Meshes::find(&dir).unwrap();
    assert!(
        meshes.links.iter().all(Option::is_some),
        "{}",
        meshes.describe()
    );
    let model = Model::native_fer();
    let q0 = [0.0; 7];
    let joint7 = model.pose_q(Frame::Joint7, &q0, &IDENTITY_TRANSFORM, &IDENTITY_TRANSFORM);
    let flange = model.pose_q(Frame::Flange, &q0, &IDENTITY_TRANSFORM, &IDENTITY_TRANSFORM);
    // The flange sits 0.107 m along joint 7's own z (which points down at q = 0).
    let flange_z = franka_rerun::distance(
        &[flange[12], flange[13], flange[14]],
        &[joint7[12], joint7[13], joint7[14]],
    );
    assert!((flange_z - 0.107).abs() < 1e-9, "{flange_z}");
    // link7's mesh, in the joint 7 frame, ends at the flange face.
    let (_, hi7) = glb_bounds(meshes.links[7].as_ref().unwrap());
    assert!(
        (hi7[2] - flange_z).abs() < 1e-3,
        "link7 ends at z {:.4}, flange at {flange_z}",
        hi7[2]
    );
    // link1 (frame 0.333 m up) reaches down to where link0 ends.
    let (_, hi0) = glb_bounds(meshes.links[0].as_ref().unwrap());
    let (lo1, _) = glb_bounds(meshes.links[1].as_ref().unwrap());
    assert!(
        (0.333 + lo1[2] - hi0[2]).abs() < 2e-3,
        "link0 top {:.4}, link1 bottom {:.4}",
        hi0[2],
        0.333 + lo1[2]
    );
    assert!(lo1[2] > -0.333, "link1 does not reach below the base");
    // The hand mesh straddles the flange plane and the fingers mount inside it.
    if let (Some(hand), Some(finger)) = (&meshes.hand, &meshes.finger) {
        let (lo_h, hi_h) = glb_bounds(hand);
        assert!(
            lo_h[2] > -0.03 && hi_h[2] > FINGER_Z && hi_h[2] < 0.08,
            "hand z {:?}..{:?}",
            lo_h,
            hi_h
        );
        let (lo_f, hi_f) = glb_bounds(finger);
        assert!(
            lo_f[2] >= -1e-3 && hi_f[2] < 0.06,
            "finger z {:?}..{:?}",
            lo_f,
            hi_f
        );
        assert!(
            FINGER_Z + hi_f[2] < 0.1034 + 0.02,
            "the fingertips end near the end effector"
        );
    }
}

#[test]
fn meshes_are_logged_once_and_their_poses_per_record() {
    let Some(dir) = meshes_dir() else { return };
    let meshes = Meshes::find(&dir).unwrap();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
    meshes.log_static(&rec).unwrap();
    let before = storage.num_msgs();
    assert!(before >= 8, "{before} static messages");
    meshes.log_poses(&rec, &Model::native_fer(), &Q).unwrap();
    rec.flush_blocking().unwrap();
    assert!(storage.num_msgs() > before);
}

#[test]
fn an_empty_directory_is_an_error() {
    let dir = std::env::temp_dir().join(format!("franka-rerun-meshes-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let error = Meshes::find(&dir).unwrap_err().to_string();
    assert!(error.contains("no link0..link7 mesh"), "{error}");
}
