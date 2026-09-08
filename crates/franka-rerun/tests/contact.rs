//! The contact estimator on torques synthesised from the crate's own frames: a known point and
//! force on link 4 and on the hand at two configurations with 0.3 Nm of noise; link 5 on
//! the forearm axis, where link 4's chord (and the wrist, at the noise level) fit as well;
//! link 3, where the location along the link is not observable and the span says so; the
//! ambiguity of a force whose line of action passes through a joint axis; the lateral
//! candidates.

use franka::robot_state::IDENTITY_TRANSFORM;
use franka::{Frame, Model};
use franka_rerun::flight::contact::{estimate, point_on_link, torques, ContactOptions};
use franka_rerun::{distance, norm};

const PI: f64 = std::f64::consts::PI;

/// The ready pose, and a bent one with every joint away from zero.
const CONFIGURATIONS: [[f64; 7]; 2] = [
    [
        0.0,
        -PI / 4.0,
        0.0,
        -3.0 * PI / 4.0,
        0.0,
        PI / 2.0,
        PI / 4.0,
    ],
    [0.6, 0.3, -0.5, -1.9, 0.4, 2.2, -0.3],
];

/// The Franka Hand's `F_T_EE`: yawed by -45 degrees, 0.1034 m along the flange's `z`.
fn hand_f_t_ee() -> [f64; 16] {
    let (s, c) = (-PI / 4.0).sin_cos();
    let mut t = IDENTITY_TRANSFORM;
    t[0] = c;
    t[1] = s;
    t[4] = -s;
    t[5] = c;
    t[14] = 0.1034;
    t
}

/// A small deterministic generator, uniform in `[-amplitude, amplitude]`.
struct Noise(u64);

impl Noise {
    fn next(&mut self, amplitude: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let unit = (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        (2.0 * unit - 1.0) * amplitude
    }
}

fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The torques of `force` at `point` on `link` at `q`, plus 0.3 Nm of noise from `seed`.
fn noisy_torques(
    model: &Model,
    q: &[f64; 7],
    link: usize,
    point: &[f64; 3],
    force: &[f64; 3],
    seed: u64,
) -> [f64; 7] {
    let mut tau = torques(model, q, &hand_f_t_ee(), link, point, force);
    let mut noise = Noise(seed);
    for t in tau.iter_mut() {
        *t += noise.next(0.3);
    }
    tau
}

/// Synthesises the torques of `force` at `distance` on `link`, adds noise, and checks that
/// the estimate recovers the link, the point within 2.5 cm (the grid is 1 cm, and 0.3 Nm of
/// noise on a 30 N force moves the optimum by a centimetre or two) and the force within 10 %.
fn check(model: &Model, q: &[f64; 7], link: usize, distance_m: f64, force: [f64; 3], seed: u64) {
    let f_t_ee = hand_f_t_ee();
    let point = point_on_link(model, q, &f_t_ee, link, distance_m, false);
    let tau = noisy_torques(model, q, link, &point, &force, seed);
    let estimate = estimate(model, q, &tau, &f_t_ee, &ContactOptions::default())
        .expect("torques above the floor");
    let label = format!("link {link} at {distance_m} m, q {q:?}: {estimate}");
    assert_eq!(estimate.link, link, "{label}");
    let gap = distance(&estimate.point, &point);
    assert!(gap < 0.025, "{label}: point {gap:.3} m off");
    let error = distance(&estimate.force, &force) / norm(&force);
    assert!(error < 0.10, "{label}: force {:.1} % off", error * 100.0);
    assert!(estimate.residual < 1.5, "{label}");
    assert!(!estimate.on_joint_axis, "{label}");
    let [lo, hi] = estimate.span;
    assert!(
        lo <= estimate.distance && estimate.distance <= hi,
        "{label}"
    );
    if link >= 6 {
        assert!(hi - lo < 0.12, "{label}: the location is not pinned down");
    }
}

#[test]
fn recovers_link_4_and_the_hand_at_two_configurations() {
    let model = Model::native_fer();
    // (link, distance, force): link 4 ten centimetres down the forearm chord from the elbow,
    // and the hand 5 cm and 10 cm past the flange (the wrist, link 6, is 8.8 cm long and at
    // this noise level not told apart from the near end of link 7).
    let cases = [
        (4, 0.10, [22.0, 15.0, -12.0]),
        (7, 0.107 + 0.05, [13.0, -24.0, 18.0]),
        (7, 0.107 + 0.10, [-21.0, 16.0, 13.0]),
    ];
    for (n, q) in CONFIGURATIONS.iter().enumerate() {
        for (m, (link, d, force)) in cases.iter().enumerate() {
            check(&model, q, *link, *d, *force, 1 + (n * 4 + m) as u64);
        }
    }
}

#[test]
fn a_contact_on_the_forearm_axis_is_placed_on_the_forearm() {
    // Four loaded joints fix a point on a line but cannot tell link 5's axis from link 4's
    // chord a few centimetres away, nor -- at the noise level -- from the near end of the
    // wrist; the true link fits within the tolerance of whichever wins.
    let model = Model::native_fer();
    let f_t_ee = hand_f_t_ee();
    for (n, q) in CONFIGURATIONS.iter().enumerate() {
        let point = point_on_link(&model, q, &f_t_ee, 5, -0.12, true);
        let tau = noisy_torques(&model, q, 5, &point, &[-21.0, 16.0, 13.0], 30 + n as u64);
        let estimate = estimate(&model, q, &tau, &f_t_ee, &ContactOptions::default()).unwrap();
        assert!((4..=6).contains(&estimate.link), "{estimate}");
        assert!(
            estimate.link_residuals[4] < estimate.residual + 0.3,
            "{estimate}"
        );
        assert!(distance(&estimate.point, &point) < 0.15, "{estimate}");
    }
}

#[test]
fn a_contact_on_link_3_is_found_but_not_located_along_it() {
    // Three loaded joints cannot pin down three force components and a position: every point
    // of the elbow offset fits, and the estimate says so through its span.
    let model = Model::native_fer();
    let f_t_ee = hand_f_t_ee();
    for (n, q) in CONFIGURATIONS.iter().enumerate() {
        let point = point_on_link(&model, q, &f_t_ee, 3, 0.04, false);
        let tau = noisy_torques(&model, q, 3, &point, &[12.0, -9.0, 15.0], 20 + n as u64);
        let estimate = estimate(&model, q, &tau, &f_t_ee, &ContactOptions::default()).unwrap();
        assert_eq!(estimate.link, 3, "{estimate}");
        let [lo, hi] = estimate.span;
        assert!(lo - 0.011 <= 0.04 && 0.04 <= hi + 0.011, "{estimate}");
        assert!(hi - lo >= 0.03, "{estimate}");
    }
}

#[test]
fn a_force_through_a_joint_axis_is_reported_ambiguous() {
    let model = Model::native_fer();
    let q = CONFIGURATIONS[1];
    let f_t_ee = hand_f_t_ee();
    // A force along the forearm chord passes through joint 4's origin, which is on its axis:
    // every point of the chord, and link 3's end at that origin, explain it equally well.
    let o4 = model.pose_q(Frame::Joint4, &q, &f_t_ee, &IDENTITY_TRANSFORM);
    let point = point_on_link(&model, &q, &f_t_ee, 4, 0.2, false);
    let along: [f64; 3] = std::array::from_fn(|k| point[k] - o4[12 + k]);
    let force = along.map(|v| 20.0 * v / norm(&along));
    let tau = torques(&model, &q, &f_t_ee, 4, &point, &force);
    let estimate = estimate(&model, &q, &tau, &f_t_ee, &ContactOptions::default()).unwrap();
    let (next, residual) = estimate.next_best().unwrap();
    assert!(
        (residual - estimate.residual).abs() < 1e-9,
        "{estimate}: link {next} should tie"
    );
    assert!(
        matches!((estimate.link, next), (3, 4) | (4, 3)),
        "{estimate}"
    );
    let text = estimate.to_string();
    assert!(text.contains("next best link"), "{text}");
    if estimate.link == 4 {
        let [lo, hi] = estimate.span;
        assert!(hi - lo > 0.1, "{estimate}: the chord should be in the span");
    }
}

#[test]
fn torques_below_the_floor_give_no_estimate() {
    let model = Model::native_fer();
    let q = CONFIGURATIONS[0];
    let tau = [0.5, -0.9, 0.2, 0.99, 0.0, -0.3, 0.1];
    let options = ContactOptions::default();
    assert!(estimate(&model, &q, &tau, &IDENTITY_TRANSFORM, &options).is_none());
    let options = ContactOptions {
        noise_floor: 0.4,
        ..ContactOptions::default()
    };
    assert!(estimate(&model, &q, &tau, &IDENTITY_TRANSFORM, &options).is_some());
}

#[test]
fn the_forward_model_is_the_zero_jacobian_at_the_frame_origins() {
    let model = Model::native_fer();
    let f_t_ee = hand_f_t_ee();
    let force = [3.0, -7.0, 11.0];
    for q in &CONFIGURATIONS {
        for (k, frame) in Frame::ALL[..7]
            .iter()
            .chain([&Frame::EndEffector])
            .enumerate()
        {
            let link = (k + 1).min(7);
            let pose = model.pose_q(*frame, q, &f_t_ee, &IDENTITY_TRANSFORM);
            let point = [pose[12], pose[13], pose[14]];
            let tau = torques(&model, q, &f_t_ee, link, &point, &force);
            let j = model.zero_jacobian_q(*frame, q, &f_t_ee, &IDENTITY_TRANSFORM);
            for joint in 0..7 {
                let expected = j[joint * 6] * force[0]
                    + j[joint * 6 + 1] * force[1]
                    + j[joint * 6 + 2] * force[2];
                assert!(
                    (tau[joint] - expected).abs() < 1e-9,
                    "{frame:?} joint {joint}"
                );
            }
        }
    }
}

#[test]
fn lateral_candidates_still_find_an_on_axis_contact() {
    let model = Model::native_fer();
    let q = CONFIGURATIONS[0];
    let f_t_ee = hand_f_t_ee();
    // A point on the candidate grid (40 steps along the forearm chord), so that the exact fit
    // is among the candidates; the force perpendicular to the forearm axis and to the arm
    // from joint 5.
    let o4 = model.pose_q(Frame::Joint4, &q, &f_t_ee, &IDENTITY_TRANSFORM);
    let o5 = model.pose_q(Frame::Joint5, &q, &f_t_ee, &IDENTITY_TRANSFORM);
    let chord: [f64; 3] = std::array::from_fn(|k| o5[12 + k] - o4[12 + k]);
    let point = point_on_link(&model, &q, &f_t_ee, 4, norm(&chord) * 15.0 / 40.0, false);
    let z5 = [o5[8], o5[9], o5[10]];
    let arm: [f64; 3] = std::array::from_fn(|k| point[k] - o5[12 + k]);
    let direction = cross(&arm, &z5);
    let force = direction.map(|v| 15.0 * v / norm(&direction));
    let tau = torques(&model, &q, &f_t_ee, 4, &point, &force);
    let options = ContactOptions {
        lateral: 0.06,
        ..ContactOptions::default()
    };
    let estimate = estimate(&model, &q, &tau, &f_t_ee, &options).unwrap();
    assert_eq!(estimate.link, 4, "{estimate}");
    assert_eq!(estimate.offset, 0.0, "{estimate}");
    assert!(distance(&estimate.point, &point) < 1e-6, "{estimate}");
    assert!(
        distance(&estimate.force, &force) < 0.01 * norm(&force),
        "{estimate}"
    );
}
