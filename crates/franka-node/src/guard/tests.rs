use super::*;
use franka::robot::target_control::joint_position_limits;
use franka::FciVersion;

const HOLDER: u32 = 7;
const START: [f64; 7] = [0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0];

fn guard() -> Guard {
    let mut guard = Guard::new(GuardOptions::default(), START).unwrap();
    guard.set_holder(HOLDER, 0);
    guard
}

fn target(seq: u64, data: [f64; 7]) -> TargetMsg {
    TargetMsg::new(Kind::Cartesian, HOLDER, seq, 0, data)
}

fn moved(dx: f64, dy: f64, dz: f64) -> [f64; 7] {
    [
        START[0] + dx,
        START[1] + dy,
        START[2] + dz,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn about_z(angle: f64) -> [f64; 7] {
    let (s, c) = (angle / 2.0).sin_cos();
    [START[0], START[1], START[2], 0.0, 0.0, s, c]
}

fn refused(verdict: Verdict) -> Reason {
    match verdict {
        Verdict::Refuse(reason) => reason,
        Verdict::Accept => panic!("accepted"),
    }
}

#[test]
fn accepts_and_steps_from_the_accepted_target() {
    let mut guard = guard();
    assert_eq!(
        guard.check(&target(1, moved(0.04, 0.0, 0.0)), 0, None),
        Verdict::Accept
    );
    assert_eq!(guard.previous(), moved(0.04, 0.0, 0.0));
    assert_eq!(guard.last_seq(), 1);
    assert_eq!(
        guard.check(&target(2, moved(0.08, 0.0, 0.0)), 0, None),
        Verdict::Accept
    );
    assert_eq!(guard.holder(), HOLDER);
}

#[test]
fn refuses_the_other_kind_and_an_unknown_kind_byte() {
    let mut guard = guard();
    assert_eq!(guard.kind(), Kind::Cartesian);
    let msg = TargetMsg::new(Kind::Joints, HOLDER, 1, 0, START);
    assert_eq!(
        refused(guard.check(&msg, 0, None)),
        Reason::WrongKind {
            got: Kind::Joints,
            mode: Kind::Cartesian
        }
    );
    let mut unknown = msg;
    unknown.kind = 9;
    assert_eq!(refused(guard.check(&unknown, 0, None)), Reason::Kind(9));
    let mut joints = joint_guard();
    assert_eq!(joints.kind(), Kind::Joints);
    assert_eq!(
        refused(joints.check(&target(1, START), 0, None)),
        Reason::WrongKind {
            got: Kind::Cartesian,
            mode: Kind::Joints
        }
    );
}

const START_Q: [f64; 7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];

fn joint_guard() -> Guard {
    let limits = joint_position_limits(FciVersion::V10);
    let mut guard = Guard::joints(GuardOptions::default(), limits, START_Q).unwrap();
    guard.set_holder(HOLDER, 0);
    guard
}

fn joints(seq: u64, q: [f64; 7]) -> TargetMsg {
    TargetMsg::new(Kind::Joints, HOLDER, seq, 0, q)
}

#[test]
fn joint_gate_steps_per_joint_from_the_accepted_target() {
    let mut guard = joint_guard();
    let mut q = START_Q;
    q[6] += 0.19;
    assert_eq!(guard.check(&joints(1, q), 0, None), Verdict::Accept);
    assert_eq!(guard.previous(), q);
    q[6] += 0.19;
    q[0] -= 0.2;
    assert_eq!(guard.check(&joints(2, q), 0, None), Verdict::Accept);
    let mut far = q;
    far[3] += 0.21;
    let reason = refused(guard.check(&joints(3, far), 0, None));
    assert!(
        matches!(reason, Reason::JointStep { joint: 4, rad } if (rad - 0.21).abs() < 1e-12),
        "{reason:?}"
    );
    assert_eq!(guard.previous(), q);
    assert_eq!(guard.last_seq(), 2);
}

#[test]
fn joint_gate_refuses_non_finite_and_the_inset_limits() {
    let mut guard = joint_guard();
    let mut nan = START_Q;
    nan[2] = f64::NAN;
    assert_eq!(
        refused(guard.check(&joints(1, nan), 0, None)),
        Reason::NotFinite
    );
    let (lower, upper) = joint_position_limits(FciVersion::V10);
    let mut edge = START_Q;
    edge[5] = 1.571 + 0.1;
    let options = GuardOptions {
        max_step_joint: 10.0,
        ..GuardOptions::default()
    };
    let mut guard = Guard::joints(options, (lower, upper), edge).unwrap();
    guard.set_holder(HOLDER, 0);
    // The default inset is the library's joint position margin, above its bare inset.
    let inset = options.joint_limit_inset;
    assert_eq!(inset, ImpedanceOptions::joint().joint_position_margin);
    assert!(inset > JOINT_LIMIT_INSET);
    let mut inside = edge;
    inside[5] = upper[5] - inset;
    assert_eq!(guard.check(&joints(1, inside), 0, None), Verdict::Accept);
    let mut outside = edge;
    outside[5] = upper[5] - inset + 1e-9;
    let reason = refused(guard.check(&joints(2, outside), 0, None));
    assert!(
        matches!(reason, Reason::JointLimit { joint: 6, .. }),
        "{reason:?}"
    );
    let mut low = edge;
    low[3] = lower[3] + (JOINT_LIMIT_INSET + inset) / 2.0;
    let reason = refused(guard.check(&joints(2, low), 0, None));
    assert!(
        matches!(reason, Reason::JointLimit { joint: 4, .. }),
        "{reason:?}"
    );
    assert!(Guard::joints(
        GuardOptions::default(),
        joint_position_limits(FciVersion::V5),
        nan
    )
    .is_err());
}

#[test]
fn refuses_a_sender_that_is_not_the_holder() {
    let mut guard = guard();
    let msg = TargetMsg::new(Kind::Cartesian, 8, 1, 0, START);
    assert_eq!(refused(guard.check(&msg, 0, None)), Reason::NotHolder(8));
    assert_eq!(guard.last_seq(), 0);
    let mut none = Guard::new(GuardOptions::default(), START).unwrap();
    assert_eq!(
        refused(none.check(&target(1, START), 0, None)),
        Reason::NotHolder(HOLDER)
    );
}

#[test]
fn client_zero_is_never_the_holder() {
    let mut none = Guard::new(GuardOptions::default(), START).unwrap();
    assert_eq!(none.holder(), 0);
    let msg = TargetMsg::new(Kind::Cartesian, 0, 1, 0, START);
    assert_eq!(refused(none.check(&msg, 0, None)), Reason::NotHolder(0));
    let mut guard = guard();
    assert_eq!(refused(guard.check(&msg, 0, None)), Reason::NotHolder(0));
}

#[test]
fn the_initial_target_must_be_finite_and_unit() {
    let mut nan = START;
    nan[0] = f64::NAN;
    assert_eq!(
        Guard::new(GuardOptions::default(), nan).unwrap_err(),
        Reason::NotFinite
    );
    let zero_quaternion = [START[0], START[1], START[2], 0.0, 0.0, 0.0, 0.0];
    assert_eq!(
        Guard::new(GuardOptions::default(), zero_quaternion).unwrap_err(),
        Reason::NotUnit(0.0)
    );
    assert!(Guard::new(GuardOptions::default(), START).is_ok());
}

#[test]
fn nan_steps_refuse_instead_of_passing() {
    assert!(angle_between(&[0.0; 4], &START[3..]).is_nan());
    assert!(angle_between(&[f64::NAN, 0.0, 0.0, 1.0], &START[3..]).is_nan());
    assert_eq!(angle_between(&START[3..], &START[3..]), 0.0);
    assert!(norm(&[f64::NAN]).is_nan());
}

#[test]
fn refuses_seq_that_does_not_increase() {
    let mut guard = guard();
    assert_eq!(
        refused(guard.check(&target(0, START), 0, None)),
        Reason::Seq { last: 0, got: 0 }
    );
    assert_eq!(guard.check(&target(5, START), 0, None), Verdict::Accept);
    assert_eq!(
        refused(guard.check(&target(5, START), 0, None)),
        Reason::Seq { last: 5, got: 5 }
    );
    assert_eq!(
        refused(guard.check(&target(4, START), 0, None)),
        Reason::Seq { last: 5, got: 4 }
    );
    assert_eq!(guard.check(&target(6, START), 0, None), Verdict::Accept);
    guard.set_holder(HOLDER, 0);
    assert_eq!(guard.check(&target(1, START), 0, None), Verdict::Accept);
}

#[test]
fn refuses_non_finite_components() {
    let mut guard = guard();
    let mut nan = START;
    nan[2] = f64::NAN;
    assert_eq!(
        refused(guard.check(&target(1, nan), 0, None)),
        Reason::NotFinite
    );
    let mut inf = START;
    inf[6] = f64::INFINITY;
    assert_eq!(
        refused(guard.check(&target(1, inf), 0, None)),
        Reason::NotFinite
    );
    assert_eq!(guard.previous(), START);
}

#[test]
fn refuses_a_non_unit_quaternion_and_tolerates_a_millesimal() {
    let mut guard = guard();
    let mut off = START;
    off[6] = 1.01;
    let reason = refused(guard.check(&target(1, off), 0, None));
    assert!(
        matches!(reason, Reason::NotUnit(n) if (n - 1.01).abs() < 1e-12),
        "{reason:?}"
    );
    let mut near = START;
    near[6] = 1.0005;
    assert_eq!(guard.check(&target(1, near), 0, None), Verdict::Accept);
}

#[test]
fn quaternion_sign_is_not_a_rotation() {
    let mut guard = guard();
    let flipped = [START[0], START[1], START[2], 0.0, 0.0, 0.0, -1.0];
    assert_eq!(guard.check(&target(1, flipped), 0, None), Verdict::Accept);
    let small = about_z(0.2);
    let negated = [
        small[0], small[1], small[2], -small[3], -small[4], -small[5], -small[6],
    ];
    assert_eq!(guard.check(&target(2, negated), 0, None), Verdict::Accept);
}

#[test]
fn refuses_a_translation_step_beyond_max_step() {
    let mut guard = guard();
    let reason = refused(guard.check(&target(1, moved(0.0, 0.06, 0.0)), 0, None));
    assert!(
        matches!(reason, Reason::Step(s) if (s - 0.06).abs() < 1e-12),
        "{reason:?}"
    );
    assert_eq!(
        guard.check(&target(1, moved(0.0, 0.049, 0.0)), 0, None),
        Verdict::Accept
    );
    let diagonal = moved(0.0, 0.049 + 0.04, 0.04);
    let reason = refused(guard.check(&target(2, diagonal), 0, None));
    assert!(matches!(reason, Reason::Step(_)), "{reason:?}");
}

#[test]
fn refuses_a_rotation_step_beyond_max_step_rotation() {
    let mut guard = guard();
    let reason = refused(guard.check(&target(1, about_z(0.35)), 0, None));
    assert!(
        matches!(reason, Reason::RotationStep(a) if (a - 0.35).abs() < 1e-9),
        "{reason:?}"
    );
    assert_eq!(
        guard.check(&target(1, about_z(0.25)), 0, None),
        Verdict::Accept
    );
    assert_eq!(
        guard.check(&target(2, about_z(0.5)), 0, None),
        Verdict::Accept
    );
    let reason = refused(guard.check(&target(3, about_z(0.8)), 0, None));
    assert!(matches!(reason, Reason::RotationStep(_)), "{reason:?}");
}

/// The box is off by default, and this is the behaviour rather than the plumbing: with no box
/// the gate accepts a position the retired default (x 0.2..0.8) refused, and a config that
/// names that box still refuses the same position, on the axis it left.
#[test]
fn no_workspace_accepts_what_the_old_default_refused() {
    let start = [0.22, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0];
    let mut outside = start;
    outside[0] = 0.18; // a 0.04 m step, inside max_step, but under the retired floor of 0.2
    let verdict = |options| {
        let mut guard = Guard::new(options, start).unwrap();
        guard.set_holder(HOLDER, 0);
        guard.check(&target(1, outside), 0, None)
    };
    let boxed = GuardOptions {
        workspace: Some(Workspace {
            min: [0.2, -0.5, 0.0],
            max: [0.8, 0.5, 0.8],
        }),
        ..GuardOptions::default()
    };
    assert_eq!(verdict(boxed), Verdict::Refuse(Reason::Workspace(Axis::X)));
    assert_eq!(verdict(GuardOptions::default()), Verdict::Accept);
}

#[test]
fn workspace_box_is_inclusive() {
    let edge = [0.78, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0];
    let options = GuardOptions {
        workspace: Some(Workspace {
            min: [0.2, -0.5, 0.0],
            max: [0.8, 0.5, 0.8],
        }),
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, edge).unwrap();
    guard.set_holder(HOLDER, 0);
    let mut on_x = edge;
    on_x[0] = 0.8;
    assert_eq!(guard.check(&target(1, on_x), 0, None), Verdict::Accept);
    let mut past_x = edge;
    past_x[0] = 0.8 + 1e-9;
    assert_eq!(
        refused(guard.check(&target(2, past_x), 0, None)),
        Reason::Workspace(Axis::X)
    );
    let mut on_z = edge;
    on_z[2] = 0.0;
    assert_eq!(guard.check(&target(2, on_z), 0, None), Verdict::Accept);
    let mut under_z = edge;
    under_z[2] = -0.01;
    assert_eq!(
        refused(guard.check(&target(3, under_z), 0, None)),
        Reason::Workspace(Axis::Z)
    );
    let mut wide_y = edge;
    wide_y[1] = 0.04;
    assert_eq!(guard.check(&target(3, wide_y), 0, None), Verdict::Accept);
}

#[test]
fn token_bucket_holds_twice_the_rate_and_refills_at_it() {
    let options = GuardOptions {
        rate_hz: 250.0,
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, START).unwrap();
    guard.set_holder(HOLDER, 0);
    let mut seq = 0;
    let mut send = |guard: &mut Guard, now_ns: u64| {
        seq += 1;
        guard.check(&target(seq, START), now_ns, None)
    };
    for _ in 0..500 {
        assert_eq!(send(&mut guard, 0), Verdict::Accept);
    }
    assert_eq!(refused(send(&mut guard, 0)), Reason::Rate);
    assert_eq!(refused(send(&mut guard, 3_000_000)), Reason::Rate);
    assert_eq!(send(&mut guard, 5_000_000), Verdict::Accept);
    assert_eq!(refused(send(&mut guard, 5_000_000)), Reason::Rate);
    for _ in 0..500 {
        assert_eq!(send(&mut guard, 10_000_000_000), Verdict::Accept);
    }
    assert_eq!(refused(send(&mut guard, 10_000_000_000)), Reason::Rate);
    assert_eq!(refused(send(&mut guard, 9_000_000_000)), Reason::Rate);
}

#[test]
fn refusals_do_not_spend_tokens_or_advance_state() {
    let options = GuardOptions {
        rate_hz: 1.0,
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, START).unwrap();
    guard.set_holder(HOLDER, 0);
    for _ in 0..10 {
        let reason = refused(guard.check(&target(1, moved(1.0, 0.0, 0.0)), 0, None));
        assert!(matches!(reason, Reason::Step(_)), "{reason:?}");
    }
    assert_eq!(guard.check(&target(1, START), 0, None), Verdict::Accept);
    assert_eq!(guard.check(&target(2, START), 0, None), Verdict::Accept);
    assert_eq!(
        refused(guard.check(&target(3, START), 0, None)),
        Reason::Rate
    );
    assert_eq!(guard.last_seq(), 2);
}

#[test]
fn reasons_display_as_reply_text() {
    assert_eq!(Reason::Kind(9).to_string(), "unsupported kind 9");
    assert_eq!(
        Reason::WrongKind {
            got: Kind::Cartesian,
            mode: Kind::Joints
        }
        .to_string(),
        "cartesian target in joints mode"
    );
    assert_eq!(
        Reason::JointStep {
            joint: 4,
            rad: 0.25
        }
        .to_string(),
        "joint 4 step 0.250 rad too large"
    );
    assert_eq!(
        Reason::JointLimit {
            joint: 6,
            rad: 4.51
        }
        .to_string(),
        "joint 6 at 4.510 rad outside limits"
    );
    assert_eq!(
        Reason::NotHolder(9).to_string(),
        "client 9 is not the holder"
    );
    assert_eq!(
        Reason::Seq { last: 5, got: 3 }.to_string(),
        "seq 3 not after 5"
    );
    assert_eq!(Reason::NotFinite.to_string(), "not finite");
    assert_eq!(
        Reason::NotUnit(1.5).to_string(),
        "quaternion norm 1.5000 not unit"
    );
    assert_eq!(Reason::Step(0.1).to_string(), "step 0.100 m too large");
    assert_eq!(
        Reason::RotationStep(0.5).to_string(),
        "rotation step 0.500 rad too large"
    );
    assert_eq!(Reason::Lead(0.081).to_string(), "lead 0.081 m too large");
    assert_eq!(
        Reason::LeadRotation(0.31).to_string(),
        "lead rotation 0.310 rad too large"
    );
    assert_eq!(
        Reason::Workspace(Axis::Y).to_string(),
        "outside workspace on y"
    );
    assert_eq!(Reason::Rate.to_string(), "rate limit");
}

/// A gate whose previous accepted target already leads the arm by `lead` on x, so a target's
/// step and its lead are different distances and each check can be tested on its own.
fn leading(lead: f64) -> Guard {
    let mut guard = Guard::new(GuardOptions::default(), moved(lead, 0.0, 0.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    guard
}

#[test]
fn refuses_a_lead_beyond_max_lead_as_a_norm() {
    let mut guard = leading(0.04);
    // Exactly at the limit passes; a nanometre past it is refused, and the step (0.01) is not
    // what refuses it.
    assert_eq!(
        guard.check(&target(1, moved(0.05, 0.0, 0.0)), 0, Some(&START)),
        Verdict::Accept
    );
    let reason = refused(guard.check(&target(2, moved(0.05 + 1e-9, 0.0, 0.0)), 0, Some(&START)));
    assert!(
        matches!(reason, Reason::Lead(m) if (m - 0.05).abs() < 1e-6),
        "{reason:?}"
    );
    // A norm, not a per-axis check: 0.04 on each axis is 0.069 m of lead.
    let mut guard = Guard::new(GuardOptions::default(), moved(0.04, 0.04, 0.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    let diagonal = moved(0.04, 0.04, 0.04);
    let reason = refused(guard.check(&target(1, diagonal), 0, Some(&START)));
    assert!(
        matches!(reason, Reason::Lead(m) if (m - 0.04 * 3f64.sqrt()).abs() < 1e-12),
        "{reason:?}"
    );
}

#[test]
fn refuses_an_orientation_lead_beyond_max_lead_rotation_whatever_the_sign() {
    let mut guard = Guard::new(GuardOptions::default(), about_z(0.15)).unwrap();
    guard.set_holder(HOLDER, 0);
    assert_eq!(
        guard.check(&target(1, about_z(0.26)), 0, Some(&START)),
        Verdict::Accept
    );
    let reason = refused(guard.check(&target(2, about_z(0.4)), 0, Some(&START)));
    assert!(
        matches!(reason, Reason::LeadRotation(a) if (a - 0.4).abs() < 1e-9),
        "{reason:?}"
    );
    // The sign of a quaternion is not a rotation, on this reference either.
    let negated = [START[0], START[1], START[2], 0.0, 0.0, 0.0, -1.0];
    assert_eq!(
        guard.check(&target(2, about_z(0.1)), 0, Some(&negated)),
        Verdict::Accept
    );
}

#[test]
fn a_lead_refusal_advances_nothing_and_spends_no_token() {
    let options = GuardOptions {
        rate_hz: 1.0,
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, moved(0.04, 0.0, 0.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    for _ in 0..10 {
        let reason = refused(guard.check(&target(1, moved(0.08, 0.0, 0.0)), 0, Some(&START)));
        assert!(matches!(reason, Reason::Lead(_)), "{reason:?}");
    }
    assert_eq!(guard.previous(), moved(0.04, 0.0, 0.0));
    assert_eq!(guard.last_seq(), 0);
    // The bucket still holds its two tokens.
    assert_eq!(
        guard.check(&target(1, moved(0.03, 0.0, 0.0)), 0, Some(&START)),
        Verdict::Accept
    );
    assert_eq!(
        guard.check(&target(2, moved(0.02, 0.0, 0.0)), 0, Some(&START)),
        Verdict::Accept
    );
    assert_eq!(
        refused(guard.check(&target(3, START), 0, Some(&START))),
        Reason::Rate
    );
}

#[test]
fn no_reference_or_a_zero_limit_disables_the_lead_checks() {
    let far = moved(0.4, 0.0, 0.0);
    let mut guard = leading(0.38);
    assert_eq!(guard.check(&target(1, far), 0, None), Verdict::Accept);
    let options = GuardOptions {
        max_lead: 0.0,
        max_lead_rotation: 0.0,
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, moved(0.38, 0.0, 0.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    assert_eq!(
        guard.check(&target(1, far), 0, Some(&START)),
        Verdict::Accept
    );
    let mut turned = Guard::new(options, about_z(0.9)).unwrap();
    turned.set_holder(HOLDER, 0);
    assert_eq!(
        turned.check(&target(1, about_z(1.0)), 0, Some(&START)),
        Verdict::Accept
    );
}

#[test]
fn a_reference_that_is_not_finite_refuses_instead_of_passing() {
    let mut nan = START;
    nan[1] = f64::NAN;
    let mut guard = guard();
    let reason = refused(guard.check(&target(1, moved(0.01, 0.0, 0.0)), 0, Some(&nan)));
    assert!(
        matches!(reason, Reason::Lead(m) if m.is_nan()),
        "{reason:?}"
    );
    let mut zero_quaternion = START;
    zero_quaternion[6] = 0.0;
    let reason = refused(guard.check(&target(1, moved(0.01, 0.0, 0.0)), 0, Some(&zero_quaternion)));
    assert!(
        matches!(reason, Reason::LeadRotation(a) if a.is_nan()),
        "{reason:?}"
    );
    assert_eq!(guard.last_seq(), 0);
}

#[test]
fn an_anchor_steps_from_the_arm_and_lets_a_locked_out_stream_back_in() {
    // The commander's stream jumped: its last accepted target is 0.4 m from where the arm is,
    // and every target near the arm is refused for the step, for ever.
    let mut guard = leading(0.4);
    let back = moved(0.01, 0.0, 0.0);
    let reason = refused(guard.check(&target(1, back), 0, Some(&START)));
    assert!(matches!(reason, Reason::Step(_)), "{reason:?}");
    // One anchored target steps from the arm instead, and is accepted.
    assert_eq!(
        guard.check(&target(2, back).with_anchor(), 0, Some(&START)),
        Verdict::Accept
    );
    assert_eq!(guard.previous(), back);
    // An anchor is not a licence: one beyond the arm's own 5 cm is still refused. With the
    // defaults the two bounds coincide, so the step is what names it.
    let mut guard = leading(0.4);
    let reason = refused(guard.check(
        &target(1, moved(0.2, 0.0, 0.0)).with_anchor(),
        0,
        Some(&START),
    ));
    assert!(
        matches!(reason, Reason::Step(m) if (m - 0.2).abs() < 1e-12),
        "{reason:?}"
    );
    assert_eq!(guard.previous(), moved(0.4, 0.0, 0.0));
    // Nor does it do anything without a reference: there is nothing to anchor on.
    let reason = refused(guard.check(&target(1, back).with_anchor(), 0, None));
    assert!(matches!(reason, Reason::Step(_)), "{reason:?}");
}

#[test]
fn the_tighter_of_the_two_bounds_refuses_an_anchored_target() {
    // A lead limit under `max_step` is the only configuration in which the anchored path can
    // be refused for the lead rather than the step; pin that it is, so the claim that an
    // anchored target is bounded by the smaller of the two is tested and not merely asserted.
    let options = GuardOptions {
        max_lead: 0.03,
        max_lead_rotation: 0.2,
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, moved(0.4, 0.0, 0.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    assert_eq!(
        guard.check(
            &target(1, moved(0.029, 0.0, 0.0)).with_anchor(),
            0,
            Some(&START)
        ),
        Verdict::Accept
    );
    let reason = refused(guard.check(
        &target(2, moved(0.04, 0.0, 0.0)).with_anchor(),
        0,
        Some(&START),
    ));
    assert!(
        matches!(reason, Reason::Lead(m) if (m - 0.04).abs() < 1e-12),
        "{reason:?}"
    );
    // The rotational bound likewise: 0.2 rad passes, 0.25 does not, though both are inside
    // `max_step_rotation` of 0.26.
    let mut guard = Guard::new(options, about_z(1.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    assert_eq!(
        guard.check(&target(1, about_z(0.19)).with_anchor(), 0, Some(&START)),
        Verdict::Accept
    );
    let reason = refused(guard.check(&target(2, about_z(0.25)).with_anchor(), 0, Some(&START)));
    assert!(
        matches!(reason, Reason::LeadRotation(a) if (a - 0.25).abs() < 1e-9),
        "{reason:?}"
    );
}

#[test]
fn the_anchor_is_ignored_while_the_lead_limits_are_off() {
    // With no lead bound, honouring the anchor would make it the one way to command a jump of
    // any size: `max_step` from a measured pose that may be anywhere. So it is not honoured,
    // and a stream that jumped stays refused until it walks back in.
    assert!(guard().lead_bounded());
    let options = GuardOptions {
        max_lead: 0.0,
        max_lead_rotation: 0.0,
        ..GuardOptions::default()
    };
    let mut guard = Guard::new(options, moved(0.4, 0.0, 0.0)).unwrap();
    guard.set_holder(HOLDER, 0);
    assert!(!guard.lead_bounded());
    let reason = refused(guard.check(&target(1, START).with_anchor(), 0, Some(&START)));
    assert!(
        matches!(reason, Reason::Step(m) if (m - 0.4).abs() < 1e-12),
        "{reason:?}"
    );
}

#[test]
fn an_anchor_on_every_message_cannot_walk_the_target_away_from_a_moving_arm() {
    let max_lead = GuardOptions::default().max_lead;
    let mut guard = guard();
    // The arm crawls along x at 3 mm per message (0.15 m/s at 50 Hz) while a greedy commander
    // anchors every message and asks for 5 cm beyond its own last accepted target. The target
    // advances only as the arm does: the invariant holds after every message, accepted or not,
    // and the total gain over 50 messages is the arm's travel plus one lead, not 2.5 m.
    let mut arm = START;
    for seq in 1..=50u64 {
        let greedy = guard.previous()[0] - START[0] + 0.05;
        let msg = target(seq, moved(greedy, 0.0, 0.0)).with_anchor();
        let verdict = guard.check(&msg, 0, Some(&arm));
        assert!(
            distance(&guard.previous(), &arm) <= max_lead + 1e-12,
            "{seq}: {verdict:?} left the target {:.4} m from the arm",
            distance(&guard.previous(), &arm)
        );
        arm[0] += 0.003;
    }
    assert!(guard.previous()[0] > START[0], "the stream never moved");
    assert!(guard.previous()[0] <= arm[0] + max_lead);
}

#[test]
fn the_examples_sine_passes_the_defaults() {
    // 2.4 mm per message at 20 Hz and 0.8 mm at 100 Hz, against the 7 to 11 mm of tracking
    // error such streams have.
    for step in [0.0024, 0.0008] {
        let mut guard = guard();
        let (mut target_x, lag) = (0.0, 0.011);
        for seq in 1..=15 {
            target_x += step;
            let arm = moved(target_x - lag, 0.0, 0.0);
            let verdict = guard.check(&target(seq, moved(target_x, 0.0, 0.0)), 0, Some(&arm));
            assert_eq!(verdict, Verdict::Accept, "step {step} seq {seq}");
        }
    }
}
