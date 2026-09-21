//! Joints mode and the `home` verb against the fake.

use super::*;
use crate::arm::{HOME_SETTLE_TICKS, HOME_TIMEOUT};
use crate::msg::FLAG_JOINTS;

fn joints_request(client: u32) -> CmdRequest {
    CmdRequest {
        mode: Some(Kind::Joints),
        ..CmdRequest::new(client)
    }
}

fn home_request(client: u32, speed: Option<f64>) -> CmdRequest {
    CmdRequest {
        speed,
        ..CmdRequest::new(client)
    }
}

fn joint_target(seq: u64, q: [f64; 7]) -> TargetMsg {
    TargetMsg::new(Kind::Joints, CLIENT, seq, 1000 + seq, q)
}

impl Rig {
    fn acquire(&mut self) {
        self.machine.handle(Event::LeaseAlive(CLIENT));
        self.ok(Verb::Acquire, CLIENT);
    }

    /// Acquired → Homing; the home query's receiver, still empty.
    fn homing(&mut self, speed: Option<f64>) -> Receiver<CmdReply> {
        self.acquire();
        let reply = self.request(Verb::Home, home_request(CLIENT, speed));
        assert!(
            reply.try_recv().is_err(),
            "home answered before it finished"
        );
        assert_eq!(self.phase(), Phase::Homing);
        reply
    }

    /// The fake's targets so far.
    fn targets(&self) -> Vec<[f64; 7]> {
        self.fake.targets.lock().unwrap().clone()
    }
}

#[test]
fn enable_defaults_to_cartesian_and_takes_joints() {
    let mut rig = rig();
    rig.activate();
    assert_eq!(rig.fake.calls().last(), Some(&"start"));
    let state = rig.state();
    assert_eq!(state.flags.get(), 0);
    assert_eq!(state.target.map(|v| v.get()), START);
    rig.ok(Verb::Stop, CLIENT);

    let reply = rig
        .request(Verb::Enable, joints_request(CLIENT))
        .recv()
        .unwrap();
    assert_eq!(reply, CmdReply::ok());
    assert_eq!(rig.phase(), Phase::Active);
    assert_eq!(rig.fake.calls().last(), Some(&"start_joints"));
    let state = rig.state();
    assert_eq!(state.flags.get(), FLAG_JOINTS);
    assert_eq!(state.target.map(|v| v.get()), START_Q);
    let fractions = rig.fake.fractions.lock().unwrap().clone();
    assert!((fractions[0].0 - 0.2).abs() < 1e-9, "{fractions:?}");
    assert_eq!(
        fractions[0].1,
        JointTargetControlOptions::default().max_deviation
    );

    let mut q = START_Q;
    q[6] += 0.1;
    rig.machine.handle(Event::Target(joint_target(1, q), 0));
    assert_eq!(rig.fake.targets.lock().unwrap().as_slice(), [q]);
    let state = rig.state();
    assert_eq!(state.accepted.get(), 1);
    assert_eq!(state.target.map(|v| v.get()), q);
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.state().flags.get(), 0);
}

#[test]
fn the_other_kind_is_refused_in_either_mode() {
    let mut rig = rig();
    rig.activate();
    rig.machine
        .handle(Event::Target(joint_target(1, START_Q), 0));
    assert_eq!(rig.state().refused.get(), 1);
    assert!(rig.fake.targets.lock().unwrap().is_empty());
    rig.ok(Verb::Stop, CLIENT);
    rig.request(Verb::Enable, joints_request(CLIENT))
        .recv()
        .unwrap();
    rig.target(1, moved(0.01));
    assert_eq!(rig.state().refused.get(), 1);
    assert_eq!(rig.state().accepted.get(), 0);
    assert!(rig.fake.targets.lock().unwrap().is_empty());
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn home_runs_a_joints_session_to_ready_and_answers_when_there() {
    let mut rig = rig();
    assert!(rig.err(Verb::Home, CLIENT).contains("not the holder"));
    // A Cartesian session first, so the counters have something to start over from.
    rig.activate();
    rig.target(1, moved(0.01));
    assert_eq!(rig.state().accepted.get(), 1);
    rig.ok(Verb::Stop, CLIENT);
    rig.fake.ready_after.store(3, Ordering::SeqCst);
    let reply = rig.homing(Some(0.4));
    assert_eq!(rig.fake.calls().last(), Some(&"start_joints"));
    let fractions = rig.fake.fractions.lock().unwrap().clone();
    assert!((fractions[0].0 - 0.4).abs() < 1e-9, "{fractions:?}");
    // 0.3 rad of travel plus the margin is under the config's 1 rad.
    assert_eq!(fractions[0].1, 1.0);
    assert_eq!(rig.targets(), [READY]);
    let state = rig.state();
    assert_eq!(state.flags.get(), FLAG_JOINTS);
    assert_eq!(state.target.map(|v| v.get()), READY);
    assert_eq!(state.client_id.get(), CLIENT);
    assert_eq!((state.accepted.get(), state.seq_accepted.get()), (0, 0));

    // A target, a second home, an enable and a release are refused meanwhile.
    rig.machine.handle(Event::Target(joint_target(1, READY), 0));
    assert_eq!(rig.state().refused.get(), 1);
    assert_eq!(rig.err(Verb::Home, CLIENT), "not allowed in homing");
    assert_eq!(rig.err(Verb::Enable, CLIENT), "not allowed in homing");
    assert_eq!(rig.err(Verb::Release, CLIENT), "not allowed in homing");
    assert_eq!(rig.err(Verb::Recover, CLIENT), "not allowed in homing");

    for _ in 0..3 {
        rig.machine.tick();
        assert_eq!(rig.phase(), Phase::Homing);
        assert!(reply.try_recv().is_err());
    }
    rig.machine.tick();
    assert_eq!(reply.recv().unwrap(), CmdReply::ok());
    assert_eq!(rig.phase(), Phase::Acquired);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert!(rig.phases().contains(&Phase::Stopping));
    // Arrived: no re-target before the stop.
    assert_eq!(rig.targets(), [READY]);
    let state = rig.state();
    assert_eq!(state.client_id.get(), CLIENT);
    assert_eq!(state.flags.get(), 0);
    assert_eq!(state.q.map(|v| v.get()), READY);
    // A `home` is a session like any other, so it is its own episode: the Cartesian pair
    // above, then this one.
    let samples = rig.episodes.lock().unwrap().clone();
    let phases: Vec<EpisodePhase> = samples.iter().map(|s| s.phase).collect();
    use EpisodePhase::{End, Start};
    assert_eq!(phases, [Start, End, Start, End], "{samples:?}");
    assert_ne!(samples[2].recording_id, samples[0].recording_id);
    assert_eq!(samples[3].recording_id, samples[2].recording_id);
}

#[test]
fn home_arrives_after_the_settle_ticks_while_the_joints_still_move() {
    let mut rig = rig();
    rig.fake.ready_after.store(0, Ordering::SeqCst);
    rig.fake.moving_at_ready.store(true, Ordering::SeqCst);
    let reply = rig.homing(None);
    for _ in 1..HOME_SETTLE_TICKS {
        rig.machine.tick();
        assert_eq!(rig.phase(), Phase::Homing);
    }
    rig.machine.tick();
    assert_eq!(reply.recv().unwrap(), CmdReply::ok());
    assert_eq!(rig.phase(), Phase::Acquired);
    assert_eq!(rig.targets(), [READY]);

    // Leaving the tolerance restarts the count.
    rig.fake.moving_at_ready.store(true, Ordering::SeqCst);
    rig.fake.ready_after.store(0, Ordering::SeqCst);
    let reply = rig.homing(None);
    for _ in 1..HOME_SETTLE_TICKS {
        rig.machine.tick();
    }
    rig.fake.ready_after.store(1, Ordering::SeqCst);
    rig.machine.tick();
    assert_eq!(rig.phase(), Phase::Homing);
    assert_eq!(rig.machine.homing.as_ref().unwrap().within, 0);
    for _ in 1..HOME_SETTLE_TICKS {
        rig.machine.tick();
        assert_eq!(rig.phase(), Phase::Homing);
    }
    rig.machine.tick();
    assert_eq!(reply.recv().unwrap(), CmdReply::ok());
}

#[test]
fn home_speed_defaults_and_is_clamped() {
    let mut rig = rig();
    rig.fake.ready_after.store(0, Ordering::SeqCst);
    for (speed, fraction) in [(None, 0.2), (Some(0.9), 0.5), (Some(0.01), 0.05)] {
        let reply = rig.homing(speed);
        rig.machine.tick();
        assert_eq!(reply.recv().unwrap(), CmdReply::ok());
        let fractions = rig.fake.fractions.lock().unwrap().clone();
        assert!(
            (fractions.last().unwrap().0 - fraction).abs() < 1e-9,
            "{speed:?}: {fractions:?}"
        );
    }
    rig.acquire();
    let reply = rig
        .request(Verb::Home, home_request(CLIENT, Some(f64::NAN)))
        .recv()
        .unwrap();
    assert_eq!(reply, CmdReply::err("speed must be finite"));
    assert_eq!(rig.phase(), Phase::Acquired);
}

#[test]
fn home_is_allowed_from_acquired_only() {
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.ok(Verb::Enable, CLIENT);
    assert_eq!(rig.err(Verb::Home, CLIENT), "not allowed in active");
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.err(Verb::Home, CLIENT), "not allowed in idle");
    assert_eq!(rig.err(Verb::Home, OTHER), "client 9 is not the holder");
}

#[test]
fn home_times_out_stops_and_reports() {
    let mut rig = rig();
    let reply = rig.homing(None);
    rig.machine.tick();
    assert!(reply.try_recv().is_err());
    rig.machine.homing.as_mut().unwrap().started -= HOME_TIMEOUT;
    rig.machine.tick();
    assert_eq!(reply.recv().unwrap(), CmdReply::err("home timed out"));
    assert_eq!(rig.phase(), Phase::Acquired);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert_eq!(rig.state().client_id.get(), CLIENT);
    assert_eq!(rig.targets(), [READY, START_Q]);
}

#[test]
fn stop_during_homing_ends_it_and_answers_both() {
    let mut rig = rig();
    let reply = rig.homing(None);
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(reply.recv().unwrap(), CmdReply::err("home stopped"));
    assert_eq!(rig.phase(), Phase::Acquired);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert_eq!(rig.state().flags.get(), 0);
    rig.ok(Verb::Enable, CLIENT);
    assert_eq!(rig.phase(), Phase::Active);
}

#[test]
fn stop_during_homing_decelerates_in_place() {
    let mut rig = rig();
    let _reply = rig.homing(None);
    rig.ok(Verb::Stop, CLIENT);
    // The loop was re-targeted to the reported configuration, not left heading for READY.
    let q = rig.fake.state().q;
    assert_eq!(q, START_Q);
    assert_eq!(rig.targets(), [READY, q]);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
}

#[test]
fn lease_lost_during_homing_stops_and_clears_the_holder() {
    let mut rig = rig();
    let reply = rig.homing(None);
    rig.machine.handle(Event::LeaseLost(OTHER));
    assert_eq!(rig.phase(), Phase::Homing);
    rig.machine.handle(Event::LeaseLost(CLIENT));
    assert_eq!(reply.recv().unwrap(), CmdReply::err("lease lost"));
    assert_eq!(rig.phase(), Phase::Idle);
    assert_eq!(rig.state().client_id.get(), 0);
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert_eq!(rig.targets(), [READY, START_Q]);
}

#[test]
fn a_loop_that_ends_during_homing_faults_and_answers() {
    let mut rig = rig();
    let reply = rig.homing(None);
    rig.fake.running.store(false, Ordering::SeqCst);
    rig.machine.tick();
    assert_eq!(
        reply.recv().unwrap(),
        CmdReply::err("the control loop ended")
    );
    assert_eq!(rig.phase(), Phase::Faulted);
    rig.ok(Verb::Recover, CLIENT);
    assert_eq!(rig.phase(), Phase::Idle);
}

#[test]
fn a_failing_stop_after_homing_faults_and_reports_it() {
    let mut rig = rig();
    rig.fake.ready_after.store(0, Ordering::SeqCst);
    rig.fake.stop_fails.store(true, Ordering::SeqCst);
    let reply = rig.homing(None);
    rig.machine.tick();
    assert_eq!(reply.recv().unwrap(), CmdReply::err("aborted"));
    assert_eq!(rig.phase(), Phase::Faulted);
}

#[test]
fn shutdown_during_homing_answers_shutting_down() {
    let mut rig = rig();
    let reply = rig.homing(None);
    rig.sender.send(Event::Shutdown);
    rig.machine.run(rig.rx);
    assert_eq!(reply.recv().unwrap(), CmdReply::err("shutting down"));
    assert_eq!(rig.fake.calls().last(), Some(&"stop"));
    assert_eq!(
        rig.fake.targets.lock().unwrap().as_slice(),
        [READY, START_Q]
    );
}

/// Every end of a joints session re-targets the loop to the measured configuration first, so
/// it decelerates where the arm is rather than on towards a target that may lead it (a joints
/// session has no lead bound). A Cartesian session stops on its last target, which `max_lead`
/// bounds.
#[test]
fn a_joints_session_stops_where_the_arm_is() {
    let mut ahead = START_Q;
    ahead[6] += 0.1;
    for end in ["stop", "watchdog", "lease lost", "shutdown"] {
        let mut rig = rig();
        rig.acquire();
        let reply = rig.request(Verb::Enable, joints_request(CLIENT));
        assert_eq!(reply.recv().unwrap(), CmdReply::ok());
        rig.machine.handle(Event::Target(joint_target(1, ahead), 0));
        match end {
            "stop" => rig.ok(Verb::Stop, CLIENT),
            "watchdog" => {
                rig.machine.last_target -= Duration::from_secs(1);
                rig.machine.tick();
            }
            "lease lost" => rig.machine.handle(Event::LeaseLost(CLIENT)),
            _ => {
                rig.sender.send(Event::Shutdown);
                rig.machine.run(rig.rx);
                let targets = rig.fake.targets.lock().unwrap().clone();
                assert_eq!(targets, [ahead, START_Q], "{end}");
                continue;
            }
        }
        assert_eq!(rig.phase(), Phase::Idle, "{end}");
        assert_eq!(rig.fake.calls().last(), Some(&"stop"), "{end}");
        assert_eq!(rig.targets(), [ahead, START_Q], "{end}");
    }

    let mut rig = rig();
    rig.activate();
    rig.target(1, moved(0.01));
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.targets(), [moved(0.01)]);
}

/// An arm measured inside the limits' inset is held at the inset's edge, which `set_joints`
/// accepts, not at its own configuration, which it would refuse.
#[test]
fn a_joints_stop_inside_the_limit_margin_holds_at_its_edge() {
    let (lower, upper) = franka::robot::target_control::joint_position_limits(FciVersion::V10);
    let inset = arm_config().guard_options().joint_limit_inset;
    let mut inside = START_Q;
    inside[0] = lower[0] + 0.25 * inset;
    inside[6] = upper[6] - 0.5 * inset;
    let mut edge = inside;
    edge[0] = lower[0] + inset;
    edge[6] = upper[6] - inset;

    let mut rig = rig();
    rig.acquire();
    let reply = rig.request(Verb::Enable, joints_request(CLIENT));
    assert_eq!(reply.recv().unwrap(), CmdReply::ok());
    *rig.fake.q.lock().unwrap() = Some(inside);
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.targets(), [edge]);

    // The early end of a `home` shares the hold.
    *rig.fake.q.lock().unwrap() = None;
    let _reply = rig.homing(None);
    *rig.fake.q.lock().unwrap() = Some(inside);
    rig.ok(Verb::Stop, CLIENT);
    assert_eq!(rig.targets(), [READY, edge]);
}

#[test]
fn client_zero_never_holds_the_arm() {
    let mut rig = rig();
    // Nobody holds the arm, so holder and client are both 0.
    assert_eq!(rig.err(Verb::Acquire, 0), "no lease");
    assert_eq!(rig.err(Verb::Enable, 0), "client 0 is not the holder");
    assert_eq!(rig.err(Verb::Release, 0), "client 0 is not the holder");
    assert_eq!(rig.err(Verb::GripperHome, 0), "client 0 is not the holder");
    // Acquired without a holder is out of the verbs' reach; `home` refuses it all the same.
    rig.machine.phase = Phase::Acquired;
    assert_eq!(rig.err(Verb::Home, 0), "client 0 is not the holder");
    assert_eq!(rig.phase(), Phase::Acquired);
    assert!(rig.fake.calls().is_empty(), "{:?}", rig.fake.calls());
    assert!(rig.episodes.lock().unwrap().is_empty());
}
